import pymorse
import math, sys
import logging
import gdal, numpy
from gdalconst import *

""" Install first the Requirements:

git clone git://github.com/OSGeo/gdal.git
cd gdal/gdal/swig/python/
git checkout tags/1.10.0
sudo python3 setup.py install

git clone git://github.com/numpy/numpy.git
cd numpy/
git checkout v1.7.1
sudo python3 setup.py install
"""

# -----------------------------LOGGING CONFIGURATION -----------------------------------
logger = logging.getLogger("morse."+__name__)
logger.setLevel(logging.DEBUG)
# File Handler
fh = logging.FileHandler('robots_communications.log', mode = 'w')
fh.setLevel(logging.DEBUG)
# Console Handler
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
# Formatter for both Handlers
formatter = logging.Formatter('[%(asctime)s (%(levelname)s)]  %(message)s', "%H:%M:%S")
fh.setFormatter(formatter)
ch.setFormatter(formatter)
# Add Handlers to the logger
logger.addHandler(fh)
logger.addHandler(ch)
# --------------------------------------------------------------------------------------


## @brief This class aims to simulate if 2 robots can communicate each other according to specific models.
# @author Paulo Simões
class RCS():
    
    ## @details Tuple including all the communication models available in this module.
    __models = ('distance', 'line_of_sight', 'free_space_loss', 'plm')
    ## @details Default communication model in case it is not specified in the constructor or set_model_specifications() method.
    __default_model = 'distance'
    ## @brief In Morse: 1 unit --> 1 meter
    # @details Default distance threshold in case it is not specified in the constructor or set_model_specifications() method.
    __default_distance_threshold = 10
    ## @brief Units: MHz
    # @details Default frequency in case it is not specified in the constructor or set_model_specifications() method.
    __default_frequency = 800
    ## @brief Units: dB
    # @details Default communication model in case it is not specified in the constructor or set_model_specifications() method.
    __default_free_space_threshold = 45
    ## @brief Dictionary including the default parameters of the Path Loss Map model.
    # @details These values are established in case they are wrongly (or not) specified in the constructor or set_model_specifications() method.
    # @see __check_plm_dictionary()
    __default_plm = {'t1':20, 't2':40, 't3':60, 'dr0':54, 'dr1':42, 'dr2':27, 'dr3':0}
    
    
    ## @param[in] robot_1 Name of one robot.
    # @param[in] robot_2 Name of the other robot.
    # @param[in] kwargs See the method @ref set_model_specifications() for more information.
    # @details Here, the communication model (and specifications) can be established.
    # Default values are established, in case the respective arguments are wrongly (or not) passed.    
    # @details Usage examples:
    # @code r0r1 = rcs.RCS('r0', 'r1') 
	# r0r1 = rcs.RCS('r0', 'r1', model = "free_space_loss")                 
	# r0r1 = rcs.RCS('r0', 'r1', model = "free_space_loss", freq = 800)
	# r0r1 = rcs.RCS('r0', 'r1', model = "free_space_loss", free_space_threshold = 120)
    # ...
    # parameters = {'t1':10, 't2':20, 't3':30, 'dr0':4, 'dr1':3, 'dr2':2, 'dr3':1}
    # r1r2_4 = rcs.RCS('robo1', 'robo2', model = 'plm', plm = parameters)
    # ...
	# r0r1 = rcs.RCS('r0', 'r1', model = "free_space_loss", freq = 800, free_space_threshold = 120)
	# r0r1 = rcs.RCS('r0', 'r1', model = "distance", free_space_threshold = 100) @endcode
	# Of course, the last example doesn't make sense because the model 'distance' is not related with the loss threshold.
	# So, this method has to be used consciously.
	# @see Available models: __models
    def __init__(self, robot_1, robot_2, **kwargs):

        ## @brief Morse connection: access to Morse services and data streams
        self.__morse = pymorse.Morse()
        ## @brief Dictionary with the names of the 2 robots and also the names of their respective Pose sensors
        self.__robot_names = self.__verify_robots_names_and_pose_sensors(robot_1, robot_2)
        ## @brief Name of the communication model applied between the 2 robots
        self.__model = None
        ## @brief In Morse: 1 unit --> 1 meter
        self.__distance_threshold = None
        ## @brief Units: MHz
        self.__frequency = None
        ## @brief Units: dB
        self.__free_space_threshold = None
        ## @brief Path Loss Map parameters (dictionary)
        # @see The mandatory conditions that are checked in this dictionary at __check_plm_dictionary()
        self.__plm = None
        
        self.set_model_specifications(**kwargs)
        
    ## @details This method closes the connection with the  morse simulation.
    def __del__(self):
        self.__morse.close()
        logger.info('Morse socket closed')        

    ## @return 1 (0) if the 2 robots can (cannot) communicate according to the established communication model.
    # @exception For the Path Loss Map model, it returns the Data Rate (Mb/s). 
    def can_communicate(self):
        if self.__model == 'distance':
            return self.__simulate_comm_distance()
        elif self.__model == 'line_of_sight':
            return self.__simulate_comm_lineofsight()
        elif self.__model == 'free_space_loss':
            return self.__simulate_comm_freespaceloss()
        elif self.__model == 'plm':
            return self.__simulate_comm_pathlossmap()
        else:
            logger.debug("An error occurred in the 'Class -> RCS.can_communicate() method.")
            return 0

    ## @brief This method sets a communication model and its specifications between the 2 robots.
    # @param[in] kwargs 'model'
    # @param[in] kwargs 'distance_threshold' (meters)
    # @param[in] kwargs 'freq' (MHz)
    # @param[in] kwargs 'free_space_threshold' (dB)
    # @param[in] kwargs 'plm' This has to be a dictionary containing the Path Loss Map parameters
    # @details Available models: @ref __models
    # @details Default values are established in case the respective arguments are wrongly (or not) passed.
	# @details Usage examples:
	# @code r0r1.set_model_specifications(model = "free_space_loss", freq = 800, free_space_threshold = 120)
	# r0r1.set_model_specifications(freq = 500)
	# ...
	# r0r1.set_model_specifications(model = "distance", free_space_threshold = 100) @endcode
	# Of course, the last example doesn't make sense because the model 'distance' is not related with the loss threshold.
	# So, this method has to be used consciously.
    def set_model_specifications(self, **kwargs):
        
        if ("model" in kwargs) and self.model_exists(kwargs["model"]):
            self.__model = kwargs["model"]
        else:
            if self.__model is None:
                self.__model = self.__default_model
            #else:  Keeping the previous one

        logger.info("Communication model between '%s' and '%s': '%s'"\
                %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), self.__model.upper()))
        #---------------------------------------

        if "distance_threshold" in kwargs:
            self.__distance_threshold = kwargs["distance_threshold"]
        else:
            if self.__distance_threshold is None:
                self.__distance_threshold = self.__default_distance_threshold
            #else: Keeping the previous value
        if self.__model is 'distance': 
            logger.info("Distance Threshold (m): %i" %self.__distance_threshold)
        #---------------------------------------

        if "freq" in kwargs:
            self.__frequency = kwargs["freq"]
        else:
            if self.__frequency is None:
                self.__frequency = self.__default_frequency
            #else: Keeping the previous value
        if self.__model is 'free_space_loss':
            logger.info("Frequency (MHz): %i" %self.__frequency)
        #---------------------------------------
        
        if "free_space_threshold" in kwargs:
            self.__free_space_threshold = kwargs["free_space_threshold"]
        else:
            if self.__free_space_threshold is None:
                self.__free_space_threshold = self.__default_free_space_threshold
            #else: Keeping the previous value
        if self.__model is 'free_space_loss':
            logger.info("Free Space Path Loss Threshold (dB): %s" %self.__free_space_threshold)
        #---------------------------------------
        
        if ("plm" in kwargs) and (self.__check_plm_dictionary(kwargs["plm"])):
                self.__plm = kwargs["plm"]
        else:
            if self.__plm is None:
                self.__plm = self.__default_plm
            #else: Keeping the previous value
        if self.__model is 'plm':
            logger.info("Path Loss Map parameters: %s" %self.__plm)
        #---------------------------------------

    ## @param[in] d Dictionary with the parameters of the Path Loss Map model.
    # @return True (False) if they (do not) satisfy the mandatory conditions.
    def __check_plm_dictionary(self, d):
        
        result = False
        if (len(d) == len(self.__default_plm)) and (d.keys() == self.__default_plm.keys()):
            if (d['t3'] > d['t2'] > d['t1'] > 0):
                if (d['dr0'] > d['dr1'] > d['dr2'] > d['dr3']):
                    result = True

        return result


    ## @param[in] model String containing the communication model.
    # @return True (False) if the model exists (or not) in this module.
    def model_exists(self, model):
        if model.lower() in self.__models:  return True
        else:                               return False

    ## @param[in] spec Boolean to return or not the current model specifications.
    # @return A string with the communication model currently established between the 2 robots and its current specifications if spec is true.
    def get_model_specifications(self, spec):
        if spec is False:
            return self.__model
        else:
            if self.__model is 'distance':
                return "Model: Distance;   Threshold: " + str(self.__distance_threshold)
            elif self.__model is 'line_of_sight':
                return "Model: Line of Sight"
            elif self.__model is 'free_space_loss':
                return "Model: Free Space Loss;   Threshold: " + str(self.__free_space_threshold) \
                        + "(dB)   Frequency: " + str(self.__frequency) + "(MHz)"
            elif self.__model is 'plm':
                return "Model: Path Loss Map;   Parameters: " + str(self.__plm)
    
    ## @return True (False) if the distance between the 2 robots is less (greater or equal) than DISTANCE THRESHOLD.    
    def __simulate_comm_distance(self):
        result = self.__get_distance_and_lineofsight()
        if result[0] < self.__distance_threshold: 
            print("\n\n")
            logger.info("RCS: '%s' & '%s' can communicate, because DISTANCE is: %i (<%i)"\
                    %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), result[0], self.__distance_threshold))
            return 1            
        else:  
            logger.info("RCS: '%s' & '%s' CANNOT communicate, because DISTANCE is: %i (>=%i)"\
                    %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), result[0], self.__distance_threshold))
            return 0

    ## @return True (False) if the 2 robots are (not) in line-of-sight.
    def __simulate_comm_lineofsight(self):
        result = self.__get_distance_and_lineofsight()
        if result[1]:
            logger.info("RCS: '%s' & '%s' can communicate, because LINE-of-SIGHT is: %s"\
                    %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), result[1]))
            return 1
        else:            
            logger.info("RCS: '%s' & '%s' CANNOT communicate, because LINE-of-SIGHT is: %s"\
                    %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), result[1]))
            return 0

    ## @return True (False) if the path loss is less (greater or equal) than FREE SPACE THRESHOLD.
    # @details Formula obtained from @cite paper_2002. 
    # @todo When the robot antennas will be defined on Morse, then get the gain Gt and Gr
    def __simulate_comm_freespaceloss(self):
        Gt = 2; Gr = 2; c = 299792458;
        result = self.__get_distance_and_lineofsight()
        wavelength = float(c/(self.__frequency*math.pow(10, 6)))
        loss = float(-10*math.log10((Gt*Gr*math.pow(wavelength, 2))/math.pow(4*math.pi*result[0], 2)))
        if loss < self.__free_space_threshold: 
            logger.info("RCS: '%s' & '%s' can communicate, because FREE SPACE LOSS is: %i (<%i)"\
                    %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), loss, self.__free_space_threshold))
            return 1
        else:  
            logger.info("RCS: '%s' & '%s' CANNOT communicate, because FREE SPACE LOSS is: %i (>=%i)"\
                    %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), loss, self.__free_space_threshold))
            return 0

    ## @return A list with 2 arguments: distance between the 2 robots, line-of-sight (boolean). 
    def __get_distance_and_lineofsight(self):
        try:
                return self.__morse.rpc('communication', 'distance_and_view', self.__robot_names['r1'], self.__robot_names['r2'])
        except  pymorse.MorseServerError as mse:
            logger.error('Oups! An error occurred!', mse)

    ## @return The Data Rate (Mb/s) of the respective path loss condition according to the current (robot) position.
    # @note The Path Loss Map files must be in the same directory of rcs.py and the file name must have this format: 'plm_'+robotname+'.tif'.
    def __simulate_comm_pathlossmap(self):
        
        filenames = {'r1': 'plm_'+self.__robot_names['r1']+'.tif', 'r2': 'plm_'+self.__robot_names['r2']+'.tif'}
        pathlosses = {'r1': None, 'r2': None}
        for key in filenames:
            dataset = gdal.Open(filenames[key], GA_ReadOnly)
            if dataset is None:
                logger.error('\n\nPath Loss Map File "%s" could not be opened.\nBye...\n\n' %filenames[key])
                sys.exit(1)
            else:
                gdalinfo = {}
                gdalinfo['X_custom_origin'] = float(dataset.GetMetadata()['CUSTOM_X_ORIGIN'])
                gdalinfo['Y_custom_origin'] = float(dataset.GetMetadata()['CUSTOM_Y_ORIGIN'])
                geotransform = dataset.GetGeoTransform()
                gdalinfo['X_utm_origin'] = geotransform[0]
                gdalinfo['Y_utm_origin'] = geotransform[3]
                gdalinfo['X_scale'] = geotransform[1]
                gdalinfo['Y_scale'] = geotransform[5]
                ncols = dataset.RasterXSize
                nrows = dataset.RasterYSize
                band = dataset.GetRasterBand(1) # For this case, each pixel will have only a float32 value
                map_data = band.ReadAsArray(0, 0, ncols, nrows);    
                # debug: print('\n\nData from "%s":\n%s'%(filenames[key], map_data));
                
                temp1 = getattr(self.__morse, self.__robot_names[key])
                if key == 'r1':
                    temp2 = getattr(temp1, self.__robot_names['r1_pose'])
                else:
                    temp2 = getattr(temp1, self.__robot_names['r2_pose'])
                morse_pose_coordinates = {}; pixel = {} 
                morse_pose_coordinates['x'] = temp2.get()['x']
                morse_pose_coordinates['y'] = temp2.get()['y'];     #morse_pose_coordinates['z'] = temp2.get()['z']
                pixel = self.__getPixel_coordinates(gdalinfo, morse_pose_coordinates)
                # Important Numpy note to read an array element: array[row][col]
                pathlosses[key] = map_data[pixel['y']][pixel['x']]
                # debug: print('Gdalinfo: ' + str(gdalinfo), '\n\nMorse Pose Coordinates: ', str(morse_pose_coordinates))
                # debug: print('Pixel coordinates: ', pixel)
                # debug: print('Pathloss of %s: %f' %(key, pathlosses[key]))

                # Closing variables
                dataset = None; band = None; map_data = None;
                X_custom = Y_custom = ncols = nrows = None; 
                gdalinfo = None; morse_pose_coordinates = None
        
        # debug: print('Path Losses: ', pathlosses)
        return self.__get_Data_Rate(max(pathlosses.values()))

    ## @param[in] pl Maximum Path Loss value of the 2 robots.
    # @return The Data Rate (Mb/s) according to the respective path loss condition.
    def __get_Data_Rate(self, pl):
        
        if (pl >= 0) and (pl < self.__plm['t1']):
            result = self.__plm['dr0']
        elif (pl >= self.__plm['t1']) and (pl < self.__plm['t2']):
            result = self.__plm['dr1']
        elif (pl >= self.__plm['t2']) and (pl < self.__plm['t3']):
            result = self.__plm['dr2']
        elif pl >= self.__plm['t3']:
            result = self.__plm['dr3']
        else:
            logger.error("\n\nRCS: The maximum path loss between the robots "\
                    + str(self.__robot_names['r1'].upper()) + ' & ' + str(self.__robot_names['r2'].upper())\
                    + ' is ' + str(pl) + ".\nThere is no data rate condition to deal with this case (< 0).\n\n")
            result = 0
            
        logger.info("RCS: '%s' & '%s' have a Data Rate communication of %.2f (Mb/s), because the maximum Path Loss is: %.2f"\
                %(self.__robot_names['r1'].upper(), self.__robot_names['r2'].upper(), result, pl))
        return result

    ## @param[in] gdal Dictionary containing data coming from Geotiff file.
    # @param[in] morse_pose Dictionary with the coordinates x,y given by morse.
    # @return Dictionary with the pixel coordinates (x,y) to read the path loss in 'map_data'.
    def __getPixel_coordinates(self, gdal, morse_pose):

        # Convert Morse Pose coordinates to Morse Pose UTM coordinates
        x = float(morse_pose['x'] + gdal['X_custom_origin'])
        y = float(morse_pose['y'] + gdal['Y_custom_origin'])

        # Convert UTM to Pixel
        p = {}
        p['x'] = int((x - gdal['X_utm_origin']) / gdal['X_scale'])
        p['y'] = int((y - gdal['Y_utm_origin']) / gdal['Y_scale'])
        return p


    ## @param[in] r1 Name of one robot given in the __init().
    # @param[in] r2 Name of the other robot given in the __init().
    # @return The dictionary self.__robot_names correctly filled up in case of success.
    # @exception Exit if the robot names do not exist in the current Scene 3D.
    # @exception Exit if any pose sensor (1 per robot) was not included.
    # @details The pose sensor name of each robot must have the word "pose" inside - no matter if with upper or lower letters.
    # @note The robots names and pose sensors names are defined in the builder script.
    def __verify_robots_names_and_pose_sensors(self, r1, r2):
       
        r_names = self.__morse.rpc('simulation', 'list_robots')
        streams = self.__morse.rpc('simulation', 'list_streams')
        pose_found = False
        
        # Verify names of the robots
        if (r1 in r_names) and (r2 in r_names):  
            logger.info('Robots Names are correct! :)')
            result = {'r1': r1, 'r2': r2, 'r1_pose': None, 'r2_pose': None}
        elif r1 not in r_names:   
            logger.error('\n\nThe robot name "%s" does not exist in the current Scene.\nBye...\n\n' %r1)
            sys.exit(0)
        elif r2 not in r_names:
            logger.error('\n\nThe robot name "%s" does not exist in the current Scene.\nBye...\n\n' %r2)
            sys.exit(0)

        # Verify if Robot 1 was configured with a pose sensor
        for word in streams:
            if (word.startswith(r1)) and ((word.lower()).find('pose') != -1):
                temp = word.split('.')
                result['r1_pose'] = temp[1]
                pose_found = True
                
        if not pose_found:
            logger.error('\n\nThe robot "%s" was not configured with a POSE sensor.\nBye\n\n' %r1)
            sys.exit(0)
        
        # Verify if Robot 2 was configured with a pose sensor
        pose_found = False
        for word in streams:
            if (word.startswith(r2)) and ((word.lower()).find('pose') != -1):
                temp = word.split('.')
                result['r2_pose'] = temp[1]
                pose_found = True

        if not pose_found:
            logger.error('\n\nThe robot "%s" was not configured with a POSE sensor.\nBye\n\n' %r2)
            sys.exit(0)
        
        logger.info('Robots Pose Sensors found! :)')
        return result
