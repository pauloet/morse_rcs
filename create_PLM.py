#! /usr/bin/env python3
# Run this script with sudo
import gdal
import numpy
import sys
from gdalconst import *

def main():
	
	# When 'plm_r1.tif' was opened, it was a copy of 'dsm.50cm.tif'
	dataset = gdal.Open('plm_r2.tif', GA_Update)    # Read/Write Access
	if dataset is None:
		print('Input File could not be opened.')
		sys.exit(1)
	else:
		ncols = dataset.RasterXSize
		nrows = dataset.RasterYSize
		nbands = dataset.RasterCount    # = 1
		band = dataset.GetRasterBand(1) # For my case, each pixel will have only a float value          
		datatype = gdal.GetDataTypeName(band.DataType)        # 'float32'
		print('Size (cols x rows x bands) is: ', ncols, 'x', nrows, 'x', nbands, '(', datatype, ')')
		# ReadAsArray(x_offset, y_offset, x_size, y_size)                       
		data = band.ReadAsArray(0, 0, ncols, nrows); print('Initial data: \n', data)
		step = 0.75
		row = numpy.arange(0, step*ncols, step, 'float32')

		# Modify data as I wanted
		new_data = numpy.array([row,]*nrows)
		print("Done! Saving data...\n", new_data)

		# Write Data in the output file                
		band.WriteArray(new_data, 0, 0)

	# Closing Variables
	dataset = None
	band = None

# ----------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    main()
