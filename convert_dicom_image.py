#!/usr/bin/env python

import dicom
import glob
import numpy as np
import os
import Tkinter, tkFileDialog
import time

import skimage.io as io

from operator import itemgetter

while True:
	dirname = raw_input("Type new directory name: ")

	if os.path.exists("bin/data/" + dirname):
		print("Directory already exists, choose another name")
	else:
		os.mkdir("bin/data/" + dirname)
		break

root = Tkinter.Tk()
root.withdraw()

# Browse to directory containing dicom images
file_path = tkFileDialog.askdirectory()
#file_path = "/home/vrterra/Desktop/REMBRANDT/900-00-1961/06-19-2005-04452/4-32199"
#file_path = "/home/vrterra/Desktop/CT Lymph Nodes/ABD_LYMPH_001/09-14-2014-ABDLYMPH001-abdominallymphnodes-30274/abdominallymphnodes-26828"
start_time = time.time()

dicom_files = glob.glob(file_path + "/*.dcm")

img = dicom.read_file(dicom_files[0])

all_imgs = np.zeros((img.Rows, img.Columns, len(dicom_files)), dtype=float)
locations = np.zeros((len(dicom_files), 1))
echo_numbers = np.zeros((len(dicom_files), 1))


for idx, dicom_file in enumerate(dicom_files):
	img = dicom.read_file(dicom_file)
	# all_imgs[:,:,idx] = img.pixel_array / (1.0*np.max(img.pixel_array))
	all_imgs[:,:,idx] = img.pixel_array
	# locations[idx] = 0
	if 'ImagePositionPatient' in img.trait_names():
		locations[idx] = float(img.ImagePositionPatient[2])
	elif 'Location' in img.trait_names():
		locations[idx] = float(img.Location)
	else:
		raise Exception("Can't determine slice Location, parameters: {}".format(img.trait_names()))
	
	if img.has_key('EchoNumbers'):
		echo_numbers[idx] = img.EchoNumbers
	else:
		echo_numbers[idx] = 2.0

print (np.min(all_imgs.ravel()))
#io.imshow(all_imgs[:,:,0]/ np.max(all_imgs.ravel()))
#io.show()
all_imgs = all_imgs + np.min(all_imgs.ravel())
all_imgs = all_imgs / np.max(all_imgs.ravel())
sorted_idx = np.array([t[0] for t in sorted(enumerate(locations), key=itemgetter(1), reverse=True)])
# There are duplicates, echo numbers 1 and 2? What does this mean?
echo_numbers = echo_numbers[sorted_idx].ravel()

#This was just a test to go through the files and see why there were duplicates
#f1 = dicom_files[sorted_idx[0]]
#f2 = dicom_files[sorted_idx[1]]
#i1 = dicom.read_file(f1)
#i2 = dicom.read_file(f2)

#def printi1and2(dataset, data):
#	tag = data.tag
#	#print(dir(data))
#
#	if data.name.startswith('Pixel Data'):
#		return
#	
#	print(data.name + ":" + str(data.value) + " | " + str(i2[tag].value))
#
#i1.walk(callback=printi1and2)

echo2_img_idx = sorted_idx[np.where(echo_numbers == 2.0)]

for idx, s_idx in enumerate(echo2_img_idx):
	img = np.dstack((all_imgs[:,:,s_idx],all_imgs[:,:,s_idx],all_imgs[:,:,s_idx]))
	#io.imshow(img)
	#io.show()
	io.imsave("bin/data/" + dirname + "/img_{0:06}.tif".format(idx), (img*(255*255)).astype(np.uint16))

print "Processing time = {} seconds".format(time.time() - start_time) 