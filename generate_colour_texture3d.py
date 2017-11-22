#!/usr/bin/env python

import numpy as np

from skimage.color import hsv2rgb, rgb2hsv
import skimage.io as io

for idx, hue in enumerate(np.arange(0, 1, 0.01)):
    colour1 = hsv2rgb(np.broadcast_to((hue, 1, 0.5), (1, 1, 3)))*255
    colour2 = hsv2rgb(np.broadcast_to((1-hue, 1, 0.5), (1, 1, 3)))*255

    patch = np.concatenate((np.tile(colour1, (200, 100, 1)), np.tile(colour2, (200, 100, 1))), axis=1)
    
    io.imsave('bin/data/colour_texture_split/colour_text{0:03}.tif'.format(idx), patch.astype(np.uint8))