import os
import requests

### Simple script to scrape global temperature images from NASA and save them as .tif ###
### This is relatively straightforward as the URLs are standardised ###
### https://climate.nasa.gov/interactives/climate-time-machine ###

URL_TEMPLATE = "https://climate.nasa.gov/system/time_series_images/****_gt_****_720x360.jpg"
START_VAL = 1597
START_YEAR = 1884
END_VAL = 1731
END_YEAR = 2018

cwd = os.getcwd()
path = "images\globaltemp"
directory = cwd+"\\"+path

try:
    os.makedirs(path)
except OSError:
    print("Directory creation %s failed" % (directory))
else:
    print("Successfully created directory %s" % (directory))

diff = START_YEAR - START_VAL

for current_val in range(START_VAL, END_VAL+1):
    current_year = current_val+diff
    image_data = ("%d_gt_%d" % (current_val, current_year))
    url = URL_TEMPLATE.replace("****_gt_****", image_data)
    print(url)

    image_path = ((path+"\\%d" + ".tif") % current_year)
    print(image_path)

    cur_image = requests.get(url)
    open(image_path, 'wb').write(cur_image.content)
