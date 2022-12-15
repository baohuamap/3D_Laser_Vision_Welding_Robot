
import glob
import re

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [ atoi(c) for c in re.split(r'(\d+)', text) ]


Images = glob.glob('D:/Workspace/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/MyCamera/Scan_data2/weldseam_' + '*.jpg')
print('before ', Images)

Images.sort(key=natural_keys)
print('after ', Images)
