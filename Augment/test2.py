# from cv2 import transform
import pandas as pd
import cv2 as cv

CYAN = 255,255,0
GREEN = 0,255,0

# bbox size 20 x 20
W, H = 10, 10
TRACKER_POS = 50, 50

SHOW_IMG = False

IMG_DIR         = 'D:/Code/Welding_Robot/DataSet/Augmented/'
AUGMENT_IMG_DIR = 'D:/Code/Welding_Robot/DataSet/Augmented'
ENV_DATASET_DIR = 'D:/Code/Welding_Robot/DataSet/Env/Annotations.csv'

def show_img_with_label(img, label):
    # label = [bbox pos, tracker pos]
    try:
    # draw bbox post
        cv.rectangle(img, (label[0][0]-W,label[0][1]-H), (label[0][0]+W,label[0][1]+H), CYAN, 1)
        cv.circle(img, label[0], 1, CYAN, 1)
        # draw tracker pos
        cv.rectangle(img, (label[1][0]-W, label[1][0]-H), (label[1][0]+W, label[1][0]+H), GREEN, 1)
        cv.circle(img, label[1], 1, GREEN, 1)
    except:
        pass
    # show img
    cv.namedWindow('img with label', cv.WINDOW_NORMAL)
    cv.imshow('img with label', img)
    cv.waitKey(0)
    cv.destroyAllWindows()


# load dataset
df = pd.read_csv('D:/Code/Welding_Robot/DataSet/Env/Annotations.csv')

img_name = df.iloc[13524]['filename']
img_tracker_pos = eval(df.iloc[13524]['region_shape_attributes'])
# bbox and tracker position
bbox_pos = img_tracker_pos['cx'], img_tracker_pos['cy']
#keypoints
keypoints = [bbox_pos, TRACKER_POS]
#read image
img_path = IMG_DIR + img_name
image = cv.imread(img_path)


show_img_with_label(image, keypoints)
