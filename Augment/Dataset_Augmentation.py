# from cv2 import transform
import pandas as pd
import cv2 as cv
import albumentations as A
from tqdm import tqdm

CYAN = 255,255,0
GREEN = 0,255,0

# bbox size 20 x 20
W, H = 10, 10
TRACKER_POS = 50, 50

SHOW_IMG = False

IMG_DIR         = 'D:/Code/Welding_Robot/DataSet/Env_Imgs_resized/'
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

# Augmentations transformations
transform = A.Compose([
    A.HorizontalFlip(p=0.5),    
    A.Rotate(limit=30, p=0.75),
    A.OneOf([
        A.IAAAdditiveGaussianNoise(p=0.9),
        A.GaussNoise(p=0.6),
    ], p=0.25),
    A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
], keypoint_params= A.KeypointParams(format='xy'))

# load dataset
df = pd.read_csv('D:/Code/Welding_Robot/3D_Laser_Vision_Welding_Robot/Augment/ImgAndLabel.csv')
rows = []
count = 4708

for i in tqdm(range(4707)):
    img_name = df.iloc[i]['filename']
    img_tracker_pos = eval(df.iloc[i]['region_shape_attributes'])
    # bbox and tracker position
    bbox_pos = img_tracker_pos['cx'], img_tracker_pos['cy']
    #keypoints
    keypoints = [bbox_pos, TRACKER_POS]
    #read image
    img_path = IMG_DIR + img_name
    image = cv.imread(img_path)

    for j in range(2):
        transformed = transform(image=image, keypoints=keypoints)
        transformed_image = transformed['image']
        transformed_keypoints = transformed['keypoints']
        file_name = f'frame-{count}.jpg'
        region_shape_attributes = {'name':'point','cx': transformed_keypoints[0][0], 'cy': transformed_keypoints[0][1]}
        rows.append({'filename': file_name, 'region_shape_attributes': str(region_shape_attributes)})
        # df.append(df_row)
        cv.imwrite(f'{AUGMENT_IMG_DIR}/{file_name}', transformed_image)
        count += 1

        if SHOW_IMG:
            show_img_with_label(transformed_image, transformed_keypoints)

df_rows = pd.DataFrame(rows)
final_df = pd.concat([df, df_rows], ignore_index=True)
final_df.to_csv(ENV_DATASET_DIR)