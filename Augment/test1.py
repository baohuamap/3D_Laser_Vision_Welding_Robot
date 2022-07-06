import albumentations as A
import cv2 as cv

transform = A.Compose([
    A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=1.0),
])

image = cv.imread("D:/Code/Welding_Robot/DataSet/Env_Imgs_resized/frame-0.jpg")
# image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

transformed_image_1 = transform(image=image)['image']

cv.namedWindow('trans', cv.WINDOW_NORMAL)
cv.imshow('trans',transformed_image_1)
cv.waitKey(0)

# transformed_image = transformed["image"]

# another_transformed_image = transform(image=another_image)["image"]
