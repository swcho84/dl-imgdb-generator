import numpy as np
from scipy import io
import cv2

matFile = io.loadmat('/home/drswchorndantidrone/Downloads/drone_image_label/1/anotation.mat')
listData = list(matFile['box'])
arrData = np.array(listData)

fileSrc = '/home/drswchorndantidrone/Downloads/drone_image_label/1/10.jpg'
imgSrc = cv2.imread(fileSrc, cv2.IMREAD_COLOR)
imgDst = imgSrc
cv2.rectangle(img=imgDst, pt1=(int(arrData[0][0]), int(arrData[0][1])), pt2=(int(arrData[0][0]+arrData[0][2]), int(arrData[0][1]+arrData[0][3])), color=(0, 255,0), thickness=3, lineType=cv2.LINE_AA)
cv2.imshow('imgDst', imgDst)

cv2.waitKey(0)
cv2.destroyAllWindows()
