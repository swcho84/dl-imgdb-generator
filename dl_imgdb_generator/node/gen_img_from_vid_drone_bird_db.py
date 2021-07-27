import numpy as np
from scipy import io
import cv2
import os

# selecting the reference path
basicPath = os.path.join(os.getcwd(), "videos")

# collecting video files and parsing the file name for making the folder
fileVidList = os.listdir(basicPath)
fileVidNameList = []
for i in range(len(fileVidList)):
  fileInfo = os.path.splitext(fileVidList[i])
  fileVidNameList.append(fileInfo[0])

# browsing video files for saving each frame (png file)
for i in range(len(fileVidList)):
  dirSaveImgPath = os.path.join(basicPath, fileVidNameList[i])
  vidFilePath = os.path.join(basicPath, fileVidList[i])
  os.mkdir(dirSaveImgPath)
  print("currWork:", dirSaveImgPath)
  if os.path.isfile(vidFilePath):
    cap = cv2.VideoCapture(vidFilePath)
    framePropId = int(cv2.CAP_PROP_FRAME_COUNT)
    length = int(cv2.VideoCapture.get(cap, framePropId))
    for j in range(length):
      print("currWorkFileNum:", j)
      ret, frame = cap.read()
      if ret:
        imgSaveFileName = str(dirSaveImgPath) + "/" + str(j) + ".png"
        cv2.imwrite(imgSaveFileName, frame)
        cv2.waitKey(30)
      else:
          break
  else:
    print("file not found..")
    break



