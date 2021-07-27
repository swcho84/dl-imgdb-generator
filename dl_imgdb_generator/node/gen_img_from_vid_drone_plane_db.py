import numpy as np
from scipy import io
import cv2
import os

# selecting the reference path
basicPath = os.path.join(os.getcwd(), "videos/planes")

# collecting video files and parsing the file name for making the folder
fileList = os.listdir(basicPath)
fileVidListAviWext= [file for file in fileList if file.endswith(".avi")]
fileVidNameList = []
for i in range(len(fileVidListAviWext)):
  fileInfo = os.path.splitext(fileVidListAviWext[i])
  fileVidNameList.append(fileInfo[0])
fileVidNameListAviSorted = sorted(fileVidNameList)

# browsing video files for saving each frame (png file)
for i in range(len(fileVidListAviWext)):
  dirSaveImgPath = os.path.join(basicPath, fileVidNameListAviSorted[i])
  vidFilePath = os.path.join(basicPath, fileVidNameListAviSorted[i] + ".avi")
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



