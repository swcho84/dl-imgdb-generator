import numpy as np
from scipy import io
import cv2
import os
from shutil import copyfile

# selecting the reference path
basicPath = os.path.join(os.getcwd(), "Database1")

# collecting image/text files and parsing the file name for making the folder
fileList = os.listdir(basicPath)
fileImgListWext= [file for file in fileList if file.endswith(".JPEG")]
fileTxtListWext= [file for file in fileList if file.endswith(".txt")]
fileImgNameList = []
for i in range(len(fileImgListWext)):
  fileInfo = os.path.splitext(fileImgListWext[i])
  fileImgNameList.append(fileInfo[0])
print(fileImgNameList)
fileTxtNameList = []
for i in range(len(fileTxtListWext)):
  fileInfo = os.path.splitext(fileTxtListWext[i])
  fileTxtNameList.append(fileInfo[0])
print(fileImgNameList)

for i in range(len(fileImgListWext)):
  fileSrc = os.path.join(basicPath, fileImgListWext[i])
  fileDst = os.path.join(basicPath, "imgs" + "/" + fileImgListWext[i])
  copyfile(fileSrc, fileDst)

for i in range(len(fileTxtListWext)):
  fileSrc = os.path.join(basicPath, fileTxtListWext[i])
  fileDst = os.path.join(basicPath, "labels" + "/" + fileTxtListWext[i])
  copyfile(fileSrc, fileDst)