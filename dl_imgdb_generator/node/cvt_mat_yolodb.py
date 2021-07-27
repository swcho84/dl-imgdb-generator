import numpy as np
from scipy import io
import os

# selecting the reference path
basicPath = "./26/"
basicLabel = "dji_phantom"

# collecting sorted jpg files
fileList = os.listdir(basicPath)
fileListJpgWext= [file for file in fileList if file.endswith(".jpg")]
sizeFileListJpgWext = len(fileListJpgWext)
fileListJpg = []
for i in range(sizeFileListJpgWext):
  fileInfo = os.path.splitext(fileListJpgWext[i])
  fileListJpg.append(int(fileInfo[0]))
fileListJpgSorted = sorted(fileListJpg)

# parsing bbox data (single drone in the single image)
matFile = io.loadmat(basicPath + "anotation.mat")
listData = list(matFile["box"])
arrData = np.array(listData)
arrSize = len(arrData)

# converting mat data to text data for C/C++ parsing
fileTxt = open(basicPath + "labeled.txt", mode='w', encoding='utf-8')
for i in range(arrSize):
  fileTxt.write(str(fileListJpgSorted[i]) + ".jpg" + " ")
  fileTxt.write(str(basicLabel) + " ")
  fileTxt.write(str(int(arrData[i][0])) + " ")
  fileTxt.write(str(int(arrData[i][1])) + " ")
  fileTxt.write(str(int(arrData[i][2])) + " ")
  fileTxt.write(str(int(arrData[i][3])) + " ")
  fileTxt.write("\n")
fileTxt.close()
