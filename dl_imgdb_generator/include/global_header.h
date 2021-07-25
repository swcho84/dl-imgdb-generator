#ifndef ROSCV_TEST_GLOBAL_HEADER_H
#define ROSCV_TEST_GLOBAL_HEADER_H

// using vector type data
#include <iostream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <ctime>
#include <vector>
#include <dirent.h>
#include <fstream>
#include <random>

// essential header for ROS-OpenCV operation
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

// for using json file parser
#include <nlohmann/json.hpp>

// for using xml file writer
#include <tinyxml.h>

// for using color matrix
#include "colormat.h"

// for using boost tokenizer
#include <boost/tokenizer.hpp>

// for cityscape DB
#define CITYDB_IMGFILE_RESIZER 1
#define CITYDB_XMLFILE_GENERATOR 2
#define CITYDB_XMLFILE_CHECKER 3
#define KARIDB_SEMANTIC_SEGMENTATION_LABEL_CONVERTER 4
#define ETRIDB_BBOX_DB_YOLO_CONVERTER 5

// for kitty DB
#define KITDB_IMGFILE_RESIZER 11
#define KITDB_XMLFILE_GENERATOR 22
#define KITDB_XMLFILE_CHECKER 33

// for opensource DB
#define OPENDB_IMGFILE_RESIZER 111
#define OPENDB_XMLFILE_GENERATOR 222
#define OPENDB_XMLFILE_CHECKER 333

using namespace std;
using namespace ros;
using namespace cv;

// for convenience
using json = nlohmann::json;

typedef struct
{
  int nPtXLt;
  int nPtYLt;
  int nPtXRb;
  int nPtYRb;
  int nBboxWidth;
  int nBboxHeight;
} BboxStdInfo;

typedef struct
{
  string strLabel;
  int nLabel;
  string strColor;
} OpenDroneDBlabel;

typedef struct
{
  OpenDroneDBlabel droneLabel;
  int nLabel;
  float fBboxSrc[4];
  BboxStdInfo bboxStdInfo;
} OpenDroneDB;

typedef struct
{
  string strLabel;
  int nRGB[3];
} AnnoDB;

typedef struct
{
  string strLabel;
  vector<Rect> vecBbox;
} BboxDB;

typedef struct
{
  string strLabel;
  int nTruncated;
  int nOcculded;
  float fAlphaAngRad;
  float fBbox[4];
  float fObjDimM[3];
  float fObjLocM[3];
  float fRotAngRad;
} KittyDB;

typedef struct
{
  Point ptPixel;
  uchar blue;
  uchar green;
  uchar red;
} SelectRGB;

typedef struct
{
  int nWidth;
  int nHeight;
} ImgSize;

#endif
