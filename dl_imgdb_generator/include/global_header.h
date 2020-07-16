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

using namespace std;
using namespace ros;
using namespace cv;

// for convenience
using json = nlohmann::json;

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

#endif
