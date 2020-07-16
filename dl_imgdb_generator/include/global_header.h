#ifndef ROSCV_TEST_GLOBAL_HEADER_H
#define ROSCV_TEST_GLOBAL_HEADER_H

// using vector type data
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <vector>

// essential header for ROS-OpenCV operation
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

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

// for kitty DB
#define KITDB_IMGFILE_RESIZER 11
#define KITDB_XMLFILE_GENERATOR 22
#define KITDB_XMLFILE_CHECKER 33

using namespace std;
using namespace ros;
using namespace cv;

// for convenience
using json = nlohmann::json;

typedef struct {
  string strLabel;
  int nRGB[3];
} AnnoDB;

typedef struct {
  string strLabel;
  vector<Rect> vecBbox;
} BboxDB;

typedef struct {
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
