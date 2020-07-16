#ifndef ROSCV_TEST_CVTKTTIY2BBOX_H
#define ROSCV_TEST_CVTKTTIY2BBOX_H

#include "config_param.h"
#include "global_header.h"

using namespace std;
using namespace ros;

class CvtKtt2Bbox {
public:
  CvtKtt2Bbox(const ConfigParam &cfg);
  ~CvtKtt2Bbox();

  void MainLoopImgResizer();
  void MainLoopBboxGenerator();

  bool GetSizeCalcFlag();

  bool bSizeCalcFlag;

private:
  ConfigParam cfgParam_;
  ColorStatus colorStat_;

  bool GenSizeCalcFlag(int nSize, int nTotal);

  vector<vector<KittyDB>> vecKittyDB;

  int nHeight;
  int nWidth;

  int nHeightCityScapeDB;
  int nWidthCityScapeDB;
};

#endif