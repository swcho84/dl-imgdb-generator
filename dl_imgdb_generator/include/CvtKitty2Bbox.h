#ifndef ROSCV_TEST_CVTKTTIY2BBOX_H
#define ROSCV_TEST_CVTKTTIY2BBOX_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;

class CvtKtt2Bbox
{
public:
  CvtKtt2Bbox(const ConfigParam& cfg);
  ~CvtKtt2Bbox();

  void MainLoopImgResizer();
  void MainLoopBboxGenerator();
  void MainLoopBboxGeneratorV2();
  void MainLoopBboxChecker();

  bool GetSizeCalcFlag();

  bool bSizeCalcFlag;

private:
  ConfigParam cfgParam_;
  ColorStatus colorStat_;

  bool GenSizeCalcFlag(int nSize, int nTotal);

  vector<vector<KittyDB>> vecKittyDB;

  int nHeight;
  int nWidth;
};

#endif