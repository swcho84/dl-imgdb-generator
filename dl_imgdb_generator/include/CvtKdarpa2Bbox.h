#ifndef DL_IMGDB_GENERATOR_CVTKDARPA2BBOX_H
#define DL_IMGDB_GENERATOR_CVTKDARPA2BBOX_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;

class CvtKdarpa2Bbox
{
public:
  CvtKdarpa2Bbox(const ConfigParam& cfg);
  ~CvtKdarpa2Bbox();

  void MainLoopImgResizer();
  void MainLoopBboxGenerator();
  void MainLoopBboxChecker();

  bool GetSizeCalcFlag();

  bool bSizeCalcFlag;

private:
  ConfigParam cfgParam_;
  ColorStatus colorStat_;

  bool GenSizeCalcFlag(int nSize, int nTotal);

  int nHeight;
  int nWidth;
};

#endif