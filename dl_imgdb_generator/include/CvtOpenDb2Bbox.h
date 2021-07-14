#ifndef DL_IMBDB_GENERATOR_CVOPENDB2BBOX_H
#define DL_IMBDB_GENERATOR_CVOPENDB2BBOX_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;

class CvtOpenDb2Bbox
{
public:
  CvtOpenDb2Bbox(const ConfigParam& cfg);
  ~CvtOpenDb2Bbox();

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

  int nHeight;
  int nWidth;
};

#endif