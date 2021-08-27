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
  void MainLoopBboxChecker();
  void MainLoopBboxYoloLabelConverter();

  bool GetSizeCalcFlag();

  bool bSizeCalcFlag;

private:
  ConfigParam cfgParam_;
  ColorStatus colorStat_;

  vector<vector<OpenDroneDB>> vecOpDrDBs;

  bool GenSizeCalcFlag(int nSize, int nTotal);
  BboxStdInfo CalcBboxInfoXmlType(OpenDroneDB opDrDB, int nTypeFlag, Size szImgSrc, Size szImgRes);

  int nHeight;
  int nWidth;
};

#endif