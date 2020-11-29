#ifndef ROSCV_TEST_CVTSEG2BBOX_H
#define ROSCV_TEST_CVTSEG2BBOX_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;

class CvtSeg2Bbox
{
public:
  CvtSeg2Bbox(const ConfigParam& cfg);
  ~CvtSeg2Bbox();

  void MainLoopImgResizer();
  void MainLoopBboxGenerator();
  void MainLoopBboxChecker();
  void MainLoopSemanticSegLabelConverter();
  void MainLoopBboxYoloLabelConverter();
  bool GetSizeCalcFlag();

  bool bSizeCalcFlag;

private:
  ConfigParam cfgParam_;
  ColorStatus colorStat_;

  bool GenSizeCalcFlag(int nSize, int nTotal);
  Mat GenFilteredImg(Mat imgIn, int nHeight, int nWidth, int nAnno, int nTrial);
  Mat CannyEdge(Mat imgIn, int nThresh);
  vector<Rect> GenBboxData(Mat imgIn, Scalar color, int nThresh);

  Mat GetImgFromFile(string strBaseImgName);
  ImgSize GetImgSize(Mat imgInput);
  Mat GetImgTargetResized(Mat imgTarget, ImgSize imgTargetSize, ImgSize imgBaseSize, float fWidthRatio,
                          float fHeightRatio);
  int GenRandNum(int nSize);
  Point GetRngPtTlForTargetResized(ImgSize imgTargetResizedSize, ImgSize imgBaseSize, float fRatio);
  vector<SelectRGB> GetMaskInfo(Mat imgTargetResized, ImgSize imgTargetResizedSize);
  Mat GetImgMix(Mat imgInput, vector<SelectRGB> vecInput, Point ptInput, string strCmd);
  vector<Rect> GetTargetRect(Mat imgInput);
  static bool sortArea(cv::Rect rect1, cv::Rect rect2);

  int nHeight;
  int nWidth;

  Mat imgBase_;
  ImgSize imgBaseSize_;

  Mat imgTarget_;
  ImgSize imgTargetSize_;

  Mat imgTargetResized_;
  ImgSize imgTargetResizedSize_;

  Point ptRndTargetResizedPos_;
  vector<SelectRGB> vecSelectPixelMask_;

  Mat imgForMix_;
  Mat imgForContour_;
  Mat imgMixed_;

  vector<Rect> vecRectTarget_;
};

#endif