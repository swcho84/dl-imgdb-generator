#include "CvtOpenDb2Bbox.h"

using namespace std;
using namespace ros;
using namespace cv;

CvtOpenDb2Bbox::CvtOpenDb2Bbox(const ConfigParam& cfg) : cfgParam_(cfg)
{
  // default: VGA size
  nHeight = 480;
  nWidth = 640;

  bSizeCalcFlag = false;
}

CvtOpenDb2Bbox::~CvtOpenDb2Bbox()
{
}

// main loop: xml file checker
void CvtOpenDb2Bbox::MainLoopBboxChecker()
{
  return;
}

// main loop: xml file generator
void CvtOpenDb2Bbox::MainLoopBboxGenerator()
{
  return;
}

// main loop: img file resizer
void CvtOpenDb2Bbox::MainLoopImgResizer()
{
  return;
}

// get size calculation flag
bool CvtOpenDb2Bbox::GetSizeCalcFlag()
{
  return bSizeCalcFlag;
}

// generating size calculation flag for terminating converter
bool CvtOpenDb2Bbox::GenSizeCalcFlag(int nSize, int nTotal)
{
  bool bRes = false;
  if (fabs(nSize - nTotal) == 1.0f)
    bRes = true;
  return bRes;
}