#include "CvtOpenDb2Bbox.h"

using namespace std;
using namespace ros;
using namespace cv;

CvtOpenDb2Bbox::CvtOpenDb2Bbox(const ConfigParam& cfg) : cfgParam_(cfg)
{
  bSizeCalcFlag = false;
}

CvtOpenDb2Bbox::~CvtOpenDb2Bbox()
{
}

// main loop: xml file checker
void CvtOpenDb2Bbox::MainLoopBboxChecker()
{
  // assigning variables for browsing annotated images recursively
  vector<String> vecCvtImgFileNm;
  vector<String> vecCvtXmlFileNm;
  glob(cfgParam_.strOpenDBImgResFolderPath, vecCvtImgFileNm, true);
  glob(cfgParam_.strOpenDBLabelResFolderPath, vecCvtXmlFileNm, true);

  // browsing annotated images recursively
  for (size_t k = 0; k < vecCvtImgFileNm.size(); k++)
  {
    // for debugging
    ROS_INFO("Processing_xmlCheck(%d,%d)", (int)(k), (int)(vecCvtImgFileNm.size()));

    // assigning the raw image
    Mat imgRaw = imread(vecCvtImgFileNm[k]);

    // for debugging
    ROS_INFO("[%d]Imgfile:%s", (int)(k), vecCvtImgFileNm[k].c_str());
    ROS_INFO("[%d]Xmlfile:%s", (int)(k), vecCvtXmlFileNm[k].c_str());

    // loading xml file
    TiXmlDocument docXml;
    docXml.LoadFile(vecCvtXmlFileNm[k]);
    TiXmlElement* root = docXml.FirstChildElement("annotation");

    // parsing bbox data with the label in xml file
    for (TiXmlElement* obj = root->FirstChildElement("object"); obj != NULL; obj = obj->NextSiblingElement("object"))
    {
      TiXmlElement* name = obj->FirstChildElement("name");
      const char* label = (const char*)(name->GetText());

      TiXmlElement* bndbox = obj->FirstChildElement("bndbox");
      TiXmlElement* xminElem = bndbox->FirstChildElement("xmin");
      const char* xmin = (const char*)(xminElem->GetText());
      int nXmin = atoi(xmin);

      TiXmlElement* yminElem = bndbox->FirstChildElement("ymin");
      const char* ymin = (const char*)(yminElem->GetText());
      int nYmin = atoi(ymin);

      TiXmlElement* xmaxElem = bndbox->FirstChildElement("xmax");
      const char* xmax = (const char*)(xmaxElem->GetText());
      int nXmax = atoi(xmax);

      TiXmlElement* ymaxElem = bndbox->FirstChildElement("ymax");
      const char* ymax = (const char*)(ymaxElem->GetText());
      int nYmax = atoi(ymax);

      Point ptTl, ptBr;
      ptTl.x = nXmin;
      ptTl.y = nYmin;
      ptBr.x = nXmax;
      ptBr.y = nYmax;
      rectangle(imgRaw, ptTl, ptBr, colorStat_.GetColorStatus((int)(colorStat_.mapColorInfo[label])), 2);

      // for debugging
      ROS_INFO("label(%s):tl(%d,%d),br(%d,%d)", label, nXmin, nYmin, nXmax, nYmax);
    }

    // for debugging
    imshow("raw", imgRaw);

    // pausing and destroying all imshow result
    waitKey(0);
  }
}

// main loop: xml file generator
void CvtOpenDb2Bbox::MainLoopBboxGenerator()
{
  // 2nd, reading, matching, resizing raw bbox data and saving resized bbox data w.r.t the xml type
  vector<String> vecTxtFileNm;
  glob(cfgParam_.strOpenDBLabelSrcFolderPath, vecTxtFileNm, true);

  vector<String> vecImgFileNm;
  glob(cfgParam_.strOpenDBImgSrcFolderPath, vecImgFileNm, true);

  // browsing annotated images recursively
  vecOpDrDBs.clear();
  for (size_t i = 0; i < vecTxtFileNm.size(); i++)
  {
    // assigning the raw image
    Mat imgRaw = imread(vecImgFileNm[i]);

    // image width and height info.
    Size szImgRaw;
    Size szImgRes;
    szImgRaw.height = imgRaw.rows;
    szImgRaw.width = imgRaw.cols;
    szImgRes.height = cfgParam_.nOpDbHeightRef;
    szImgRes.width = cfgParam_.nOpDbWidthRef;

    // reading txt file
    ifstream openFile(vecTxtFileNm[i]);
    if (openFile.is_open())
    {
      // parsing opensource DB label into vector-list type
      string strLine;
      vector<OpenDroneDB> tempOpDrDbVec;

      // parsing line-by-line
      while (getline(openFile, strLine))
      {
        istringstream strStrmLine(strLine);
        string strToken;
        int nFlag = 0;
        OpenDroneDB opDrDB;

        // parsing space-by-space
        while (getline(strStrmLine, strToken, ' '))
        {
          switch (nFlag)
          {
            case 0:
            {
              for (auto i = 0; i < cfgParam_.vecOpDbLabels.size(); i++)
              {
                if ((atoi(strToken.c_str())) == (cfgParam_.vecOpDbLabels[i].nLabel))
                {
                  opDrDB.droneLabel = cfgParam_.vecOpDbLabels[i];
                  opDrDB.nLabel = i;
                }
              }
              break;
            }
            case 1:
            {
              opDrDB.fBboxSrc[0] = (atof(strToken.c_str()));
              break;
            }
            case 2:
            {
              opDrDB.fBboxSrc[1] = (atof(strToken.c_str()));
              break;
            }
            case 3:
            {
              opDrDB.fBboxSrc[2] = (atof(strToken.c_str()));
              break;
            }
            case 4:
            {
              opDrDB.fBboxSrc[3] = (atof(strToken.c_str()));
              break;
            }
          }
          nFlag++;
        }

        // converting the bbox info from opensource to xml type
        opDrDB.bboxStdInfo = CalcBboxInfoXmlType(opDrDB, cfgParam_.nOpDbTxtCalcCase, szImgRaw, szImgRes);
        ROS_INFO("(%d,%d,%d,%d)", opDrDB.bboxStdInfo.nPtXLt, opDrDB.bboxStdInfo.nPtYLt, opDrDB.bboxStdInfo.nPtXRb,
                 opDrDB.bboxStdInfo.nPtYRb);

        // saving parsing result w.r.t space
        tempOpDrDbVec.push_back(opDrDB);
      }

      // saving parsing result w.r.t line
      vecOpDrDBs.push_back(tempOpDrDbVec);
      openFile.close();
    }

    // for debugging
    ROS_INFO("Processing_xmlGen(%d,%d)", (int)(i), (int)(vecTxtFileNm.size()));

    // making the filename  using stringstream, with the numbering rule
    stringstream strStreamXmlFileName;
    strStreamXmlFileName << cfgParam_.strOpenDBLabelResFileNmFwd;
    strStreamXmlFileName << std::setfill('0') << std::setw(cfgParam_.nOpDbXmlResFileNmDigit)
                         << (i + cfgParam_.nOpDbOffsetNum);
    strStreamXmlFileName << "." + cfgParam_.strOpenDBXmlResExt;

    // making the full file path
    string strCvtXmlFile;
    strCvtXmlFile = cfgParam_.strOpenDBLabelResFolderPath + strStreamXmlFileName.str();

    // declarating xml file
    TiXmlDocument docXml;

    // w.r.t pascal VOC xml file
    TiXmlElement* pRoot = new TiXmlElement("annotation");
    docXml.LinkEndChild(pRoot);

    TiXmlElement* pElem0 = new TiXmlElement("folder");
    TiXmlText* txtElem0 = new TiXmlText("VOC2017");
    pElem0->LinkEndChild(txtElem0);
    pRoot->LinkEndChild(pElem0);

    TiXmlElement* pElem1 = new TiXmlElement("filename");
    TiXmlText* txtElem1 = new TiXmlText(strStreamXmlFileName.str());
    pElem1->LinkEndChild(txtElem1);
    pRoot->LinkEndChild(pElem1);

    TiXmlElement* pElem2 = new TiXmlElement("source");
    TiXmlElement* pElem21 = new TiXmlElement("database");
    TiXmlText* txtElem21 = new TiXmlText("KARI-KAIST anti-drone recognition DB");
    pElem21->LinkEndChild(txtElem21);
    TiXmlElement* pElem22 = new TiXmlElement("annotation");
    TiXmlText* txtElem22 = new TiXmlText("PASCAL VOC2017");
    pElem22->LinkEndChild(txtElem22);
    pElem2->LinkEndChild(pElem21);
    pElem2->LinkEndChild(pElem22);
    pRoot->LinkEndChild(pElem2);

    TiXmlElement* pElem3 = new TiXmlElement("owner");
    TiXmlElement* pElem31 = new TiXmlElement("institute");
    TiXmlText* txtElem31 = new TiXmlText("KAIST-KIRoboticsCenter-CJU");
    pElem31->LinkEndChild(txtElem31);
    TiXmlElement* pElem32 = new TiXmlElement("name");
    TiXmlText* txtElem32 = new TiXmlText("Prof.S.Cho");
    pElem32->LinkEndChild(txtElem32);
    pElem3->LinkEndChild(pElem31);
    pElem3->LinkEndChild(pElem32);
    pRoot->LinkEndChild(pElem3);

    TiXmlElement* pElem4 = new TiXmlElement("size");
    TiXmlElement* pElem41 = new TiXmlElement("width");
    TiXmlText* txtElem41 = new TiXmlText(to_string(cfgParam_.nOpDbWidthRef));
    pElem41->LinkEndChild(txtElem41);
    TiXmlElement* pElem42 = new TiXmlElement("height");
    TiXmlText* txtElem42 = new TiXmlText(to_string(cfgParam_.nOpDbHeightRef));
    pElem42->LinkEndChild(txtElem42);
    TiXmlElement* pElem43 = new TiXmlElement("depth");
    TiXmlText* txtElem43 = new TiXmlText("3");
    pElem43->LinkEndChild(txtElem43);
    pElem4->LinkEndChild(pElem41);
    pElem4->LinkEndChild(pElem42);
    pElem4->LinkEndChild(pElem43);
    pRoot->LinkEndChild(pElem4);

    TiXmlElement* pElem5 = new TiXmlElement("segmented");
    TiXmlText* txtElem5 = new TiXmlText("0");
    pElem5->LinkEndChild(txtElem5);
    pRoot->LinkEndChild(pElem5);

    // making xml file
    for (auto kk = 0; kk < vecOpDrDBs[i].size(); kk++)
    {
      TiXmlElement* pElem5 = new TiXmlElement("object");
      TiXmlElement* pElem51 = new TiXmlElement("name");
      TiXmlText* txtElem51 = new TiXmlText(vecOpDrDBs[i][kk].droneLabel.strLabel);
      pElem51->LinkEndChild(txtElem51);
      TiXmlElement* pElem52 = new TiXmlElement("pose");
      TiXmlText* txtElem52 = new TiXmlText("Left");
      pElem52->LinkEndChild(txtElem52);
      TiXmlElement* pElem53 = new TiXmlElement("truncated");
      TiXmlText* txtElem53 = new TiXmlText("1");
      pElem53->LinkEndChild(txtElem53);
      TiXmlElement* pElem54 = new TiXmlElement("difficult");
      TiXmlText* txtElem54 = new TiXmlText("0");
      pElem54->LinkEndChild(txtElem54);

      TiXmlElement* pElem55 = new TiXmlElement("bndbox");
      TiXmlElement* pElem551 = new TiXmlElement("xmin");
      TiXmlText* txtElem551 = new TiXmlText(to_string(vecOpDrDBs[i][kk].bboxStdInfo.nPtXLt));
      pElem551->LinkEndChild(txtElem551);
      TiXmlElement* pElem552 = new TiXmlElement("ymin");
      TiXmlText* txtElem552 = new TiXmlText(to_string(vecOpDrDBs[i][kk].bboxStdInfo.nPtYLt));
      pElem552->LinkEndChild(txtElem552);
      TiXmlElement* pElem553 = new TiXmlElement("xmax");
      TiXmlText* txtElem553 = new TiXmlText(to_string(vecOpDrDBs[i][kk].bboxStdInfo.nPtXRb));
      pElem553->LinkEndChild(txtElem553);
      TiXmlElement* pElem554 = new TiXmlElement("ymax");
      TiXmlText* txtElem554 = new TiXmlText(to_string(vecOpDrDBs[i][kk].bboxStdInfo.nPtYRb));
      pElem554->LinkEndChild(txtElem554);
      pElem55->LinkEndChild(pElem551);
      pElem55->LinkEndChild(pElem552);
      pElem55->LinkEndChild(pElem553);
      pElem55->LinkEndChild(pElem554);

      pElem5->LinkEndChild(pElem51);
      pElem5->LinkEndChild(pElem52);
      pElem5->LinkEndChild(pElem53);
      pElem5->LinkEndChild(pElem54);
      pElem5->LinkEndChild(pElem55);
      pRoot->LinkEndChild(pElem5);
    }

    // saving xml file
    docXml.SaveFile(strCvtXmlFile);

    // calculating size flag
    bSizeCalcFlag = GenSizeCalcFlag(i, (int)(vecImgFileNm.size()));
  }

  return;
}

// main loop: img file resizer
void CvtOpenDb2Bbox::MainLoopImgResizer()
{
  // 1st, resizing raw image and saving resized images
  // assigning variables for browsing annotated images recursively
  vector<String> vecImgFileNm;
  glob(cfgParam_.strOpenDBImgSrcFolderPath, vecImgFileNm, true);

  // browsing annotated images recursively
  for (size_t i = 0; i < vecImgFileNm.size(); i++)
  {
    // for debugging
    ROS_INFO("Processing_imgResize(%d,%d)", (int)(i), (int)(vecImgFileNm.size()));

    // assigning the raw image
    Mat imgRaw = imread(vecImgFileNm[i]);

    // image width and height info.
    nHeight = imgRaw.rows;
    nWidth = imgRaw.cols;

    // resizing w.r.t the cityscapesDB
    Mat imgResize;
    resize(imgRaw, imgResize, Size(cfgParam_.nOpDbWidthRef, cfgParam_.nOpDbHeightRef), 0, 0, INTER_NEAREST);

    // making the filename  using stringstream, with the numbering rule
    stringstream strStreamImgFileName;
    strStreamImgFileName << cfgParam_.strOpenDBImgResFileNmFwd;
    strStreamImgFileName << std::setfill('0') << std::setw(cfgParam_.nOpDbImgResFileNmDigit)
                         << (i + cfgParam_.nOpDbOffsetNum);
    strStreamImgFileName << "." + cfgParam_.strOpenDBImgResExt;

    // making the full file path
    string strCvtImgFile;
    strCvtImgFile = cfgParam_.strOpenDBImgResFolderPath + strStreamImgFileName.str();

    // saving the resized image
    imwrite(strCvtImgFile, imgResize);

    // // calculating size flag
    bSizeCalcFlag = GenSizeCalcFlag(i, (int)(vecImgFileNm.size()));

    // for debugging
    // imshow("imgRaw", imgRaw);
    // imshow("imgCvtRaw", imgResize);

    // pausing and destroying all imshow result
    // waitKey(0);
  }

  return;
}

// calculating the bbox info from opensource to xml
// default: (label) (x_left_top) (y_left_top) (width) (height)
// case1: normalized w.r.t the image size, (label) (x_cen) (y_cen) (width) (height)
// case2: (label) (ptYLt) (ptXLt) (ptYRb) (ptXRb)
BboxStdInfo CvtOpenDb2Bbox::CalcBboxInfoXmlType(OpenDroneDB src, int nTypeFlag, Size szImgSrc, Size szImgRes)
{
  BboxStdInfo res;
  switch (nTypeFlag)
  {
    case 1:  // yolo txt type
    {
      res.nPtXLt = (int)(((((src.fBboxSrc[0]) * (float)(szImgSrc.width)) -
                           ((src.fBboxSrc[2]) * (float)(szImgSrc.width) * (0.5f))) /
                          (szImgSrc.width)) *
                         (szImgRes.width));
      res.nPtYLt = (int)(((((src.fBboxSrc[1]) * (float)(szImgSrc.height)) -
                           ((src.fBboxSrc[3]) * (float)(szImgSrc.width) * (0.5f))) /
                          (szImgSrc.height)) *
                         (szImgRes.height));
      res.nPtXRb = (res.nPtXLt) + (int)((src.fBboxSrc[2]) * (float)(szImgRes.width));
      res.nPtYRb = (res.nPtYLt) + (int)((src.fBboxSrc[3]) * (float)(szImgRes.height));
      res.nBboxWidth = (int)((src.fBboxSrc[2]) * (float)(szImgRes.width));
      res.nBboxHeight = (int)((src.fBboxSrc[3]) * (float)(szImgRes.height));
      break;
    }
    case 2:  // custom type
    {
      res.nPtXLt = (int)(((src.fBboxSrc[1]) / ((float)(szImgSrc.width))) * ((float)(szImgRes.width)));
      res.nPtYLt = (int)(((src.fBboxSrc[0]) / ((float)(szImgSrc.height))) * ((float)(szImgRes.height)));
      res.nPtXRb = (int)(((src.fBboxSrc[3]) / ((float)(szImgSrc.width))) * ((float)(szImgRes.width)));
      res.nPtYRb = (int)(((src.fBboxSrc[2]) / ((float)(szImgSrc.height))) * ((float)(szImgRes.height)));
      res.nBboxWidth = (res.nPtXRb) - (res.nPtXLt);
      res.nBboxHeight = (res.nPtYRb) - (res.nPtYLt);
      break;
    }
    default:  // default type
    {
      res.nPtXLt = (int)(((src.fBboxSrc[0]) / ((float)(szImgSrc.width))) * ((float)(szImgRes.width)));
      res.nPtYLt = (int)(((src.fBboxSrc[1]) / ((float)(szImgSrc.height))) * ((float)(szImgRes.height)));
      res.nPtXRb = (res.nPtXLt) + (int)(((src.fBboxSrc[2]) / ((float)(szImgSrc.width))) * ((float)(szImgRes.width)));
      res.nPtYRb = (res.nPtYLt) + (int)(((src.fBboxSrc[3]) / ((float)(szImgSrc.height))) * ((float)(szImgRes.height)));
      res.nBboxWidth = (int)(((src.fBboxSrc[2]) / ((float)(szImgSrc.width))) * ((float)(szImgRes.width)));
      res.nBboxHeight = (int)(((src.fBboxSrc[3]) / ((float)(szImgSrc.height))) * ((float)(szImgRes.height)));
      break;
    }
  }
  return res;
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