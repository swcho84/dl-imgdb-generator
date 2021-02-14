#include "CvtKdarpa2Bbox.h"

using namespace std;
using namespace ros;
using namespace cv;

CvtKdarpa2Bbox::CvtKdarpa2Bbox(const ConfigParam& cfg) : cfgParam_(cfg)
{
  // default: VGA size
  nHeight = 480;
  nWidth = 640;

  bSizeCalcFlag = false;
}

CvtKdarpa2Bbox::~CvtKdarpa2Bbox()
{
}

// main loop: xml file checker
void CvtKdarpa2Bbox::MainLoopBboxChecker()
{
  // assigning variables for browsing annotated images recursively
  vector<String> vecCvtImgFileNm;
  vector<String> vecXmlLabelFileNm;
  glob(cfgParam_.strKdarpaImgDstPath, vecCvtImgFileNm, true);
  glob(cfgParam_.strKdarpaXmlDstPath, vecXmlLabelFileNm, true);

  // browsing annotated images recursively
  for (size_t k = 0; k < vecCvtImgFileNm.size(); k++)
  {
    // for debugging
    ROS_INFO("Processing_xmlCheck(%d,%d)", (int)(k), (int)(vecCvtImgFileNm.size()));

    // assigning the raw image
    Mat imgRaw = imread(vecCvtImgFileNm[k]);

    // for debugging
    ROS_INFO("[%d]file:%s", (int)(k), vecCvtImgFileNm[k].c_str());

    // loading xml file
    TiXmlDocument docXml;
    docXml.LoadFile(vecXmlLabelFileNm[k]);
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

      rectangle(imgRaw, ptTl, ptBr, colorStat_.scalRed, 2);

      // for debugging
      ROS_INFO("label(%s):tl(%d,%d),br(%d,%d)", label, nXmin, nYmin, nXmax, nYmax);
    }

    // for debugging
    imshow("raw", imgRaw);

    // pausing and destroying all imshow result
    waitKey(0);
  }

  return;
}

// main loop: xml file generator
void CvtKdarpa2Bbox::MainLoopBboxGenerator()
{
  // 2nd, reading, matching, resizing raw bbox data and saving resized bbox data w.r.t the xml type
  vector<String> vecImgFileNm;
  vector<String> vecCvtImgFileNm;
  vector<String> vecXmlFileNm;
  glob(cfgParam_.strKdarpaImgSrcPath, vecImgFileNm, true);
  glob(cfgParam_.strKdarpaImgDstPath, vecCvtImgFileNm, true);
  glob(cfgParam_.strKdarpaXmlSrcPath, vecXmlFileNm, true);

  // browsing annotated images recursively
  for (size_t k = 0; k < vecXmlFileNm.size(); k++)
  {
    // for debugging
    ROS_INFO("Processing_xmlGen(%d,%d)", (int)(k), (int)(vecXmlFileNm.size()));

    // loading xml file
    TiXmlDocument docXmlSrc;
    docXmlSrc.LoadFile(vecXmlFileNm[k]);
    TiXmlElement* root = docXmlSrc.FirstChildElement("annotation");

    // parsing size data in xml file
    Size szCurrImg;
    for (TiXmlElement* obj = root->FirstChildElement("size"); obj != NULL; obj = obj->NextSiblingElement("size"))
    {
      TiXmlElement* widthElem = obj->FirstChildElement("width");
      const char* width = (const char*)(widthElem->GetText());
      szCurrImg.width = atoi(width);

      TiXmlElement* heightElem = obj->FirstChildElement("height");
      const char* height = (const char*)(heightElem->GetText());
      szCurrImg.height = atoi(height);
    }

    // parsing bbox data with the label in xml file
    vector<KdarpaDB> vecCurrImgXmlInfo;
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

      KdarpaDB tempCurrImgXmlInfo;
      tempCurrImgXmlInfo.strLabel = (string)(label);
      tempCurrImgXmlInfo.nBbox[0] = (int)(((float)(nXmin) / (float)(szCurrImg.width)) * (nWidth));
      tempCurrImgXmlInfo.nBbox[1] = (int)(((float)(nYmin) / (float)(szCurrImg.height)) * (nHeight));
      tempCurrImgXmlInfo.nBbox[2] = (int)(((float)(nXmax) / (float)(szCurrImg.width)) * (nWidth));
      tempCurrImgXmlInfo.nBbox[3] = (int)(((float)(nYmax) / (float)(szCurrImg.height)) * (nHeight));
      vecCurrImgXmlInfo.push_back(tempCurrImgXmlInfo);

      // for debugging
      ROS_INFO("src::label(%s):tl(%d,%d),br(%d,%d)", label, nXmin, nYmin, nXmax, nYmax);
    }

    // resetting the label and bbox info.
    Mat imgResized = imread(vecCvtImgFileNm[k]);
    for (auto i = 0; i < vecCurrImgXmlInfo.size(); i++)
    {
      // resetting the label info.
      if ((vecCurrImgXmlInfo[i].strLabel == "dog_linear") || (vecCurrImgXmlInfo[i].strLabel == "dog_poi"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaDog.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaDog.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaDog.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaDog.nRGB[2];
      }
      else if ((vecCurrImgXmlInfo[i].strLabel == "soldier_linear") ||
               (vecCurrImgXmlInfo[i].strLabel == "soldier_poi") ||
               (vecCurrImgXmlInfo[i].strLabel == "special_soldier_linear") ||
               (vecCurrImgXmlInfo[i].strLabel == "special_soldier_poi") ||
               (vecCurrImgXmlInfo[i].strLabel == "dispatch_soldier_linear") ||
               (vecCurrImgXmlInfo[i].strLabel == "dispatch_soldier_poi") ||
               (vecCurrImgXmlInfo[i].strLabel == "female_soldier_linear") ||
               (vecCurrImgXmlInfo[i].strLabel == "female_soldier_poi") ||
               (vecCurrImgXmlInfo[i].strLabel == "female_special_soldier_linear") ||
               (vecCurrImgXmlInfo[i].strLabel == "female_special_soldier_poi"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaSoldier.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaSoldier.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaSoldier.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaSoldier.nRGB[2];
      }
      else if ((vecCurrImgXmlInfo[i].strLabel == "radiation_mark_linear") || (vecCurrImgXmlInfo[i].strLabel == "radiati"
                                                                                                               "on_"
                                                                                                               "mark_"
                                                                                                               "poi"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaRadiationMark.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaRadiationMark.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaRadiationMark.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaRadiationMark.nRGB[2];
      }
      else if ((vecCurrImgXmlInfo[i].strLabel == "biochem_mark_linear") || (vecCurrImgXmlInfo[i].strLabel == "biochem_"
                                                                                                             "mark_"
                                                                                                             "poi"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaBiochemMark.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaBiochemMark.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaBiochemMark.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaBiochemMark.nRGB[2];
      }
      else if ((vecCurrImgXmlInfo[i].strLabel == "exit_mark_linear") || (vecCurrImgXmlInfo[i].strLabel == "exit_mark_"
                                                                                                          "poi"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaExitMark.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaExitMark.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaExitMark.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaExitMark.nRGB[2];
      }
      else if ((vecCurrImgXmlInfo[i].strLabel == "end_mark_linear") || (vecCurrImgXmlInfo[i].strLabel == "end_mark_"
                                                                                                         "project"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaEndMark.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaEndMark.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaEndMark.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaEndMark.nRGB[2];
      }
      else if ((vecCurrImgXmlInfo[i].strLabel == "start_mark_linear") || (vecCurrImgXmlInfo[i].strLabel == "start_mark_"
                                                                                                           "project"))
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaStartMark.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaStartMark.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaStartMark.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaStartMark.nRGB[2];
      }
      else
      {
        vecCurrImgXmlInfo[i].strLabel = cfgParam_.kdarpaDummy.strLabel;
        vecCurrImgXmlInfo[i].nRGB[0] = cfgParam_.kdarpaDummy.nRGB[0];
        vecCurrImgXmlInfo[i].nRGB[1] = cfgParam_.kdarpaDummy.nRGB[1];
        vecCurrImgXmlInfo[i].nRGB[2] = cfgParam_.kdarpaDummy.nRGB[2];
      }

      ROS_INFO("dst::label(%s):tl(%d,%d),br(%d,%d)", vecCurrImgXmlInfo[i].strLabel.c_str(),
               vecCurrImgXmlInfo[i].nBbox[0], vecCurrImgXmlInfo[i].nBbox[1], vecCurrImgXmlInfo[i].nBbox[2],
               vecCurrImgXmlInfo[i].nBbox[3]);
      rectangle(imgResized, Point(vecCurrImgXmlInfo[i].nBbox[0], vecCurrImgXmlInfo[i].nBbox[1]),
                Point(vecCurrImgXmlInfo[i].nBbox[2], vecCurrImgXmlInfo[i].nBbox[3]),
                Scalar(vecCurrImgXmlInfo[i].nRGB[2], vecCurrImgXmlInfo[i].nRGB[1], vecCurrImgXmlInfo[i].nRGB[0]), 2);
    }

    // making the filename  using stringstream, with the numbering rule
    stringstream strStreamFileName;
    strStreamFileName << cfgParam_.strKdarpaXmlFileNmFwd;
    strStreamFileName << std::setfill('0') << std::setw(cfgParam_.nKdarpaXmlFileNmDigit) << k;
    strStreamFileName << "." + cfgParam_.strKdarpaXmlExt;

    stringstream strStreamFileVisualName;
    strStreamFileVisualName << cfgParam_.strKdarpaXmlFileNmFwd;
    strStreamFileVisualName << std::setfill('0') << std::setw(cfgParam_.nKdarpaXmlFileNmDigit) << k;
    strStreamFileVisualName << "." + cfgParam_.strKdarpaImgExt;

    // making the full file path
    string strXmlFile;
    strXmlFile = cfgParam_.strKdarpaXmlDstPath + "/" + strStreamFileName.str();
    ROS_INFO("[%d]xmlFile:%s", (int)(k), strXmlFile.c_str());

    // declarating xml file
    TiXmlDocument docXmlDst;

    // w.r.t pascal VOC xml file
    TiXmlElement* pRoot = new TiXmlElement("annotation");
    docXmlDst.LinkEndChild(pRoot);

    TiXmlElement* pElem0 = new TiXmlElement("folder");
    TiXmlText* txtElem0 = new TiXmlText("VOC2017");
    pElem0->LinkEndChild(txtElem0);
    pRoot->LinkEndChild(pElem0);

    TiXmlElement* pElem1 = new TiXmlElement("filename");
    TiXmlText* txtElem1 = new TiXmlText(strStreamFileVisualName.str());
    pElem1->LinkEndChild(txtElem1);
    pRoot->LinkEndChild(pElem1);

    TiXmlElement* pElem2 = new TiXmlElement("source");
    TiXmlElement* pElem21 = new TiXmlElement("database");
    TiXmlText* txtElem21 = new TiXmlText("KDARPA Object DB");
    pElem21->LinkEndChild(txtElem21);
    TiXmlElement* pElem22 = new TiXmlElement("annotation");
    TiXmlText* txtElem22 = new TiXmlText("PASCAL VOC2017");
    pElem22->LinkEndChild(txtElem22);
    pElem2->LinkEndChild(pElem21);
    pElem2->LinkEndChild(pElem22);
    pRoot->LinkEndChild(pElem2);

    TiXmlElement* pElem3 = new TiXmlElement("owner");
    TiXmlElement* pElem31 = new TiXmlElement("institute");
    TiXmlText* txtElem31 = new TiXmlText("CJU");
    pElem31->LinkEndChild(txtElem31);
    TiXmlElement* pElem32 = new TiXmlElement("name");
    TiXmlText* txtElem32 = new TiXmlText("Prof. S.Cho");
    pElem32->LinkEndChild(txtElem32);
    pElem3->LinkEndChild(pElem31);
    pElem3->LinkEndChild(pElem32);
    pRoot->LinkEndChild(pElem3);

    TiXmlElement* pElem4 = new TiXmlElement("size");
    TiXmlElement* pElem41 = new TiXmlElement("width");
    TiXmlText* txtElem41 = new TiXmlText(to_string(nWidth));
    pElem41->LinkEndChild(txtElem41);
    TiXmlElement* pElem42 = new TiXmlElement("height");
    TiXmlText* txtElem42 = new TiXmlText(to_string(nHeight));
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
    for (auto kk = 0; kk < vecCurrImgXmlInfo.size(); kk++)
    {
      // reselecting data using the selected label
      TiXmlElement* pElem5 = new TiXmlElement("object");
      TiXmlElement* pElem51 = new TiXmlElement("name");
      TiXmlText* txtElem51 = new TiXmlText(vecCurrImgXmlInfo[kk].strLabel);
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
      TiXmlText* txtElem551 = new TiXmlText(to_string(vecCurrImgXmlInfo[kk].nBbox[0]));
      pElem551->LinkEndChild(txtElem551);
      TiXmlElement* pElem552 = new TiXmlElement("ymin");
      TiXmlText* txtElem552 = new TiXmlText(to_string(vecCurrImgXmlInfo[kk].nBbox[1]));
      pElem552->LinkEndChild(txtElem552);
      TiXmlElement* pElem553 = new TiXmlElement("xmax");
      TiXmlText* txtElem553 = new TiXmlText(to_string(vecCurrImgXmlInfo[kk].nBbox[2]));
      pElem553->LinkEndChild(txtElem553);
      TiXmlElement* pElem554 = new TiXmlElement("ymax");
      TiXmlText* txtElem554 = new TiXmlText(to_string(vecCurrImgXmlInfo[kk].nBbox[3]));
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
    docXmlDst.SaveFile(strXmlFile);

    // calculating size flag
    bSizeCalcFlag = GenSizeCalcFlag(k, (int)(vecImgFileNm.size()));

    // for debugging
    // imshow("imgResized", imgResized);

    // pausing and destroying all imshow result
    // waitKey(0);
  }

  return;
}

// main loop: img file resizer
void CvtKdarpa2Bbox::MainLoopImgResizer()
{
  // 1st, resizing raw image and saving resized images
  // assigning variables for browsing annotated images recursively
  vector<String> vecImgFileNm;
  glob(cfgParam_.strKdarpaImgSrcPath, vecImgFileNm, true);

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
    resize(imgRaw, imgResize, Size(cfgParam_.nKttWidthRef, cfgParam_.nKttHeightRef), 0, 0, INTER_NEAREST);

    // making the filename  using stringstream, with the numbering rule
    stringstream strStreamImgFileName;
    strStreamImgFileName << cfgParam_.strKdarpaImgFileNmFwd;
    strStreamImgFileName << std::setfill('0') << std::setw(cfgParam_.nKdarpaImgFileNmDigit) << (i);
    strStreamImgFileName << "." + cfgParam_.strKdarpaImgExt;

    // making the full file path
    string strCvtImgFile;
    strCvtImgFile = cfgParam_.strKdarpaImgDstPath + "/" + strStreamImgFileName.str();

    // saving the resized image
    imwrite(strCvtImgFile, imgResize);

    // calculating size flag
    bSizeCalcFlag = GenSizeCalcFlag(i, (int)(vecImgFileNm.size()));

    // // for debugging
    // imshow("imgRaw", imgRaw);
    // imshow("imgResize", imgResize);

    // // pausing and destroying all imshow result
    // waitKey(0);
  }

  return;
}

// getting size calculation flag
bool CvtKdarpa2Bbox::GetSizeCalcFlag()
{
  return bSizeCalcFlag;
}

// generating size calculation flag for terminating converter
bool CvtKdarpa2Bbox::GenSizeCalcFlag(int nSize, int nTotal)
{
  bool bRes = false;
  if (fabs(nSize - nTotal) == 1.0f)
    bRes = true;
  return bRes;
}