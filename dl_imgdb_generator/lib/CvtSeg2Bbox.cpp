#include "CvtSeg2Bbox.h"

using namespace std;
using namespace ros;
using namespace cv;

CvtSeg2Bbox::CvtSeg2Bbox(const ConfigParam& cfg) : cfgParam_(cfg)
{
  // default: VGA size
  nHeight = 480;
  nWidth = 640;

  bSizeCalcFlag = false;
}

CvtSeg2Bbox::~CvtSeg2Bbox()
{
}

// main loop: seg label converter (from RGB to label)
void CvtSeg2Bbox::MainLoopSemanticSegLabelConverter()
{
  // assigning variables for browsing annotated images recursively
  vector<String> vecSegLabelImgFileNm;
  glob(cfgParam_.strSegLabelImgFolderPath, vecSegLabelImgFileNm, true);

  // browsing annotated images recursively
  for (size_t k = 0; k < vecSegLabelImgFileNm.size(); k++)
  {
    // convert the rgb label w.r.t pixel to the gray label w.r.t pixel

  }  

  return;
}

// main loop: xml file checker
void CvtSeg2Bbox::MainLoopBboxChecker()
{
  // assigning variables for browsing annotated images recursively
  vector<String> vecCvtImgFileNm;
  vector<String> vecXmlLabelFileNm;
  glob(cfgParam_.strCvtImgFolderPath, vecCvtImgFileNm, true);
  glob(cfgParam_.strXmlFolderPath + cfgParam_.strXmlType, vecXmlLabelFileNm, true);

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
void CvtSeg2Bbox::MainLoopBboxGenerator()
{
  // 1st, making polygonDB-based bbox
  // assigning variables for browsing annotated images recursively
  vector<String> vecAnnoFileNm;
  glob(cfgParam_.strAnnoFolderPath, vecAnnoFileNm, true);
  cfgParam_.vecImgBboxDB.clear();

  // browsing annotated images recursively
  for (size_t i = 0; i < vecAnnoFileNm.size(); i++)
  {
    // for debugging
    ROS_INFO("Processing_maskImgDB(%d,%d)", (int)(i), (int)(vecAnnoFileNm.size()));

    // assigning the raw image
    Mat imgRaw = imread(vecAnnoFileNm[i]);

    // image width and height info. (h1024, w2048)
    nHeight = imgRaw.rows;
    nWidth = imgRaw.cols;

    // generating bbox data w.r.t the labelDB
    vector<BboxDB> vecBboxDB;
    for (auto ii = 0; ii < cfgParam_.vecAnnoDB.size(); ii++)
    {
      // generating the filtered image using the label data
      Mat imgFiltered;
      imgFiltered = GenFilteredImg(imgRaw, nHeight, nWidth, ii, cfgParam_.nMorphThresh);

      // detecting canny edge data
      Mat imgCanny;
      imgCanny = CannyEdge(imgFiltered, cfgParam_.nCannyThresh);

      // generating bounding box data
      vector<Rect> vecBbox;
      vecBbox = GenBboxData(
          imgCanny,
          Scalar(cfgParam_.vecAnnoDB[ii].nRGB[2], cfgParam_.vecAnnoDB[ii].nRGB[1], cfgParam_.vecAnnoDB[ii].nRGB[0]),
          cfgParam_.nPolyDPThesh);

      // saving bbox data
      if (vecBbox.size() > 0)
      {
        BboxDB tempBbox;
        tempBbox.vecBbox.clear();
        tempBbox.strLabel = cfgParam_.vecAnnoDB[ii].strLabel;
        for (auto iii = 0; iii < vecBbox.size(); iii++)
          tempBbox.vecBbox.push_back(vecBbox[iii]);

        vecBboxDB.push_back(tempBbox);
      }
    }

    // saving results using vector
    cfgParam_.vecImgBboxDB.push_back(vecBboxDB);
  }

  // for debugging
  ROS_INFO("vecImgBboxDB.size:%d", (int)(cfgParam_.vecImgBboxDB.size()));
  ROS_INFO(" ");

  // 2nd, making maskImg-based bbox
  // assigning variables for browsing polygon data w.r.t json file recursively
  vector<String> vecPolygonFileNm;
  glob(cfgParam_.strPolygonFolderPath, vecPolygonFileNm, true);
  cfgParam_.vecPolygonBboxDB.clear();

  // browsing mask images recursively
  for (size_t k = 0; k < vecPolygonFileNm.size(); k++)
  {
    // for debugging
    ROS_INFO("Processing_polygonDB(%d,%d)", (int)(k), (int)(vecPolygonFileNm.size()));

    // assigning the polygon file
    ifstream polyJson(vecPolygonFileNm[k]);
    json js;
    polyJson >> js;

    // generating polygon data by using the selected label
    vector<BboxDB> vecBboxDB;
    for (auto kk = 0; kk < js["objects"].size(); kk++)
    {
      // for using debugging image
      Mat imgTest = Mat::zeros(Size(nWidth, nHeight), CV_8UC3);

      // w.r.t the selected label
      for (auto kkk = 0; kkk < cfgParam_.vecAnnoDB.size(); kkk++)
      {
        Rect rectBbox;
        BboxDB tempBbox;

        // if the selected label in the objects of JSON file
        if ((js["objects"][kk]["label"] == cfgParam_.vecAnnoDB[kkk].strLabel))
        {
          vector<Point> vecContour;

          // generating polygon data
          for (auto kkkk = 0; kkkk < js["objects"][kk]["polygon"].size(); kkkk++)
          {
            // parsing each point data
            Point tempPt;
            tempPt.x = js["objects"][kk]["polygon"][kkkk][0];
            tempPt.y = js["objects"][kk]["polygon"][kkkk][1];
            vecContour.push_back(tempPt);
          }

          // calculating bounding box information
          if (vecContour.size() > 0)
          {
            rectBbox = boundingRect(vecContour);
            tempBbox.strLabel = cfgParam_.vecAnnoDB[kkk].strLabel;
            tempBbox.vecBbox.push_back(rectBbox);

            vecBboxDB.push_back(tempBbox);
          }
        }
      }
    }

    // saving polygon-based bbox DB
    cfgParam_.vecPolygonBboxDB.push_back(vecBboxDB);
  }

  // for debugging
  ROS_INFO("vecPolygonBboxDB.size:%d", (int)(cfgParam_.vecPolygonBboxDB.size()));
  ROS_INFO(" ");

  // using pet image mixing loop, for ver.2.0
  if (cfgParam_.bPetMix)
  {
    // for debugging
    ROS_INFO("Adding usecase: pet mixing for ver2.0");

    // assigning variables for browsing raw images recursively
    vector<String> vecCvtBaseImgFileNm;
    glob(cfgParam_.strRawFolderPath, vecCvtBaseImgFileNm, true);

    // assigning variables for browsing pet images recursively
    vector<String> vecCvtPetImgFileNm;  // cat and dog
    glob(cfgParam_.strPetImgFolderPath, vecCvtPetImgFileNm, true);
    vecSelectPixelMask_.clear();
    vecRectTarget_.clear();
    cfgParam_.vecPetBboxDB.clear();

    // mixing loop
    for (unsigned int ii = 0; ii < cfgParam_.nTrialPetMix; ii++)
    {
      // for debugging
      ROS_INFO("Processing_petMixImgDB(%d, %d, %d)", (int)(ii), cfgParam_.nTrialPetMix,
               (int)(cfgParam_.vecPetBboxDB.size()));

      // getting the base image
      int nRandomBaseNum = GenRandNum((int)(vecCvtBaseImgFileNm.size()));
      imgBase_ = GetImgFromFile(vecCvtBaseImgFileNm[nRandomBaseNum]);
      imgBaseSize_ = GetImgSize(imgBase_);

      // getting the target image (single type)
      imgTarget_ = GetImgFromFile(vecCvtPetImgFileNm[GenRandNum((int)(vecCvtPetImgFileNm.size()))]);
      imgTargetSize_ = GetImgSize(imgTarget_);

      // calculating the resized tareget image and mask
      imgTargetResized_ =
          GetImgTargetResized(imgTarget_, imgTargetSize_, imgBaseSize_, cfgParam_.fWidthRatio, cfgParam_.fHeightRatio);
      imgTargetResizedSize_ = GetImgSize(imgTargetResized_);
      ptRndTargetResizedPos_ = GetRngPtTlForTargetResized(imgTargetResizedSize_, imgBaseSize_, cfgParam_.fInnerRatio);
      vecSelectPixelMask_ = GetMaskInfo(imgTargetResized_, imgTargetResizedSize_);

      // calculating the mask, contour and mixed image
      imgForMix_ = GetImgMix(imgBase_, vecSelectPixelMask_, ptRndTargetResizedPos_, "black");
      imgForContour_ = GetImgMix(imgBase_, vecSelectPixelMask_, ptRndTargetResizedPos_, "contour");
      imgMixed_ = GetImgMix(imgForMix_, vecSelectPixelMask_, ptRndTargetResizedPos_, "rgb");

      // calculating the resized image
      Mat imgMixedResize;
      resize(imgMixed_, imgMixedResize, Size(640, 480));

      // saving resized imgs (from No. ~~~ to No. ~~~)
      // making the filename  using stringstream, with the numbering rule
      stringstream strStreamImgFileName;
      strStreamImgFileName << cfgParam_.strImgFileNmFwd;
      strStreamImgFileName << std::setfill('0') << std::setw(cfgParam_.nImgFileNmDigit) << (ii + 7481);
      strStreamImgFileName << "." + cfgParam_.strImgExt;

      // making the full file path
      string strCvtImgFile;
      strCvtImgFile = cfgParam_.strCvtPetMixImgFolderPath + strStreamImgFileName.str();

      // saving the resized image
      imwrite(strCvtImgFile, imgMixedResize);

      // calculating the resized bounding rectangle information
      vecRectTarget_ = GetTargetRect(imgForContour_);
      sort(vecRectTarget_.begin(), vecRectTarget_.end(), &sortArea);
      rectangle(imgMixedResize, vecRectTarget_[0].tl(), vecRectTarget_[0].br(), Scalar(0, 0, 255), 2, 8, 0);

      // saving data into DB style, for ver.2.0
      vector<BboxDB> vecBboxDB;
      BboxDB tempBbox;
      tempBbox.strLabel = "pet";
      tempBbox.vecBbox.push_back(vecRectTarget_[0]);
      vecBboxDB.push_back(tempBbox);
      cfgParam_.vecPetBboxDB.push_back(vecBboxDB);

      // making the filename  using stringstream, with the numbering rule
      stringstream strStreamFileName;
      strStreamFileName << cfgParam_.strXmlFileNmFwd;

      strStreamFileName << std::setfill('0') << std::setw(cfgParam_.nXmlFileNmDigit) << (ii + 7481);
      strStreamFileName << "." + cfgParam_.strXmlExt;

      // making the full file path
      string strXmlFile;
      strXmlFile = cfgParam_.strXmlFolderPath + strStreamFileName.str();

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
      TiXmlText* txtElem1 = new TiXmlText(strStreamFileName.str());
      pElem1->LinkEndChild(txtElem1);
      pRoot->LinkEndChild(pElem1);

      TiXmlElement* pElem2 = new TiXmlElement("source");
      TiXmlElement* pElem21 = new TiXmlElement("database");
      TiXmlText* txtElem21 = new TiXmlText("ETRI collision avoidance DB");
      pElem21->LinkEndChild(txtElem21);
      TiXmlElement* pElem22 = new TiXmlElement("annotation");
      TiXmlText* txtElem22 = new TiXmlText("PASCAL VOC2017");
      pElem22->LinkEndChild(txtElem22);
      pElem2->LinkEndChild(pElem21);
      pElem2->LinkEndChild(pElem22);
      pRoot->LinkEndChild(pElem2);

      TiXmlElement* pElem3 = new TiXmlElement("owner");
      TiXmlElement* pElem31 = new TiXmlElement("institute");
      TiXmlText* txtElem31 = new TiXmlText("ETRI");
      pElem31->LinkEndChild(txtElem31);
      TiXmlElement* pElem32 = new TiXmlElement("name");
      TiXmlText* txtElem32 = new TiXmlText("Dr. Eunhye Kim");
      pElem32->LinkEndChild(txtElem32);
      pElem3->LinkEndChild(pElem31);
      pElem3->LinkEndChild(pElem32);
      pRoot->LinkEndChild(pElem3);

      TiXmlElement* pElem4 = new TiXmlElement("size");
      TiXmlElement* pElem41 = new TiXmlElement("width");
      TiXmlText* txtElem41 = new TiXmlText(to_string(cfgParam_.nWidthRef));
      pElem41->LinkEndChild(txtElem41);
      TiXmlElement* pElem42 = new TiXmlElement("height");
      TiXmlText* txtElem42 = new TiXmlText(to_string(cfgParam_.nHeightRef));
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

      // making bbox, maskImg-based
      for (auto kk = 0; kk < cfgParam_.vecImgBboxDB[nRandomBaseNum].size(); kk++)
      {
        for (auto kkk = 0; kkk < cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].vecBbox.size(); kkk++)
        {
          // applying the selected label
          if (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel != "vegetation")
          {
            string strSelectedLabelMask;
            if (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "person")
              strSelectedLabelMask = "person";
            else if ((cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "rider") ||
                     (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "motorcycle") ||
                     (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "bicycle"))
              strSelectedLabelMask = "two_wheel_vehicle";
            else if ((cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "car") ||
                     (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "truck") ||
                     (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "bus") ||
                     (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "caravan") ||
                     (cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].strLabel == "trailer"))
              strSelectedLabelMask = "four_wheel_vehicle";
            else
            {
            }

            TiXmlElement* pElem5 = new TiXmlElement("object");
            TiXmlElement* pElem51 = new TiXmlElement("name");
            TiXmlText* txtElem51 = new TiXmlText(strSelectedLabelMask);
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

            Rect rectBbox;
            rectBbox = cfgParam_.vecImgBboxDB[nRandomBaseNum][kk].vecBbox[kkk];
            TiXmlElement* pElem55 = new TiXmlElement("bndbox");
            TiXmlElement* pElem551 = new TiXmlElement("xmin");
            float fNormalizedTlX = (float)(rectBbox.tl().x) / nWidth;
            int nResizedTlX = (int)(fNormalizedTlX * cfgParam_.nWidthRef);
            TiXmlText* txtElem551 = new TiXmlText(to_string(nResizedTlX));
            pElem551->LinkEndChild(txtElem551);
            TiXmlElement* pElem552 = new TiXmlElement("ymin");
            float fNormalizedTlY = (float)(rectBbox.tl().y) / nHeight;
            int nResizedTlY = (int)(fNormalizedTlY * cfgParam_.nHeightRef);
            TiXmlText* txtElem552 = new TiXmlText(to_string(nResizedTlY));
            pElem552->LinkEndChild(txtElem552);
            TiXmlElement* pElem553 = new TiXmlElement("xmax");
            float fNormalizedBrX = (float)(rectBbox.br().x) / nWidth;
            int nResizedBrX = (int)(fNormalizedBrX * cfgParam_.nWidthRef);
            TiXmlText* txtElem553 = new TiXmlText(to_string(nResizedBrX));
            pElem553->LinkEndChild(txtElem553);
            TiXmlElement* pElem554 = new TiXmlElement("ymax");
            float fNormalizedBrY = (float)(rectBbox.br().y) / nHeight;
            int nResizedBrY = (int)(fNormalizedBrY * cfgParam_.nHeightRef);
            TiXmlText* txtElem554 = new TiXmlText(to_string(nResizedBrY));
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
        }
      }

      // making bbox, polygon-based
      for (auto kk = 0; kk < cfgParam_.vecPolygonBboxDB[nRandomBaseNum].size(); kk++)
      {
        for (auto kkk = 0; kkk < cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].vecBbox.size(); kkk++)
        {
          // applying the selected label
          if (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel != "vegetation")
          {
            string strSelectedLabelPolygon;
            if (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "person")
              strSelectedLabelPolygon = "person";
            else if ((cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "rider") ||
                     (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "motorcycle") ||
                     (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "bicycle"))
              strSelectedLabelPolygon = "two_wheel_vehicle";
            else if ((cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "car") ||
                     (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "truck") ||
                     (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "bus") ||
                     (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "caravan") ||
                     (cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].strLabel == "trailer"))
              strSelectedLabelPolygon = "four_wheel_vehicle";
            else
            {
            }

            TiXmlElement* pElem6 = new TiXmlElement("object");
            TiXmlElement* pElem61 = new TiXmlElement("name");
            TiXmlText* txtElem61 = new TiXmlText(strSelectedLabelPolygon);
            pElem61->LinkEndChild(txtElem61);
            TiXmlElement* pElem62 = new TiXmlElement("pose");
            TiXmlText* txtElem62 = new TiXmlText("Left");
            pElem62->LinkEndChild(txtElem62);
            TiXmlElement* pElem63 = new TiXmlElement("truncated");
            TiXmlText* txtElem63 = new TiXmlText("1");
            pElem63->LinkEndChild(txtElem63);
            TiXmlElement* pElem64 = new TiXmlElement("difficult");
            TiXmlText* txtElem64 = new TiXmlText("0");
            pElem64->LinkEndChild(txtElem64);

            Rect rectBbox;
            rectBbox = cfgParam_.vecPolygonBboxDB[nRandomBaseNum][kk].vecBbox[kkk];
            TiXmlElement* pElem65 = new TiXmlElement("bndbox");
            TiXmlElement* pElem651 = new TiXmlElement("xmin");
            float fNormalizedTlX = (float)(rectBbox.tl().x) / nWidth;
            int nResizedTlX = (int)(fNormalizedTlX * cfgParam_.nWidthRef);
            TiXmlText* txtElem651 = new TiXmlText(to_string(nResizedTlX));
            pElem651->LinkEndChild(txtElem651);
            TiXmlElement* pElem652 = new TiXmlElement("ymin");
            float fNormalizedTlY = (float)(rectBbox.tl().y) / nHeight;
            int nResizedTlY = (int)(fNormalizedTlY * cfgParam_.nHeightRef);
            TiXmlText* txtElem652 = new TiXmlText(to_string(nResizedTlY));
            pElem652->LinkEndChild(txtElem652);
            TiXmlElement* pElem653 = new TiXmlElement("xmax");
            float fNormalizedBrX = (float)(rectBbox.br().x) / nWidth;
            int nResizedBrX = (int)(fNormalizedBrX * cfgParam_.nWidthRef);
            TiXmlText* txtElem653 = new TiXmlText(to_string(nResizedBrX));
            pElem653->LinkEndChild(txtElem653);
            TiXmlElement* pElem654 = new TiXmlElement("ymax");
            float fNormalizedBrY = (float)(rectBbox.br().y) / nHeight;
            int nResizedBrY = (int)(fNormalizedBrY * cfgParam_.nHeightRef);
            TiXmlText* txtElem654 = new TiXmlText(to_string(nResizedBrY));
            pElem654->LinkEndChild(txtElem654);

            pElem65->LinkEndChild(pElem651);
            pElem65->LinkEndChild(pElem652);
            pElem65->LinkEndChild(pElem653);
            pElem65->LinkEndChild(pElem654);

            pElem6->LinkEndChild(pElem61);
            pElem6->LinkEndChild(pElem62);
            pElem6->LinkEndChild(pElem63);
            pElem6->LinkEndChild(pElem64);
            pElem6->LinkEndChild(pElem65);
            pRoot->LinkEndChild(pElem6);
          }
        }
      }

      // making bbox, using pet image mixing loop, for ver.2.0
      for (auto kk = 0; kk < cfgParam_.vecPetBboxDB[ii].size(); kk++)
      {
        for (auto kkk = 0; kkk < cfgParam_.vecPetBboxDB[ii][kk].vecBbox.size(); kkk++)
        {
          // applying the selected label
          if (cfgParam_.vecPetBboxDB[ii][kk].strLabel != "vegetation")
          {
            string strSelectedLabelPet;
            strSelectedLabelPet = "pet";

            TiXmlElement* pElem7 = new TiXmlElement("object");
            TiXmlElement* pElem71 = new TiXmlElement("name");
            TiXmlText* txtElem71 = new TiXmlText(strSelectedLabelPet);
            pElem71->LinkEndChild(txtElem71);
            TiXmlElement* pElem72 = new TiXmlElement("pose");
            TiXmlText* txtElem72 = new TiXmlText("Left");
            pElem72->LinkEndChild(txtElem72);
            TiXmlElement* pElem73 = new TiXmlElement("truncated");
            TiXmlText* txtElem73 = new TiXmlText("1");
            pElem73->LinkEndChild(txtElem73);
            TiXmlElement* pElem74 = new TiXmlElement("difficult");
            TiXmlText* txtElem74 = new TiXmlText("0");
            pElem74->LinkEndChild(txtElem74);

            // already resized
            Rect rectBbox;
            rectBbox = cfgParam_.vecPetBboxDB[ii][kk].vecBbox[kkk];
            TiXmlElement* pElem75 = new TiXmlElement("bndbox");
            TiXmlElement* pElem751 = new TiXmlElement("xmin");
            TiXmlText* txtElem751 = new TiXmlText(to_string(rectBbox.tl().x));
            pElem751->LinkEndChild(txtElem751);
            TiXmlElement* pElem752 = new TiXmlElement("ymin");
            TiXmlText* txtElem752 = new TiXmlText(to_string(rectBbox.tl().y));
            pElem752->LinkEndChild(txtElem752);
            TiXmlElement* pElem753 = new TiXmlElement("xmax");
            TiXmlText* txtElem753 = new TiXmlText(to_string(rectBbox.br().x));
            pElem753->LinkEndChild(txtElem753);
            TiXmlElement* pElem754 = new TiXmlElement("ymax");
            TiXmlText* txtElem754 = new TiXmlText(to_string(rectBbox.br().y));
            pElem754->LinkEndChild(txtElem754);

            pElem75->LinkEndChild(pElem751);
            pElem75->LinkEndChild(pElem752);
            pElem75->LinkEndChild(pElem753);
            pElem75->LinkEndChild(pElem754);

            pElem7->LinkEndChild(pElem71);
            pElem7->LinkEndChild(pElem72);
            pElem7->LinkEndChild(pElem73);
            pElem7->LinkEndChild(pElem74);
            pElem7->LinkEndChild(pElem75);
            pRoot->LinkEndChild(pElem7);
          }
        }
      }

      // saving xml file
      docXml.SaveFile(strXmlFile);

      // // pausing and destroying all imshow result
      // waitKey(0);
    }
  }

  // assigning variables for browsing raw images with bbox result and saving bbox position data recursively
  vector<String> vecRawFileNm;
  glob(cfgParam_.strRawFolderPath, vecRawFileNm, true);

  // browsing raw images recursively
  for (size_t k = 0; k < vecRawFileNm.size(); k++)
  {
    // for debugging
    ROS_INFO("Processing_xmlGen(%d,%d)", (int)(k), (int)(vecRawFileNm.size()));

    // making the filename  using stringstream, with the numbering rule
    stringstream strStreamFileName;
    strStreamFileName << cfgParam_.strXmlFileNmFwd;

    strStreamFileName << std::setfill('0') << std::setw(cfgParam_.nXmlFileNmDigit) << k;
    strStreamFileName << "." + cfgParam_.strXmlExt;

    // making the full file path
    string strXmlFile;
    strXmlFile = cfgParam_.strXmlFolderPath + strStreamFileName.str();

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
    TiXmlText* txtElem1 = new TiXmlText(strStreamFileName.str());
    pElem1->LinkEndChild(txtElem1);
    pRoot->LinkEndChild(pElem1);

    TiXmlElement* pElem2 = new TiXmlElement("source");
    TiXmlElement* pElem21 = new TiXmlElement("database");
    TiXmlText* txtElem21 = new TiXmlText("ETRI collision avoidance DB");
    pElem21->LinkEndChild(txtElem21);
    TiXmlElement* pElem22 = new TiXmlElement("annotation");
    TiXmlText* txtElem22 = new TiXmlText("PASCAL VOC2017");
    pElem22->LinkEndChild(txtElem22);
    pElem2->LinkEndChild(pElem21);
    pElem2->LinkEndChild(pElem22);
    pRoot->LinkEndChild(pElem2);

    TiXmlElement* pElem3 = new TiXmlElement("owner");
    TiXmlElement* pElem31 = new TiXmlElement("institute");
    TiXmlText* txtElem31 = new TiXmlText("ETRI");
    pElem31->LinkEndChild(txtElem31);
    TiXmlElement* pElem32 = new TiXmlElement("name");
    TiXmlText* txtElem32 = new TiXmlText("Dr. Eunhye Kim");
    pElem32->LinkEndChild(txtElem32);
    pElem3->LinkEndChild(pElem31);
    pElem3->LinkEndChild(pElem32);
    pRoot->LinkEndChild(pElem3);

    TiXmlElement* pElem4 = new TiXmlElement("size");
    TiXmlElement* pElem41 = new TiXmlElement("width");
    TiXmlText* txtElem41 = new TiXmlText(to_string(cfgParam_.nWidthRef));
    pElem41->LinkEndChild(txtElem41);
    TiXmlElement* pElem42 = new TiXmlElement("height");
    TiXmlText* txtElem42 = new TiXmlText(to_string(cfgParam_.nHeightRef));
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

    // making bbox, maskImg-based
    for (auto kk = 0; kk < cfgParam_.vecImgBboxDB[k].size(); kk++)
    {
      for (auto kkk = 0; kkk < cfgParam_.vecImgBboxDB[k][kk].vecBbox.size(); kkk++)
      {
        // applying the selected label
        if (cfgParam_.vecImgBboxDB[k][kk].strLabel != "vegetation")
        {
          string strSelectedLabelMask;
          if (cfgParam_.vecImgBboxDB[k][kk].strLabel == "person")
            strSelectedLabelMask = "person";
          else if ((cfgParam_.vecImgBboxDB[k][kk].strLabel == "rider") ||
                   (cfgParam_.vecImgBboxDB[k][kk].strLabel == "motorcycle") ||
                   (cfgParam_.vecImgBboxDB[k][kk].strLabel == "bicycle"))
            strSelectedLabelMask = "two_wheel_vehicle";
          else if ((cfgParam_.vecImgBboxDB[k][kk].strLabel == "car") ||
                   (cfgParam_.vecImgBboxDB[k][kk].strLabel == "truck") ||
                   (cfgParam_.vecImgBboxDB[k][kk].strLabel == "bus") ||
                   (cfgParam_.vecImgBboxDB[k][kk].strLabel == "caravan") ||
                   (cfgParam_.vecImgBboxDB[k][kk].strLabel == "trailer"))
            strSelectedLabelMask = "four_wheel_vehicle";
          else
          {
          }

          TiXmlElement* pElem5 = new TiXmlElement("object");
          TiXmlElement* pElem51 = new TiXmlElement("name");
          TiXmlText* txtElem51 = new TiXmlText(strSelectedLabelMask);
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

          Rect rectBbox;
          rectBbox = cfgParam_.vecImgBboxDB[k][kk].vecBbox[kkk];
          TiXmlElement* pElem55 = new TiXmlElement("bndbox");
          TiXmlElement* pElem551 = new TiXmlElement("xmin");
          float fNormalizedTlX = (float)(rectBbox.tl().x) / nWidth;
          int nResizedTlX = (int)(fNormalizedTlX * cfgParam_.nWidthRef);
          TiXmlText* txtElem551 = new TiXmlText(to_string(nResizedTlX));
          pElem551->LinkEndChild(txtElem551);
          TiXmlElement* pElem552 = new TiXmlElement("ymin");
          float fNormalizedTlY = (float)(rectBbox.tl().y) / nHeight;
          int nResizedTlY = (int)(fNormalizedTlY * cfgParam_.nHeightRef);
          TiXmlText* txtElem552 = new TiXmlText(to_string(nResizedTlY));
          pElem552->LinkEndChild(txtElem552);
          TiXmlElement* pElem553 = new TiXmlElement("xmax");
          float fNormalizedBrX = (float)(rectBbox.br().x) / nWidth;
          int nResizedBrX = (int)(fNormalizedBrX * cfgParam_.nWidthRef);
          TiXmlText* txtElem553 = new TiXmlText(to_string(nResizedBrX));
          pElem553->LinkEndChild(txtElem553);
          TiXmlElement* pElem554 = new TiXmlElement("ymax");
          float fNormalizedBrY = (float)(rectBbox.br().y) / nHeight;
          int nResizedBrY = (int)(fNormalizedBrY * cfgParam_.nHeightRef);
          TiXmlText* txtElem554 = new TiXmlText(to_string(nResizedBrY));
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
      }
    }

    // making bbox, polygon-based
    for (auto kk = 0; kk < cfgParam_.vecPolygonBboxDB[k].size(); kk++)
    {
      for (auto kkk = 0; kkk < cfgParam_.vecPolygonBboxDB[k][kk].vecBbox.size(); kkk++)
      {
        // applying the selected label
        if (cfgParam_.vecPolygonBboxDB[k][kk].strLabel != "vegetation")
        {
          string strSelectedLabelPolygon;
          if (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "person")
            strSelectedLabelPolygon = "person";
          else if ((cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "rider") ||
                   (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "motorcycle") ||
                   (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "bicycle"))
            strSelectedLabelPolygon = "two_wheel_vehicle";
          else if ((cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "car") ||
                   (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "truck") ||
                   (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "bus") ||
                   (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "caravan") ||
                   (cfgParam_.vecPolygonBboxDB[k][kk].strLabel == "trailer"))
            strSelectedLabelPolygon = "four_wheel_vehicle";
          else
          {
          }

          TiXmlElement* pElem6 = new TiXmlElement("object");
          TiXmlElement* pElem61 = new TiXmlElement("name");
          TiXmlText* txtElem61 = new TiXmlText(strSelectedLabelPolygon);
          pElem61->LinkEndChild(txtElem61);
          TiXmlElement* pElem62 = new TiXmlElement("pose");
          TiXmlText* txtElem62 = new TiXmlText("Left");
          pElem62->LinkEndChild(txtElem62);
          TiXmlElement* pElem63 = new TiXmlElement("truncated");
          TiXmlText* txtElem63 = new TiXmlText("1");
          pElem63->LinkEndChild(txtElem63);
          TiXmlElement* pElem64 = new TiXmlElement("difficult");
          TiXmlText* txtElem64 = new TiXmlText("0");
          pElem64->LinkEndChild(txtElem64);

          Rect rectBbox;
          rectBbox = cfgParam_.vecPolygonBboxDB[k][kk].vecBbox[kkk];
          TiXmlElement* pElem65 = new TiXmlElement("bndbox");
          TiXmlElement* pElem651 = new TiXmlElement("xmin");
          float fNormalizedTlX = (float)(rectBbox.tl().x) / nWidth;
          int nResizedTlX = (int)(fNormalizedTlX * cfgParam_.nWidthRef);
          TiXmlText* txtElem651 = new TiXmlText(to_string(nResizedTlX));
          pElem651->LinkEndChild(txtElem651);
          TiXmlElement* pElem652 = new TiXmlElement("ymin");
          float fNormalizedTlY = (float)(rectBbox.tl().y) / nHeight;
          int nResizedTlY = (int)(fNormalizedTlY * cfgParam_.nHeightRef);
          TiXmlText* txtElem652 = new TiXmlText(to_string(nResizedTlY));
          pElem652->LinkEndChild(txtElem652);
          TiXmlElement* pElem653 = new TiXmlElement("xmax");
          float fNormalizedBrX = (float)(rectBbox.br().x) / nWidth;
          int nResizedBrX = (int)(fNormalizedBrX * cfgParam_.nWidthRef);
          TiXmlText* txtElem653 = new TiXmlText(to_string(nResizedBrX));
          pElem653->LinkEndChild(txtElem653);
          TiXmlElement* pElem654 = new TiXmlElement("ymax");
          float fNormalizedBrY = (float)(rectBbox.br().y) / nHeight;
          int nResizedBrY = (int)(fNormalizedBrY * cfgParam_.nHeightRef);
          TiXmlText* txtElem654 = new TiXmlText(to_string(nResizedBrY));
          pElem654->LinkEndChild(txtElem654);

          pElem65->LinkEndChild(pElem651);
          pElem65->LinkEndChild(pElem652);
          pElem65->LinkEndChild(pElem653);
          pElem65->LinkEndChild(pElem654);

          pElem6->LinkEndChild(pElem61);
          pElem6->LinkEndChild(pElem62);
          pElem6->LinkEndChild(pElem63);
          pElem6->LinkEndChild(pElem64);
          pElem6->LinkEndChild(pElem65);
          pRoot->LinkEndChild(pElem6);
        }
      }
    }

    // saving xml file
    docXml.SaveFile(strXmlFile);

    // calculating size flag
    bSizeCalcFlag = GenSizeCalcFlag(k, (int)(vecRawFileNm.size()));

    // // pausing and destroying all imshow result
    // waitKey(0);
  }

  // for debugging
  ROS_INFO(" ");

  return;
}

// main loop: img file resizer
void CvtSeg2Bbox::MainLoopImgResizer()
{
  // 1st, resizing raw image and saving resized images
  // assigning variables for browsing raw images recursively
  vector<String> vecImgFileNm;
  glob(cfgParam_.strRawFolderPath, vecImgFileNm, true);

  // browsing raw images recursively
  for (size_t i = 0; i < vecImgFileNm.size(); i++)
  {
    // for debugging
    ROS_INFO("Processing_imgResize(%d,%d)", (int)(i), (int)(vecImgFileNm.size()));

    // assigning the raw image
    Mat imgRaw = imread(vecImgFileNm[i]);

    // image width and height info. (h1024, w2048)
    nHeight = imgRaw.rows;
    nWidth = imgRaw.cols;

    // resizing w.r.t the cityscapesDB
    Mat imgResize;
    resize(imgRaw, imgResize, Size(cfgParam_.nWidthRef, cfgParam_.nHeightRef), 0, 0, INTER_NEAREST);

    // making the filename  using stringstream, with the numbering rule
    stringstream strStreamImgFileName;
    strStreamImgFileName << cfgParam_.strImgFileNmFwd;
    strStreamImgFileName << std::setfill('0') << std::setw(cfgParam_.nImgFileNmDigit) << i;
    strStreamImgFileName << "." + cfgParam_.strImgExt;

    // making the full file path
    string strCvtImgFile;
    strCvtImgFile = cfgParam_.strCvtImgFolderPath + strStreamImgFileName.str();

    // saving the resized image
    imwrite(strCvtImgFile, imgResize);

    // calculating size flag
    bSizeCalcFlag = GenSizeCalcFlag(i, (int)(vecImgFileNm.size()));
  }

  return;
}

// get size calculation flag
bool CvtSeg2Bbox::GetSizeCalcFlag()
{
  return bSizeCalcFlag;
}

// generating size calculation flag for terminating converter
bool CvtSeg2Bbox::GenSizeCalcFlag(int nSize, int nTotal)
{
  bool bRes = false;
  if (fabs(nSize - nTotal) == 1.0f)
    bRes = true;
  return bRes;
}

// generating bbox data
vector<Rect> CvtSeg2Bbox::GenBboxData(Mat imgIn, Scalar color, int nThresh)
{
  // finding contours
  vector<vector<Point>> vecContours;
  vector<Vec4i> vecHierarchy;
  findContours(imgIn, vecContours, vecHierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // finding bounding boxes
  vector<vector<Point>> vecContoursPoly(vecContours.size());
  vector<vector<Point>> vecHull(vecContours.size());
  vector<Rect> vecRes(vecContours.size());
  Mat imgDrawing = Mat::zeros(imgIn.size(), CV_8UC3);

  if (vecContours.size() > 0)
  {
    // finding bounding boxes using approximated polygons
    for (size_t i = 0; i < vecContours.size(); i++)
    {
      approxPolyDP(vecContours[i], vecContoursPoly[i], nThresh, false);
      convexHull(vecContoursPoly[i], vecHull[i]);
      vecRes[i] = boundingRect(vecHull[i]);
    }

    // drawing bbox data for debugging
    // for (size_t i = 0; i < vecContours.size(); i++)
    // {
    //   drawContours(imgDrawing, vecHull, (int)(i), color);
    //   rectangle(imgDrawing, vecRes[i].tl(), vecRes[i].br(), color, 2);
    // }

    // for debugging
    // imshow("Contours", imgDrawing);
  }

  return vecRes;
}

// detecting canny edge data
Mat CvtSeg2Bbox::CannyEdge(Mat imgIn, int nThresh)
{
  Mat imgRes;
  Canny(imgIn, imgRes, nThresh, (nThresh * 2));
  return imgRes;
}

// generating filtered image using erode
Mat CvtSeg2Bbox::GenFilteredImg(Mat imgIn, int nHeight, int nWidth, int nAnno, int nTrial)
{
  // filtering image using the label info.
  Mat imgRes(nHeight, nWidth, CV_8UC1);
  uchar* reqData = imgIn.data;
  for (int y = 0; y < nHeight; y++)
  {
    uchar* resData = imgRes.data;
    for (int x = 0; x < nWidth; x++)
    {
      uchar b = reqData[y * nWidth * 3 + x * 3];
      uchar g = reqData[y * nWidth * 3 + x * 3 + 1];
      uchar r = reqData[y * nWidth * 3 + x * 3 + 2];

      if ((b == cfgParam_.vecAnnoDB[nAnno].nRGB[2]) && (g == cfgParam_.vecAnnoDB[nAnno].nRGB[1]) &&
          (r == cfgParam_.vecAnnoDB[nAnno].nRGB[0]))
      {
        resData[nWidth * y + x] = 255;
      }
      else
      {
        resData[nWidth * y + x] = 0;
      }
    }
  }

  // making smooth image using morphological filtering
  Mat mask = getStructuringElement(MORPH_RECT, Size(5, 5), Point(1, 1));
  morphologyEx(imgRes, imgRes, cv::MorphTypes::MORPH_OPEN, mask, Point(-1, -1), nTrial);

  // for debugging
  // imshow("filtered", imgRes);

  return imgRes;
}

// calculating the bounding rectangle w.r.t the resized target image
vector<Rect> CvtSeg2Bbox::GetTargetRect(Mat imgInput)
{
  Mat imgContourResize;
  resize(imgInput, imgContourResize, Size(640, 480));
  cvtColor(imgContourResize, imgContourResize, COLOR_BGR2GRAY);
  threshold(imgContourResize, imgContourResize, 250, 255, THRESH_BINARY);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(imgContourResize, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

  vector<vector<Point>> contours_poly(contours.size());
  vector<Rect> result(contours.size());
  for (size_t i = 0; i < contours.size(); i++)
  {
    approxPolyDP(contours[i], contours_poly[i], 3, true);
    result[i] = boundingRect(contours_poly[i]);
  }

  return result;
}

// calculating the image w.r.t the string command
Mat CvtSeg2Bbox::GetImgMix(Mat imgInput, vector<SelectRGB> vecInput, Point ptInput, string strCmd)
{
  Mat result;
  imgInput.copyTo(result);
  vector<Point> vecSelectPtMask;

  if (strCmd == "contour")
    result = Mat::zeros(imgInput.size(), CV_8UC3);
  else
    imgInput.copyTo(result);

  for (unsigned int k = 0; k < vecInput.size(); k++)
  {
    Point ptMixRef;
    ptMixRef.x = ptInput.y + vecInput[k].ptPixel.y;
    ptMixRef.y = ptInput.x + vecInput[k].ptPixel.x;

    if (strCmd == "black")
    {
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[0] = 0;
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[1] = 0;
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[2] = 0;
    }
    else if (strCmd == "rgb")
    {
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[0] = vecInput[k].blue;
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[1] = vecInput[k].green;
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[2] = vecInput[k].red;
    }
    else if (strCmd == "contour")
    {
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[0] = 255;
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[1] = 255;
      result.at<Vec3b>(ptMixRef.x, ptMixRef.y)[2] = 255;
    }
    else
    {
    }
  }

  return result;
}

// calculating the mask information
vector<SelectRGB> CvtSeg2Bbox::GetMaskInfo(Mat imgTargetResized, ImgSize imgTargetResizedSize)
{
  vector<SelectRGB> result;
  for (unsigned int j = 0; j < imgTargetResizedSize.nHeight; j++)
  {
    uchar* ptInTargetResized = imgTargetResized.ptr<uchar>(j);
    for (unsigned int i = 0; i < imgTargetResizedSize.nWidth; i++)
    {
      uchar bTar = ptInTargetResized[i * 3 + 0];
      uchar gTar = ptInTargetResized[i * 3 + 1];
      uchar rTar = ptInTargetResized[i * 3 + 2];

      if (!((bTar > 245) && (gTar > 245) && (rTar > 245)))
      {
        SelectRGB temp;
        temp.ptPixel.x = i;
        temp.ptPixel.y = j;
        temp.blue = bTar;
        temp.green = gTar;
        temp.red = rTar;
        result.push_back(temp);
      }
    }
  }
  return result;
}

// calculating the random top-left point w.r.t the resized target
Point CvtSeg2Bbox::GetRngPtTlForTargetResized(ImgSize imgTargetResizedSize, ImgSize imgBaseSize, float fRatio)
{
  Point result;
  int nRngRangeHeight = (imgBaseSize.nHeight - imgTargetResizedSize.nHeight * 1.5f);
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> rngPtTlX(
      0, ((imgBaseSize.nWidth - (imgBaseSize.nWidth * fRatio)) - imgTargetResizedSize.nWidth));
  uniform_int_distribution<> rngPtTlY(
      (int)(imgBaseSize.nHeight * 0.35f),
      ((imgBaseSize.nHeight - (imgBaseSize.nHeight * fRatio)) - imgTargetResizedSize.nHeight));
  result.x = rngPtTlX(gen);
  result.y = rngPtTlY(gen);

  if (result.x > (imgBaseSize.nWidth - imgTargetResizedSize.nWidth))
    result.x = (imgBaseSize.nWidth - imgTargetResizedSize.nWidth);
  if (result.y > (imgBaseSize.nHeight - imgTargetResizedSize.nHeight))
    result.y = (imgBaseSize.nHeight - imgTargetResizedSize.nHeight);
  return result;
}

// generating the random number w.r.t the range of number of image
int CvtSeg2Bbox::GenRandNum(int nSize)
{
  int result;
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> rngRange(0, nSize - 1);
  result = rngRange(gen);
  return result;
}

// calculating the resized target image
Mat CvtSeg2Bbox::GetImgTargetResized(Mat imgTarget, ImgSize imgTargetSize, ImgSize imgBaseSize, float fWidthRatio,
                                     float fHeightRatio)
{
  Mat result;
  if ((imgTargetSize.nWidth > ((int)(imgBaseSize.nWidth * fWidthRatio))) ||
      (imgTargetSize.nHeight > ((int)(imgBaseSize.nHeight * fHeightRatio))))
  {
    float fRatio = 0.5f;
    Size szResizeTarget;
    szResizeTarget.width = (int)(imgTargetSize.nWidth * fRatio);
    szResizeTarget.height = (int)(imgTargetSize.nHeight * fRatio);
    resize(imgTarget, result, szResizeTarget);
  }
  else
    imgTarget.copyTo(result);
  return result;
}

// getting the size information of the image
ImgSize CvtSeg2Bbox::GetImgSize(Mat imgInput)
{
  ImgSize result;
  result.nWidth = imgInput.size().width;
  result.nHeight = imgInput.size().height;
  return result;
}

// getting the image from file
Mat CvtSeg2Bbox::GetImgFromFile(string strBaseImgName)
{
  Mat result;
  result = imread(strBaseImgName, IMREAD_COLOR);
  return result;
}

bool CvtSeg2Bbox::sortArea(cv::Rect rect1, cv::Rect rect2)
{
  int nArea1 = rect1.width * rect1.height;
  int nArea2 = rect2.width * rect2.height;
  return (nArea1 > nArea2);
}