#ifndef ROSCV_TEST_CONFIG_PARAM_H
#define ROSCV_TEST_CONFIG_PARAM_H

#include "global_header.h"

using namespace std;
using namespace ros;

class RosParamNotFoundException : public std::exception
{
public:
  string key;
  explicit RosParamNotFoundException(const string& key_)
  {
    key = key_;
  }
  virtual const char* what() const throw()
  {
    string msg = "Failed to read param at key ";
    return (msg + key).c_str();
  }
};

class ConfigParam
{
public:
  ConfigParam();
  ~ConfigParam();

  bool GetRosParams();

  void ReadRosParam(ros::NodeHandle& nh, const string& key, float& val);
  void ReadRosParam(ros::NodeHandle& nh, const string& key, double& val);
  void ReadRosParam(ros::NodeHandle& nh, const string& key, bool& val);
  void ReadRosParam(ros::NodeHandle& nh, const string& key, int32_t& val);
  void ReadRosParam(ros::NodeHandle& nh, const string& key, string& val);

  string strHomeName;
  string strPicType;
  string strPolygonType;
  string strXmlType;
  string strImgExt;
  string strXmlExt;
  string strCvtImgFolderNm;
  string strCvtImgFolderPath;
  string strRawFolderNm;
  string strRawFolderPath;
  string strAnnoFolderNm;
  string strAnnoFolderPath;
  string strPolygonFolderNm;
  string strPolygonFolderPath;
  string strXmlFolderNm;
  string strXmlFolderPath;
  string strImgFileNmFwd;
  string strXmlFileNmFwd;
  string strPetImgFolderNm;
  string strPetImgFolderPath;
  string strSegLabelImgFoldeNm;
  string strSegLabelImgFolderPath;
  string strCvtPetMixImgFolderNm;
  string strCvtPetMixImgFolderPath;

  string strKttPicType;
  string strKttTxtType;
  string strKttXmlType;
  string strKttImgExt;
  string strKttXmlExt;
  string strKttImgFolderNm;
  string strKttImgFolderPath;
  string strKttLabelFolderNm;
  string strKttLabelFolderPath;
  string strKttCvtImgFolderNm;
  string strKttCvtImgFolderPath;
  string strKttXmlFolderNm;
  string strKttXmlFolderPath;
  string strKttImgFileNmFwd;
  string strKttXmlFileNmFwd;

  bool bPetMix;
  int nTrialPetMix;
  float fWidthRatio;
  float fHeightRatio;
  float fInnerRatio;

  int nKttFeatureCase;
  int nKttImgFileNmDigit;
  int nKttXmlFileNmDigit;

  int nFeatureCase;
  int nImgFileNmDigit;
  int nXmlFileNmDigit;

  int nCannyThresh;
  int nMorphThresh;
  int nPolyDPThesh;

  int nWidthRef;
  int nHeightRef;
  int nKttWidthRef;
  int nKttHeightRef;

  vector<string> vecLabels;
  vector<AnnoDB> vecAnnoDB;

  vector<string> vecKttLabels;
  vector<AnnoDB> vecAnnoKttDB;

  vector<AnnoDB> vecAnnoKariDB;

  vector<vector<BboxDB>> vecImgBboxDB;
  vector<vector<BboxDB>> vecPolygonBboxDB;
  vector<vector<BboxDB>> vecPetBboxDB;

  AnnoDB vegetation;
  AnnoDB person;
  AnnoDB rider;
  AnnoDB car;
  AnnoDB truck;
  AnnoDB bus;
  AnnoDB caravan;
  AnnoDB trailer;
  AnnoDB motorcycle;
  AnnoDB bicycle;

  AnnoDB kttCar;
  AnnoDB kttVan;
  AnnoDB kttTruck;
  AnnoDB kttPedestrian;
  AnnoDB kttPersonSit;
  AnnoDB kttCyclist;

  AnnoDB building;
  AnnoDB sky;
  AnnoDB ground;
  AnnoDB river;

  int nOrderBuilding;
  int nOrderSky;
  int nOrderGround;
  int nOrderRiver;

private:
  bool ReadRosParams();
};

#endif  // ROSCV_TEST_CONFIG_PARAM_H