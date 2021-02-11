#include "config_param.h"

using namespace std;
using namespace ros;

ConfigParam::ConfigParam()
{
  // 0: xml file generator
  // 1: xml file checker
  nFeatureCase = 0;
}

ConfigParam::~ConfigParam()
{
}

// reading rosparams (public function)
bool ConfigParam::GetRosParams()
{
  return ReadRosParams();
}

// reading rosparams (private function)
bool ConfigParam::ReadRosParams()
{
  try
  {
    NodeHandle nh("");

    // general information
    strHomeName = getenv("HOME");

    // feature case
    ReadRosParam(nh, "/KdarpaDBConverter/feature", nKdarpaFeatureCase);
    ReadRosParam(nh, "/KdarpaDBConverter/imgfile_type", strKdarpaPicType);
    ReadRosParam(nh, "/KdarpaDBConverter/txtfile_type", strKdarpaTxtType);
    ReadRosParam(nh, "/KdarpaDBConverter/xmlfile_type", strKdarpaXmlType);
    ReadRosParam(nh, "/KdarpaDBConverter/imgfile_extension", strKdarpaImgExt);
    ReadRosParam(nh, "/KdarpaDBConverter/xmlfile_extension", strKdarpaXmlExt);
    ReadRosParam(nh, "/KdarpaDBConverter/file_name_fwd", strKdarpaImgFileNmFwd);
    ReadRosParam(nh, "/KdarpaDBConverter/file_name_num_digit", nKdarpaImgFileNmDigit);
    ReadRosParam(nh, "/KdarpaDBConverter/cvtimg_width", nKdarpaWidthRef);
    ReadRosParam(nh, "/KdarpaDBConverter/cvtimg_height", nKdarpaHeightRef);
    nKdarpaXmlFileNmDigit = nKdarpaImgFileNmDigit;
    strKdarpaXmlFileNmFwd = strKdarpaImgFileNmFwd;

    // folder name and picture file type
    ReadRosParam(nh, "/KdarpaDBfolder/type", strKdarpaType);
    ReadRosParam(nh, "/KdarpaDBfolder/targetSrc", strKdarpaTarSrcNm);
    ReadRosParam(nh, "/KdarpaDBfolder/imgSrc", strKdarpaImgSrcNm);
    ReadRosParam(nh, "/KdarpaDBfolder/xmlSrc", strKdarpaXmlSrcNm);
    ReadRosParam(nh, "/KdarpaDBfolder/txtSrc", strKdarpaTxtSrcNm);
    ReadRosParam(nh, "/KdarpaDBfolder/targetDst", strKdarpaTarDstNm);
    ReadRosParam(nh, "/KdarpaDBfolder/imgDst", strKdarpaImgDstNm);
    ReadRosParam(nh, "/KdarpaDBfolder/xmlDst", strKdarpaXmlDstNm);

    // folder path, raw and annotated images
    strKdarpaTarSrcPath = strHomeName + strKdarpaTarSrcNm + strKdarpaType;
    strKdarpaImgSrcPath = strKdarpaTarSrcPath + strKdarpaImgSrcNm;
    strKdarpaXmlSrcPath = strKdarpaTarSrcPath + strKdarpaXmlSrcNm;
    strKdarpaTxtSrcPath = strKdarpaTarSrcPath + strKdarpaTxtSrcNm;
    strKdarpaTarDstPath = strHomeName + strKdarpaTarDstNm + strKdarpaType;
    strKdarpaImgDstPath = strKdarpaTarDstPath + strKdarpaImgDstNm;
    strKdarpaXmlDstPath = strKdarpaTarDstPath + strKdarpaXmlDstNm;

    // label DB, for Kdarpa
    vecKdarpaLabels.clear();
    vecKdarpaAnnoDB.clear();
    ReadRosParam(nh, "/LabelKdarpaDB/dummy", kdarpaDummy.strLabel);
    kdarpaDummy.nRGB[0] = colorStat_.scalBlack.val[0];
    kdarpaDummy.nRGB[1] = colorStat_.scalBlack.val[1];
    kdarpaDummy.nRGB[2] = colorStat_.scalBlack.val[2];
    vecKdarpaLabels.push_back(kdarpaDummy.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaDummy);

    ReadRosParam(nh, "/LabelKdarpaDB/dog", kdarpaDog.strLabel);
    kdarpaDog.nRGB[0] = colorStat_.scalBlue.val[0];
    kdarpaDog.nRGB[1] = colorStat_.scalBlue.val[1];
    kdarpaDog.nRGB[2] = colorStat_.scalBlue.val[2];
    vecKdarpaLabels.push_back(kdarpaDog.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaDog);

    vecKdarpaLabels.push_back(kdarpaDummy.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaDummy);

    ReadRosParam(nh, "/LabelKdarpaDB/soldier", kdarpaSoldier.strLabel);
    kdarpaSoldier.nRGB[0] = colorStat_.scalLime.val[0];
    kdarpaSoldier.nRGB[1] = colorStat_.scalLime.val[1];
    kdarpaSoldier.nRGB[2] = colorStat_.scalLime.val[2];
    vecKdarpaLabels.push_back(kdarpaSoldier.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaSoldier);

    ReadRosParam(nh, "/LabelKdarpaDB/radiationMark", kdarpaRadiationMark.strLabel);
    kdarpaRadiationMark.nRGB[0] = colorStat_.scalRed.val[0];
    kdarpaRadiationMark.nRGB[1] = colorStat_.scalRed.val[1];
    kdarpaRadiationMark.nRGB[2] = colorStat_.scalRed.val[2];
    vecKdarpaLabels.push_back(kdarpaRadiationMark.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaRadiationMark);

    ReadRosParam(nh, "/LabelKdarpaDB/biochemMark", kdarpaBiochemMark.strLabel);
    kdarpaBiochemMark.nRGB[0] = colorStat_.scalYellow.val[0];
    kdarpaBiochemMark.nRGB[1] = colorStat_.scalYellow.val[1];
    kdarpaBiochemMark.nRGB[2] = colorStat_.scalYellow.val[2];
    vecKdarpaLabels.push_back(kdarpaBiochemMark.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaBiochemMark);

    ReadRosParam(nh, "/LabelKdarpaDB/exitMark", kdarpaExitMark.strLabel);
    kdarpaExitMark.nRGB[0] = colorStat_.scalGreen.val[0];
    kdarpaExitMark.nRGB[1] = colorStat_.scalGreen.val[1];
    kdarpaExitMark.nRGB[2] = colorStat_.scalGreen.val[2];
    vecKdarpaLabels.push_back(kdarpaExitMark.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaExitMark);

    ReadRosParam(nh, "/LabelKdarpaDB/endMark", kdarpaEndMark.strLabel);
    kdarpaEndMark.nRGB[0] = colorStat_.scalNavy.val[0];
    kdarpaEndMark.nRGB[1] = colorStat_.scalNavy.val[1];
    kdarpaEndMark.nRGB[2] = colorStat_.scalNavy.val[2];
    vecKdarpaLabels.push_back(kdarpaEndMark.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaEndMark);

    ReadRosParam(nh, "/LabelKdarpaDB/startMark", kdarpaStartMark.strLabel);
    kdarpaStartMark.nRGB[0] = colorStat_.scalMaroon.val[0];
    kdarpaStartMark.nRGB[1] = colorStat_.scalMaroon.val[1];
    kdarpaStartMark.nRGB[2] = colorStat_.scalMaroon.val[2];
    vecKdarpaLabels.push_back(kdarpaStartMark.strLabel);
    vecKdarpaAnnoDB.push_back(kdarpaStartMark);

    // for debugging
    ROS_INFO("kdarpa:labelSize:%d", (int)(vecKdarpaLabels.size()));
    for (auto i = 0; i < vecKdarpaAnnoDB.size(); i++)
    {
      ROS_INFO("[%d]%s:RGB(%d,%d,%d)", i, vecKdarpaAnnoDB[i].strLabel.c_str(), vecKdarpaAnnoDB[i].nRGB[0],
               vecKdarpaAnnoDB[i].nRGB[1], vecKdarpaAnnoDB[i].nRGB[2]);
    }
    ROS_INFO(" ");

    // feature case
    ReadRosParam(nh, "/CityScapesDBConverter/feature", nFeatureCase);
    ReadRosParam(nh, "/CityScapesDBConverter/use_pet_mixing", bPetMix);
    ReadRosParam(nh, "/CityScapesDBConverter/trial_pet_mixing", nTrialPetMix);
    ReadRosParam(nh, "/CityScapesDBConverter/mix_width_ratio", fWidthRatio);
    ReadRosParam(nh, "/CityScapesDBConverter/mix_height_ratio", fHeightRatio);
    ReadRosParam(nh, "/CityScapesDBConverter/mix_inner_ratio", fInnerRatio);

    // folder name and picture file type
    ReadRosParam(nh, "/CityScapesDBfolder/raw", strRawFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/cvtimg", strCvtImgFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/color_label", strAnnoFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/polygon_data", strPolygonFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/xml_label", strXmlFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/pet_mix", strPetImgFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/seg_color_img", strSegColorImgFoldeNm);
    ReadRosParam(nh, "/CityScapesDBfolder/seg_label_img", strSegLabelImgFoldeNm);
    ReadRosParam(nh, "/CityScapesDBfolder/pet_mix_img", strCvtPetMixImgFolderNm);
    ReadRosParam(nh, "/CityScapesDBfolder/imgfile_type", strPicType);
    ReadRosParam(nh, "/CityScapesDBfolder/polygonfile_type", strPolygonType);
    ReadRosParam(nh, "/CityScapesDBfolder/imgfile_extension", strImgExt);
    ReadRosParam(nh, "/CityScapesDBfolder/xmlfile_extension", strXmlExt);
    ReadRosParam(nh, "/CityScapesDBfolder/xmlfile_type", strXmlType);
    ReadRosParam(nh, "/CityScapesDBfolder/file_name_fwd", strXmlFileNmFwd);
    ReadRosParam(nh, "/CityScapesDBfolder/file_name_num_digit", nXmlFileNmDigit);
    ReadRosParam(nh, "/CityScapesDBfolder/cvtimg_width", nWidthRef);
    ReadRosParam(nh, "/CityScapesDBfolder/cvtimg_height", nHeightRef);

    // folder path, raw and annotated images
    strRawFolderPath = strHomeName + strRawFolderNm + strPicType;
    strCvtImgFolderPath = strHomeName + strCvtImgFolderNm;
    strAnnoFolderPath = strHomeName + strAnnoFolderNm + strPicType;
    strPolygonFolderPath = strHomeName + strPolygonFolderNm + strPolygonType;
    strXmlFolderPath = strHomeName + strXmlFolderNm;
    strPetImgFolderPath = strHomeName + strPetImgFolderNm;
    strSegLabelImgFolderPath = strHomeName + strSegLabelImgFoldeNm;
    strSegColorImgFolderPath = strHomeName + strSegColorImgFoldeNm;
    strCvtPetMixImgFolderPath = strHomeName + strCvtPetMixImgFolderNm;

    ReadRosParam(nh, "/CityScapesDBfolder/yolo_label", strYoloLabelFolderNm);
    strYoloLabelFolderPath = strHomeName + strYoloLabelFolderNm;

    strImgFileNmFwd = strXmlFileNmFwd;
    nImgFileNmDigit = nXmlFileNmDigit;

    // feature case
    ReadRosParam(nh, "/KittyDBConverter/feature", nKttFeatureCase);

    // folder name and picture file type
    ReadRosParam(nh, "/KittyDBfolder/image", strKttImgFolderNm);
    ReadRosParam(nh, "/KittyDBfolder/label", strKttLabelFolderNm);
    ReadRosParam(nh, "/KittyDBfolder/cvtimg", strKttCvtImgFolderNm);
    ReadRosParam(nh, "/KittyDBfolder/xml_label", strKttXmlFolderNm);
    ReadRosParam(nh, "/KittyDBfolder/imgfile_type", strKttPicType);
    ReadRosParam(nh, "/KittyDBfolder/txtfile_type", strKttTxtType);
    ReadRosParam(nh, "/KittyDBfolder/xmlfile_type", strKttXmlType);
    ReadRosParam(nh, "/KittyDBfolder/imgfile_extension", strKttImgExt);
    ReadRosParam(nh, "/KittyDBfolder/xmlfile_extension", strKttXmlExt);
    ReadRosParam(nh, "/KittyDBfolder/file_name_fwd", strKttXmlFileNmFwd);
    ReadRosParam(nh, "/KittyDBfolder/file_name_num_digit", nKttXmlFileNmDigit);
    ReadRosParam(nh, "/KittyDBfolder/cvtimg_width", nKttWidthRef);
    ReadRosParam(nh, "/KittyDBfolder/cvtimg_height", nKttHeightRef);

    // folder path, for kitty data
    strKttImgFolderPath = strHomeName + strKttImgFolderNm + strKttPicType;
    strKttLabelFolderPath = strHomeName + strKttLabelFolderNm + strKttTxtType;
    strKttCvtImgFolderPath = strHomeName + strKttCvtImgFolderNm;
    strKttXmlFolderPath = strHomeName + strKttXmlFolderNm;

    strKttImgFileNmFwd = strKttXmlFileNmFwd;
    nKttImgFileNmDigit = nKttXmlFileNmDigit;

    // label DB, for kitty
    vecKttLabels.clear();
    vecAnnoKttDB.clear();
    ReadRosParam(nh, "/LabelsKittyDB/car", kttCar.strLabel);
    vecKttLabels.push_back(kttCar.strLabel);
    vecAnnoKttDB.push_back(kttCar);

    ReadRosParam(nh, "/LabelsKittyDB/van", kttVan.strLabel);
    vecKttLabels.push_back(kttVan.strLabel);
    vecAnnoKttDB.push_back(kttVan);

    ReadRosParam(nh, "/LabelsKittyDB/truck", kttTruck.strLabel);
    vecKttLabels.push_back(kttTruck.strLabel);
    vecAnnoKttDB.push_back(kttTruck);

    ReadRosParam(nh, "/LabelsKittyDB/pedestrian", kttCar.strLabel);
    vecKttLabels.push_back(kttCar.strLabel);
    vecAnnoKttDB.push_back(kttCar);

    ReadRosParam(nh, "/LabelsKittyDB/person_sit", kttCar.strLabel);
    vecKttLabels.push_back(kttCar.strLabel);
    vecAnnoKttDB.push_back(kttCar);

    ReadRosParam(nh, "/LabelsKittyDB/cyclist", kttCar.strLabel);
    vecKttLabels.push_back(kttCar.strLabel);
    vecAnnoKttDB.push_back(kttCar);

    // for filtering imgs
    ReadRosParam(nh, "/LabelsCityScapesDB/cannyThresh", nCannyThresh);
    ReadRosParam(nh, "/LabelsCityScapesDB/morphThresh", nMorphThresh);
    ReadRosParam(nh, "/LabelsCityScapesDB/polyDPThesh", nPolyDPThesh);

    // label DB, if need to add, please follow the rule
    vecLabels.clear();
    vecAnnoDB.clear();
    ReadRosParam(nh, "/LabelsCityScapesDB/vegetation/name", vegetation.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/vegetation/R", vegetation.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/vegetation/G", vegetation.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/vegetation/B", vegetation.nRGB[2]);
    vecLabels.push_back(vegetation.strLabel);
    vecAnnoDB.push_back(vegetation);

    ReadRosParam(nh, "/LabelsCityScapesDB/person/name", person.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/person/R", person.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/person/G", person.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/person/B", person.nRGB[2]);
    vecLabels.push_back(person.strLabel);
    vecAnnoDB.push_back(person);

    ReadRosParam(nh, "/LabelsCityScapesDB/rider/name", rider.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/rider/R", rider.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/rider/G", rider.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/rider/B", rider.nRGB[2]);
    vecLabels.push_back(rider.strLabel);
    vecAnnoDB.push_back(rider);

    ReadRosParam(nh, "/LabelsCityScapesDB/car/name", car.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/car/R", car.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/car/G", car.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/car/B", car.nRGB[2]);
    vecLabels.push_back(car.strLabel);
    vecAnnoDB.push_back(car);

    ReadRosParam(nh, "/LabelsCityScapesDB/truck/name", truck.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/truck/R", truck.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/truck/G", truck.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/truck/B", truck.nRGB[2]);
    vecLabels.push_back(truck.strLabel);
    vecAnnoDB.push_back(truck);

    ReadRosParam(nh, "/LabelsCityScapesDB/bus/name", bus.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/bus/R", bus.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/bus/G", bus.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/bus/B", bus.nRGB[2]);
    vecLabels.push_back(bus.strLabel);
    vecAnnoDB.push_back(bus);

    ReadRosParam(nh, "/LabelsCityScapesDB/caravan/name", caravan.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/caravan/R", caravan.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/caravan/G", caravan.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/caravan/B", caravan.nRGB[2]);
    vecLabels.push_back(caravan.strLabel);
    vecAnnoDB.push_back(caravan);

    ReadRosParam(nh, "/LabelsCityScapesDB/trailer/name", trailer.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/trailer/R", trailer.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/trailer/G", trailer.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/trailer/B", trailer.nRGB[2]);
    vecLabels.push_back(trailer.strLabel);
    vecAnnoDB.push_back(trailer);

    ReadRosParam(nh, "/LabelsCityScapesDB/motorcycle/name", motorcycle.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/motorcycle/R", motorcycle.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/motorcycle/G", motorcycle.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/motorcycle/B", motorcycle.nRGB[2]);
    vecLabels.push_back(motorcycle.strLabel);
    vecAnnoDB.push_back(motorcycle);

    ReadRosParam(nh, "/LabelsCityScapesDB/bicycle/name", bicycle.strLabel);
    ReadRosParam(nh, "/LabelsCityScapesDB/bicycle/R", bicycle.nRGB[0]);
    ReadRosParam(nh, "/LabelsCityScapesDB/bicycle/G", bicycle.nRGB[1]);
    ReadRosParam(nh, "/LabelsCityScapesDB/bicycle/B", bicycle.nRGB[2]);
    vecLabels.push_back(bicycle.strLabel);
    vecAnnoDB.push_back(bicycle);

    // for debugging
    ROS_INFO("cityscape:labelSize:%d", (int)(vecLabels.size()));
    for (auto i = 0; i < vecAnnoDB.size(); i++)
    {
      ROS_INFO("[%d]%s:RGB(%d,%d,%d)", i, vecAnnoDB[i].strLabel.c_str(), vecAnnoDB[i].nRGB[0], vecAnnoDB[i].nRGB[1],
               vecAnnoDB[i].nRGB[2]);
    }
    ROS_INFO(" ");

    // for debugging
    ROS_INFO("kitty:labelSize:%d", (int)(vecKttLabels.size()));
    for (auto i = 0; i < vecAnnoKttDB.size(); i++)
    {
      ROS_INFO("[%d]%s", i, vecAnnoDB[i].strLabel.c_str());
    }
    ROS_INFO(" ");

    ReadRosParam(nh, "/LabelKariDB/building/name", building.strLabel);
    ReadRosParam(nh, "/LabelKariDB/building/order", nOrderBuilding);
    ReadRosParam(nh, "/LabelKariDB/building/R", building.nRGB[0]);
    ReadRosParam(nh, "/LabelKariDB/building/G", building.nRGB[1]);
    ReadRosParam(nh, "/LabelKariDB/building/B", building.nRGB[2]);
    vecAnnoKariDB.push_back(building);

    ReadRosParam(nh, "/LabelKariDB/sky/name", sky.strLabel);
    ReadRosParam(nh, "/LabelKariDB/sky/order", nOrderSky);
    ReadRosParam(nh, "/LabelKariDB/sky/R", sky.nRGB[0]);
    ReadRosParam(nh, "/LabelKariDB/sky/G", sky.nRGB[1]);
    ReadRosParam(nh, "/LabelKariDB/sky/B", sky.nRGB[2]);
    vecAnnoKariDB.push_back(sky);

    ReadRosParam(nh, "/LabelKariDB/ground/name", ground.strLabel);
    ReadRosParam(nh, "/LabelKariDB/ground/order", nOrderGround);
    ReadRosParam(nh, "/LabelKariDB/ground/R", ground.nRGB[0]);
    ReadRosParam(nh, "/LabelKariDB/ground/G", ground.nRGB[1]);
    ReadRosParam(nh, "/LabelKariDB/ground/B", ground.nRGB[2]);
    vecAnnoKariDB.push_back(ground);

    ReadRosParam(nh, "/LabelKariDB/river/name", river.strLabel);
    ReadRosParam(nh, "/LabelKariDB/river/order", nOrderRiver);
    ReadRosParam(nh, "/LabelKariDB/river/R", river.nRGB[0]);
    ReadRosParam(nh, "/LabelKariDB/river/G", river.nRGB[1]);
    ReadRosParam(nh, "/LabelKariDB/river/B", river.nRGB[2]);
    vecAnnoKariDB.push_back(river);

    // for debugging
    ROS_INFO("kari:labelSize:%d", (int)(vecAnnoKariDB.size()));
    for (auto i = 0; i < vecAnnoKariDB.size(); i++)
    {
      ROS_INFO("[%d]%s:RGB(%d,%d,%d)", i, vecAnnoKariDB[i].strLabel.c_str(), vecAnnoKariDB[i].nRGB[0],
               vecAnnoKariDB[i].nRGB[1], vecAnnoKariDB[i].nRGB[2]);
    }
    ROS_INFO(" ");
  }
  catch (RosParamNotFoundException& ex)
  {
    ROS_ERROR("Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }

  return true;
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, float& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, double& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, bool& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, int32_t& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, string& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
  if (val.empty())
    throw RosParamNotFoundException(key);
}