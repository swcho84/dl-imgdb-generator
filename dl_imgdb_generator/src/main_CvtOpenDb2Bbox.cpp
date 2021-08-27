#include "CvtOpenDb2Bbox.h"

using namespace std;
using namespace ros;
using namespace cv;

// using SIGINT handler
void SigIntHandler(int param)
{
  ROS_INFO("User pressed Ctrl+C..forced exit..");
  exit(1);
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  // Set up ROS.
  init(argc, argv, "convert_specified_openDB_to_bboxDB");
  NodeHandle nh("");

  // reading ros params
  ConfigParam cfg;
  if (!cfg.GetRosParams())
  {
    ROS_ERROR("Wrong params!! Please check the parameter sheet..");
    return 0;
  }

  // converting annotated segmentation DB to bboxs with SIGINT handler
  ROS_INFO("DB generator using specified openDB");
  CvtOpenDb2Bbox opDb2Bbox(cfg);
  signal(SIGINT, SigIntHandler);

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  // Main loop.
  while (ok())
  {
    switch (cfg.nOpDbFeatureCase)
    {
      case OPENDB_IMGFILE_RESIZER:  // img file resizer
      {
        ROS_INFO("Feature: img file resizer");
        opDb2Bbox.MainLoopImgResizer();
        break;
      }
      case OPENDB_XMLFILE_GENERATOR:  // xml file generator
      {
        ROS_INFO("Feature: xml file generator");
        opDb2Bbox.MainLoopBboxGenerator();
        break;
      }
      case OPENDB_XMLFILE_CHECKER:  // xml file checker
      {
        ROS_INFO("Feature: xml file checker");
        opDb2Bbox.MainLoopBboxChecker();
        break;
      }
      case OPENDB_XMLFILE_YOLO_CONVERTER:  // xml file checker
      {
        ROS_INFO("Feature: converter from xml file to yolo file");
        opDb2Bbox.MainLoopBboxYoloLabelConverter();
        break;
      }      
      default:
      {
        ROS_INFO("Please check your parameter..");
        break;
      }
    }

    // breaking loop
    if (opDb2Bbox.GetSizeCalcFlag())
      break;

    spinOnce();
    loopRate.sleep();
  }

  opDb2Bbox.~CvtOpenDb2Bbox();
  ROS_INFO("Work Done: convert_specified_openDB_to_bboxDB");

  return 0;
}  // end main()
