#include "CvtSeg2Bbox.h"

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
  init(argc, argv, "convert_cityscapes_segDB_to_bboxDB");
  NodeHandle nh("");

  // reading ros params
  ConfigParam cfg;
  if (!cfg.GetRosParams())
  {
    ROS_ERROR("Wrong params!! Please check the parameter sheet..");
    return 0;
  }

  // converting annotated segmentation DB to bboxs with SIGINT handler
  ROS_INFO("DB generator using cityscapeDB");
  CvtSeg2Bbox seg2Bbox(cfg);
  signal(SIGINT, SigIntHandler);

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  // Main loop.
  while (ok())
  {
    switch (cfg.nFeatureCase)
    {
      case CITYDB_IMGFILE_RESIZER:  // img file resizer
      {
        ROS_INFO("Feature: img file resizer");
        seg2Bbox.MainLoopImgResizer();
        break;
      }
      case CITYDB_XMLFILE_GENERATOR:  // xml file generator
      {
        ROS_INFO("Feature: xml file generator");
        seg2Bbox.MainLoopBboxGenerator();
        break;
      }
      case CITYDB_XMLFILE_CHECKER:  // xml file checker
      {
        ROS_INFO("Feature: xml file checker");
        seg2Bbox.MainLoopBboxChecker();
        break;
      }
      case KARIDB_SEMANTIC_SEGMENTATION_LABEL_CONVERTER:
      {
        ROS_INFO("Feature: semantic segmentation label converter for kariDB");
        seg2Bbox.MainLoopSemanticSegLabelConverter();
        break;
      }
      case ETRIDB_BBOX_DB_YOLO_CONVERTER:
      {
        ROS_INFO("Feature: bbox label converter for etriDB (from Pascal VOC xml to YOLO");
        seg2Bbox.MainLoopBboxYoloLabelConverter();
        break;
      }
      default:
      {
        ROS_INFO("Please check your parameter..");
        break;
      }
    }

    // breaking loop
    if (seg2Bbox.GetSizeCalcFlag())
      break;

    spinOnce();
    loopRate.sleep();
  }

  seg2Bbox.~CvtSeg2Bbox();
  ROS_INFO("Work Done: convert_cityscapes_segDB_to_bboxDB");

  return 0;
}  // end main()
