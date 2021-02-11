#include "CvtKdarpa2Bbox.h"

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
  init(argc, argv, "convert_KdarpaDB_to_bboxDB");
  NodeHandle nh("");

  // reading ros params
  ConfigParam cfg;
  if (!cfg.GetRosParams())
  {
    ROS_ERROR("Wrong params!! Please check the parameter sheet..");
    return 0;
  }

  // converting annotated segmentation DB to bboxs with SIGINT handler
  ROS_INFO("DB generator using KdarpaDB");
  CvtKdarpa2Bbox kdarpa2Bbox(cfg);
  signal(SIGINT, SigIntHandler);

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  // Main loop.
  while (ok())
  {
    switch (cfg.nKdarpaFeatureCase)
    {
      case KDARPADB_IMGFILE_RESIZER:  // img file resizer
      {
        ROS_INFO("Feature: img file resizer");
        kdarpa2Bbox.MainLoopImgResizer();
        break;
      }
      case KDARPADB_XMLFILE_GENERATOR:  // xml file generator
      {
        ROS_INFO("Feature: xml file generator");
        kdarpa2Bbox.MainLoopBboxGenerator();
        break;
      }
      case KDARPADB_XMLFILE_CHECKER:  // xml file checker
      {
        ROS_INFO("Feature: xml file checker");
        kdarpa2Bbox.MainLoopBboxChecker();
        break;
      }
      default:
      {
        ROS_INFO("Please check your parameter..");
        break;
      }
    }

    // breaking loop
    if (kdarpa2Bbox.GetSizeCalcFlag())
      break;

    spinOnce();
    loopRate.sleep();
  }

  kdarpa2Bbox.~CvtKdarpa2Bbox();
  ROS_INFO("Work Done: convert_kittyDB_to_bboxDB");

  return 0;
}  // end main()
