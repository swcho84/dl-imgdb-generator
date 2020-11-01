#include "CvtKitty2Bbox.h"

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
  init(argc, argv, "convert_kittyDB_to_bboxDB");
  NodeHandle nh("");

  // reading ros params
  ConfigParam cfg;
  if (!cfg.GetRosParams())
  {
    ROS_ERROR("Wrong params!! Please check the parameter sheet..");
    return 0;
  }

  // converting annotated segmentation DB to bboxs with SIGINT handler
  ROS_INFO("DB generator using kittyDB");
  CvtKtt2Bbox ktt2Bbox(cfg);
  signal(SIGINT, SigIntHandler);

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  // Main loop.
  while (ok())
  {
    switch (cfg.nKttFeatureCase)
    {
      case KITDB_IMGFILE_RESIZER:  // img file resizer
      {
        ROS_INFO("Feature: img file resizer");
        ktt2Bbox.MainLoopImgResizer();
        break;
      }
      case KITDB_XMLFILE_GENERATOR:  // xml file generator
      {
        ROS_INFO("Feature: xml file generator");
        ktt2Bbox.MainLoopBboxGenerator();
        break;
      }
      case KITDB_XMLFILE_CHECKER:  // xml file checker
      {
        ROS_INFO("Feature: xml file checker");
        ktt2Bbox.MainLoopBboxChecker();
        break;
      }      
      case KITDB_XMLFILE_GENERATOR_V2:  // xml file generator v2
      {
        ROS_INFO("Feature: xml file checker v2");
        ktt2Bbox.MainLoopBboxGeneratorV2();
        break;
      }      
      default:
      {
        ROS_INFO("Please check your parameter..");
        break;
      }
    }

    // breaking loop
    if (ktt2Bbox.GetSizeCalcFlag())
      break;

    spinOnce();
    loopRate.sleep();
  }

  ktt2Bbox.~CvtKtt2Bbox();
  ROS_INFO("Work Done: convert_kittyDB_to_bboxDB");

  return 0;
}  // end main()
