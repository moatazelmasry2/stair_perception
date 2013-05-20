/*
 * test_smartplane.cpp
 *
 *  Created on: Jan 6, 2012
 *      Author: elmasry
 */
#include <stdlib.h>

#include <string>
#include <map>
#include <limits>

#include <iostream>
#include <fstream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "pcl/point_types.h"
#include <pcl/registration/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/surface/organized_fast_mesh.h>

#include <pcl/features/normal_3d.h>
#include "utils/pcl_commons.h"

#include <visualization_msgs/MarkerArray.h>

#include <fhg_pcl/types.h>
#include <fhg_pcl/point_types.h>
//#include <fhg_pcl/fpe/smartline3d.h>
#include <fhg_pcl/fpe/smartplane3dbuilder.h>
#include <fhg_pcl/fpe/edge.h>

#include "fhg_pcl/types/plane3d.h"
#include "fhg_pcl/types/planehistogram.h"
#include "utils/pcl_commons.h"

using namespace boost;
using namespace std;

typedef pcl::PointXYZ XYZD;
typedef Eigen::aligned_allocator<XYZD> Alloc;
typedef Eigen::aligned_allocator<pcl::PointXYZRGBNormal> AllocRGBNormal;
typedef pcl::SmartPlane3D<XYZD> SmartPlane3D;
typedef pcl::LineSegment<XYZD> LineSegment;
typedef pcl::SmartLine3D<XYZD> SmartLine3D;

typedef std::vector<LineSegment, Alloc> LineSegmentVtr;
typedef std::vector<LineSegmentVtr, Alloc> LineSegmentVtrVtr;
typedef std::vector<XYZD, Alloc> PointsVector;
typedef std::vector<SmartPlane3D, Alloc> PlaneVector;
typedef std::vector<SmartLine3D, Alloc> SmartLineVector;
typedef std::list<SmartPlane3D, Alloc> PlaneList;
typedef std::vector<pcl::Edge<XYZD>, Alloc> EdgesVector;

class Test_FPE
{
public:

  ros::Publisher pub;
  ros::Publisher colorSegPub2;

  static const bool DIVIDEBY1000 = true;
  int horznumberOfRays;
  double MSECUTOFF;
  double MAXPOINTDIST;
  double MINSEGLENGTH;

  double MAXSEGMENTDISTANCE;

  //double isINTERSECTINGPARAM = 0.00000005;
  double isINTERSECTINGPARAM;
  double maxPARALELLANG_DEG;
  //double maxMSE = 0.00001;
  double MAXMSE;int NN;int MINNUMPOINTSPERPLANE;
  bool planes_merge;int mergeCycles;
  bool colorEdges;

  string inputPath;
  string outputFolder;int cloudSeq;

  bool ledger_update, divideby1000, rotate_all;
  double MAXPLANEDISTANCE, maxAngleDeviation;
  pcl::RobotPosition robotPosition;int pos;int camera_tilt;

  void
  readFile (pcl::PointCloud<XYZD>& cloud)
  {
    string line;

    ifstream inFile (inputPath.c_str ());

    if (inFile.is_open ())
    {
      char_separator<char> sep (" ");
      int numLines = 0;
      while (inFile.good ())
      {
        getline (inFile, line);
        tokenizer<char_separator<char> > tokens (line, sep);
        XYZD point;

        char * pEnd;
        point.x = strtod (line.c_str (), &pEnd);
        point.y = strtod (pEnd, &pEnd);
        point.z = strtod (pEnd, NULL);
        if (isnan (point.x))
        {
          point.x = 0.0;
        }
        if (isnan (point.y))
        {
          point.y = 0.0;
        }
        if (isnan (point.z))
        {
          point.z = 0.0;
        }
        //point.id = numLines;
        cloud.push_back (point);
        numLines++;
      }
      cloud.width = horznumberOfRays;
      cloud.height = cloud.size () / horznumberOfRays;
      inFile.close ();
    }
  }

  void
  writeCloud (pcl::PointCloud<pcl::PointXYZRGB> cloudBlob, string prefixFileName)
  {
    ofstream myfile;
    string path = outputFolder.c_str ();
    path.append (prefixFileName.c_str ());

    path.append (".pcd");
    myfile.open (path.c_str ());

    myfile << "#.PCD v.7 - Point Cloud Data file format" << std::endl;
    myfile << "VERSION .7" << std::endl;
    myfile << "FIELDS x y z rgb" << std::endl;
    myfile << "SIZE 4 4 4 4" << std::endl;
    myfile << "TYPE F F F F" << std::endl;
    myfile << "COUNT 1 1 1 1" << std::endl;
    myfile << "WIDTH " << cloudBlob.size () << std::endl;
    myfile << "HEIGHT 1" << std::endl;
    myfile << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    myfile << "POINTS " << cloudBlob.size () << std::endl;
    myfile << "DATA ascii" << std::endl;
    pcl::PointXYZRGB point;

    typedef std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > RGBPointsVector;

    for (RGBPointsVector::iterator it = cloudBlob.begin (); it != cloudBlob.end (); it++)
    {
      point = *it;
      myfile << point.x << " " << point.y << " " << point.z << " " << point.rgb << std::endl;
    }
    myfile.close ();
  }

  float
  generateColor ()
  {
    uint8_t r = rand () % 255;
    uint8_t g = rand () % 255;
    uint8_t b = rand () % 255;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    return *reinterpret_cast<float*> (&rgb);
  }

  float
  generateColor (int r, int g, int b)
  {
    uint8_t _r = (uint8_t)r;
    uint8_t _g = (uint8_t)g;
    uint8_t _b = (uint8_t)b;
    uint32_t rgb = ((uint32_t)_r << 16 | (uint32_t)_g << 8 | (uint32_t)_b);
    return *reinterpret_cast<float*> (&rgb);
  }

  inline void
  colorPoint (pcl::PointCloud<pcl::PointXYZRGB> &cloud, XYZD point, float color, bool divideby1000)
  {
    pcl::PointXYZRGB pNew;
    pNew.x = divideby1000 ? point.x / 1000 : point.x;
    pNew.y = divideby1000 ? point.y / 1000 : point.y;
    pNew.z = divideby1000 ? point.z / 1000 : point.z;
    pNew.rgb = color;
    cloud.push_back (pNew);
  }

  inline void
  colorPoints (pcl::PointCloud<pcl::PointXYZRGB> &cloud, PointsVector points, float color, bool divideby1000)
  {

    for (PointsVector::iterator it = points.begin (); it != points.end (); it++)
    {
      colorPoint (cloud, *it, color, divideby1000);
    }
  }

  /**
   * colors a plane and adds it to the cloud
   */
  inline void
  colorPlane (pcl::PointCloud<pcl::PointXYZRGB> &cloud, SmartPlane3D& plane, float color, bool divideby1000)
  {

    PointsVector points = plane.getPoints ();

    for (PointsVector::iterator it = points.begin (); it != points.end (); it++)
    {
      colorPoint (cloud, *it, color, divideby1000);
    }
  }

  /*inline void
   addPlaneToCloud (pcl::PointCloud<pcl::PointXYZRGB>& cloud, SmartPlane3D plane, float color)
   {
   SmartLineVector lines = plane.supportingSmartLines;
   SmartLineVector::iterator lineIt;
   for (int j = 0; j < (int)lines.size (); j++)
   {
   SmartLine3D line = lines[j];
   PointsVector pVec = lines[j].supportingPoints;

   PointsVector::iterator pIt;
   for (pIt = pVec.begin (); pIt != pVec.end (); pIt++)
   {
   XYZD pOld = *pIt;
   colorPoint (cloud, pOld, color, false);
   }
   }
   }*/

  inline void
  addPlaneToCloud (pcl::PointCloud<pcl::PointXYZRGB>& cloud, SmartPlane3D plane, float color, bool divideby1000)
  {
    PointsVector points = plane.getPoints ();
    for (size_t i = 0; i < points.size (); i++)
    {
      XYZD& pOld = points[i];
      colorPoint (cloud, pOld, color, divideby1000);
    }
  }

  inline void
  writePlane (SmartPlane3D plane, string path)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.header.frame_id = "/base_link";
    cloud.header.stamp = ros::Time::now ();
    addPlaneToCloud (cloud, plane, generateColor (), true);
    writeCloud (cloud, path);
  }

  inline void
  writeAllPlanes (pcl::SmartPlane3DBuilder<XYZD> builder, bool divideby1000)
  {
    for (int i = 0; i <= builder.getLastPlaneLabel (); i++)
    {
      SmartPlane3D plane = builder.getPlanes ()[i];
      char buf[20];
      sprintf (buf, "plane%d", plane.id);
      string path = buf;
      pcl::PointCloud<pcl::PointXYZRGB> out;
      extractPlane (plane, true, divideby1000, out);
      extractLeftRightBorders (plane, out, divideby1000);
      //float blue = generateColor(0,0,255), green = generateColor(0,255,0);
      //extractTopBottomBorder(plane, blue, green, out);
      writeCloud (out, path);
    }
  }

  inline void
  extractLeftRightBorders (pcl::SmartPlane3D<XYZD> plane, pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool divideby1000)
  {
    float red = generateColor (255, 0, 0);
    colorPoints (cloud, plane.getLeftBorder ().getPoints (), red, divideby1000);
    colorPoints (cloud, plane.getRightBorder ().getPoints (), red, divideby1000);
  }

  inline void
  extractLeftRightRawBorders (pcl::SmartPlane3D<XYZD> plane, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                              bool divideby1000)
  {
    float red = generateColor (255, 0, 0);
    colorPoints (cloud, plane.getLeftBorder ().getInputPoints (), red, divideby1000);
    colorPoints (cloud, plane.getRightBorder ().getInputPoints (), red, divideby1000);
  }

  void
  extractLeftRightBorders (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                           bool divideby1000)
  {
    for (int i = 0; i <= builder.getLastPlaneLabel (); i++)
    {
      SmartPlane3D plane = builder.getPlanes ()[i];
      extractLeftRightBorders (plane, cloud, divideby1000);
    }
  }

  void
  extractLeftRightRawBorders (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                              bool divideby1000)
  {
    float red = generateColor (255, 0, 0);
    for (int i = 0; i <= builder.getLastPlaneLabel (); i++)
    {
      SmartPlane3D plane = builder.getPlanes ()[i];
      colorPoints (cloud, plane.getLeftBorder ().getInputPoints (), red, divideby1000);
      colorPoints (cloud, plane.getRightBorder ().getInputPoints (), red, divideby1000);

      SmartLineVector lines = builder.getPlanes ()[i].supportingSmartLines;
    }
  }

  inline void
  extractPlane (SmartPlane3D plane, bool colored, bool divideby1000, pcl::PointCloud<pcl::PointXYZRGB>& out)
  {
    float white = generateColor (255, 254, 254), color;
    SmartLineVector lines = plane.supportingSmartLines;
    SmartLineVector::iterator lineIt;
    if (colored)
    {
      color = generateColor ();
    }

    for (int j = 0; j < (int)lines.size (); j++)
    {
      SmartLine3D line = lines[j];
      PointsVector pVec = lines[j].supportingPoints;

      PointsVector::iterator pIt;
      for (pIt = pVec.begin (); pIt != pVec.end (); pIt++)
      {
        if (colored)
        {
          colorPoint (out, *pIt, color, divideby1000);
        }
        else
        {
          colorPoint (out, *pIt, white, divideby1000);
        }
      }
    }
  }
  void
  extractPlanes (pcl::SmartPlane3DBuilder<XYZD> builder, bool colored, pcl::PointCloud<pcl::PointXYZRGB>& out,
                 bool divideby1000)
  {
    for (int i = 0; i <= builder.getLastPlaneLabel (); i++)
    {
      SmartPlane3D plane = builder.getPlanes ().at (i);
      extractPlane (plane, colored, divideby1000, out);

    }
  }

  void
  extractColoredCloud (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                       bool divideby1000)
  {
    extractPlanes (builder, true, cloud, divideby1000);
    //extractLeftRightBorders (builder, cloud, divideby1000);
  }

  void
  extractColoredSegments2 (pcl::PointCloud<pcl::PointXYZRGB>& cloud, const pcl::PointCloud<XYZD>& cloudBlob,
                           SmartLineVector lines, bool divideby1000)
  {
    map<int, float> colors;

    cloud.header = cloudBlob.header;
    for (SmartLineVector::iterator lineIt = lines.begin (); lineIt != lines.end (); lineIt++)
    {
      float color = generateColor ();

      SmartLine3D l = *lineIt;

      PointsVector pVec = (*lineIt).supportingPoints;
      PointsVector::iterator pIt;
      for (pIt = pVec.begin (); pIt != pVec.end (); pIt++)
      {
        XYZD pOld = *pIt;
        colorPoint (cloud, pOld, color, divideby1000);
      }
    }
  }

  void
  publishCloud (pcl::PointCloud<pcl::PointXYZRGB> cloud, ros::Publisher publisher)
  {
    cloud.header.stamp = ros::Time::now ();
    cloud.header.frame_id = "/base_link";
    cloud.header.seq = cloudSeq++;
    publisher.publish (cloud.makeShared ());
  }

  void
  publishMarkers (pcl::SmartPlane3DBuilder<XYZD> builder)
  {
    visualization_msgs::MarkerArray a;

    PlaneVector planes = builder.getPlanes ();
  }

  void
  writeSteps (pcl::SmartPlane3DBuilder<XYZD> builder)
  {

    //builder.computeSteps ();

    PlaneList risers = builder.getRisers ();
    PlaneList treads = builder.getTreads ();

    pcl::PointCloud<pcl::PointXYZRGB> riserCloud, treadsCloud;

    for (PlaneList::iterator it = treads.begin (); it != treads.end (); it++)
    {
      SmartPlane3D plane = *it;
      addPlaneToCloud (treadsCloud, plane, generateColor (), DIVIDEBY1000);
    }
    writeCloud (treadsCloud, "treads");

    for (PlaneList::iterator it = risers.begin (); it != risers.end (); it++)
    {
      SmartPlane3D plane = *it;
      addPlaneToCloud (riserCloud, plane, generateColor (), DIVIDEBY1000);
    }
    writeCloud (riserCloud, "risers");
  }

  void
  extractEdges (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& out, bool divideby1000)
  {

    //extractPlanes (builder, false, out, divideby1000);
    float red = generateColor (253, 0, 0);
    EdgesVector edges = builder.getEdgesVector ();
    std::cout << "edges=" << edges.size () << std::endl;
    for (EdgesVector::iterator it = edges.begin (); it != edges.end (); it++)
    {
      pcl::Edge<XYZD> edge = *it;
      PointsVector points = edge.getPoints ();
      for (PointsVector::iterator pIt = points.begin (); pIt != points.end (); pIt++)
      {
        colorPoint (out, *pIt, red, divideby1000);
      }
    }
  }

  void
  extractBordersCloud (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& out,
                       bool divideby1000)
  {

    extractPlanes (builder, false, out, divideby1000);
    extractLeftRightBorders (builder, out, divideby1000);

    float green = generateColor (0, 255, 0);
    float blue = generateColor (0, 0, 255);

    for (int i = 0; i <= builder.getLastPlaneLabel (); i++)
    {
      SmartPlane3D plane = builder.getPlanes ()[i];
      pcl::Edge<XYZD> topBorder = plane.getTopBorder (), bottomBorder = plane.getBottomBorder ();

      colorPoints (out, topBorder.getPoints (), blue, divideby1000);
      colorPoints (out, bottomBorder.getPoints (), green, divideby1000);
    }
  }

  void
  extractRawBordersCloud (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& out,
                          bool divideby1000)
  {

    extractPlanes (builder, false, out, divideby1000);
    extractLeftRightRawBorders (builder, out, divideby1000);

    float green = generateColor (0, 255, 0);
    float blue = generateColor (0, 0, 255);

    for (int i = 0; i <= builder.getLastPlaneLabel (); i++)
    {
      SmartPlane3D plane = builder.getPlanes ()[i];
      extractTopBottomBorder (plane, blue, green, out);
    }
  }

  /**
   * extracts raw points for top/bottom borders
   */
  void
  extractTopBottomBorder (SmartPlane3D plane, float topColor, float bottomColor, pcl::PointCloud<pcl::PointXYZRGB>& out)
  {
    pcl::Edge<XYZD> topBorder = plane.getTopBorder (), bottomBorder = plane.getBottomBorder ();
    colorPoints (out, topBorder.getInputPoints (), topColor, divideby1000);
    colorPoints (out, bottomBorder.getInputPoints (), bottomColor, divideby1000);
  }

  void
  tests (pcl::SmartPlane3DBuilder<XYZD> builder)
  {

    //builder.logPlanes ();

    PlaneList risers = builder.getRisers ();
    PlaneList treads = builder.getTreads ();

    /*std::cout << "printing treads, size=" << treads.size() << std::endl;
     for (PlaneList::iterator it = treads.begin (); it != treads.end (); it++)
     {
     SmartPlane3D plane = *it;
     std::cout << "id=" << plane.id << ", center=" << plane.center[0] << "," << plane.center[1] << "," << plane.center[2] << std::endl;
     }

     std::cout << "printing risers size=" << risers.size() << std::endl;
     for (PlaneList::iterator it = risers.begin (); it != risers.end (); it++)
     {
     SmartPlane3D plane = *it;
     std::cout << "id=" << plane.id <<", center=" << plane.center[0] << "," << plane.center[1] << "," << plane.center[2] << std::endl;
     }*/
  }

  void
  extractReconstructMeshes (pcl::SmartPlane3DBuilder<XYZD> builder, pcl::PointCloud<pcl::PointXYZRGB>& out,
                            bool divideby1000)
  {

  }

  void
  writeHistogramPlanes (pcl::SmartPlane3DBuilder<XYZD> builder)
  {
    PlaneVector planes = builder.getHistPlanes ();

    printf ("Hist planes.size=%d", planes.size ());
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (PlaneVector::iterator it = planes.begin (); it != planes.end (); it++)
    {
      SmartPlane3D& plane = *it;
      addPlaneToCloud (cloud, plane, generateColor (), true);
    }
    //pcl::io::savePCDFile("histogramplanes.pcd", cloud);
    writeCloud (cloud, "histogram");
  }
};

Test_FPE fpe;

void
callback (const sensor_msgs::PointCloud2ConstPtr& cloudBlob)
{

  pcl::PointCloud<pcl::PointXYZRGB> source, tmpCloud;
  //pcl::fromROSMsg (*cloudBlob, cloud);

  pcl::fromROSMsg (*cloudBlob, tmpCloud);

  Eigen::Vector3f axis (0, 1, 0);

  Quaternion<float> q;
  q = Quaternion<float> (-0.500398163355, 0.499999841466, -0.499601836645, 0.499999841466);
  pcl::transformPointCloud (tmpCloud, source, Eigen::Vector3f (0, 0, 0), q);
  fpe.writeCloud (tmpCloud, "source");

  q = AngleAxis<float> (fpe.camera_tilt * M_PI / 180, axis);
  pcl::transformPointCloud (source, source, Eigen::Vector3f (0, 0, 0), q);
  fpe.writeCloud (source, "rotated");

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::copyPointCloud (source, cloud);

  for (PointsVector::iterator it = cloud.points.begin (); it != cloud.points.end (); it++)
  {
    XYZD& p = *it;
    if (isnan (p.x) || isnan (p.y) || isnan (p.z))
    {
      p.x = 0.0;
      p.y = 0.0;
      p.z = 0.0;
    }
    else
    {
      p.x *= 1000;
      p.y *= 1000;
      p.z *= 1000;
    }
  }

  std::cout << "init" << std::endl;
  pcl::SmartPlane3DBuilder<XYZD> builder (fpe.MSECUTOFF, fpe.MAXPOINTDIST, fpe.MINSEGLENGTH, fpe.MAXSEGMENTDISTANCE,
                                          fpe.isINTERSECTINGPARAM, fpe.maxPARALELLANG_DEG, fpe.MAXMSE, fpe.NN);
  builder.MINNUMPOINTSPERPLANE = fpe.MINNUMPOINTSPERPLANE;
  builder.planes_merge = fpe.planes_merge;
  builder.MAXPLANEDISTANCE = fpe.MAXPLANEDISTANCE;
  builder.maxAngleDeviation = fpe.maxAngleDeviation;
  builder.ledger_update = fpe.ledger_update;
  builder.rotate_all = fpe.rotate_all;
  builder.robotPosition = fpe.robotPosition;
  builder.camera_tilt = fpe.camera_tilt;
  builder.setInputCloud (cloud.makeShared ());
  std::cout << "before calculate" << std::endl;
  builder.calculate ();

  builder.logPlanes ();
  pcl::PointCloud<pcl::PointXYZRGB> coloredCloud;

  printf ("writing histogram\n");
  fpe.writeHistogramPlanes (builder);

  std::cout << "write fpe_planes" << std::endl;
  fpe.extractColoredCloud (builder, coloredCloud, fpe.divideby1000);
  fpe.writeCloud (coloredCloud, "fpe_planes");

  std::cout << "write separate planes" << std::endl;
  fpe.writeAllPlanes (builder, fpe.divideby1000);
  std::cout << "write steps" << std::endl;
  fpe.writeSteps (builder);

  std::cout << "write borders" << std::endl;
  coloredCloud.clear ();
  fpe.extractBordersCloud (builder, coloredCloud, fpe.divideby1000);
  fpe.writeCloud (coloredCloud, "borders");

  std::cout << "write raw borders" << std::endl;
  coloredCloud.clear ();
  fpe.extractRawBordersCloud (builder, coloredCloud, fpe.divideby1000);
  fpe.writeCloud (coloredCloud, "rawborders");

  std::cout << "write edges" << std::endl;
  coloredCloud.clear ();
  fpe.extractEdges (builder, coloredCloud, fpe.divideby1000);
  fpe.writeCloud (coloredCloud, "edges");

  //tests (builder);
  std::cout << "done" << std::endl;
  exit (0);
}

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  std::string topic;
  topic = "/cloud_pcd";
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> (topic, 1, callback);

  fpe.MINNUMPOINTSPERPLANE = 0;
  fpe.planes_merge = false;
  fpe.colorEdges = false;
  fpe.ledger_update = false;
  fpe.divideby1000 = false;
  fpe.rotate_all = false;
  fpe.MAXPLANEDISTANCE = 100;
  fpe.maxAngleDeviation = 10;
  fpe.camera_tilt = 0;

  ros::NodeHandle priv_nh ("~");
  std::string prefix_;
  priv_nh.getParam ("horznumberOfRays", fpe.horznumberOfRays);
  priv_nh.getParam ("robotPosition", fpe.pos);
  switch (fpe.pos)
  {
    case 0:
      fpe.robotPosition = pcl::UP;
      break;
    case 1:
      fpe.robotPosition = pcl::DOWN;
      break;
    default:
      fpe.robotPosition = pcl::DOWN;
  }

  priv_nh.getParam ("MSECUTOFF", fpe.MSECUTOFF);
  priv_nh.getParam ("MAXPOINTDIST", fpe.MAXPOINTDIST);
  priv_nh.getParam ("MINSEGLENGTH", fpe.MINSEGLENGTH);
  priv_nh.getParam ("MAXSEGMENTDISTANCE", fpe.MAXSEGMENTDISTANCE);
  priv_nh.getParam ("isINTERSECTINGPARAM", fpe.isINTERSECTINGPARAM);
  priv_nh.getParam ("maxPARALELLANG_DEG", fpe.maxPARALELLANG_DEG);
  priv_nh.getParam ("MAXMSE", fpe.MAXMSE);
  priv_nh.getParam ("NN", fpe.NN);
  priv_nh.getParam ("MINNUMPOINTSPERPLANE", fpe.MINNUMPOINTSPERPLANE);
  priv_nh.getParam ("colorEdges", fpe.colorEdges);

  priv_nh.getParam ("rotate_all", fpe.rotate_all);
  priv_nh.getParam ("planes_merge", fpe.planes_merge);
  priv_nh.getParam ("mergeCycles", fpe.mergeCycles);
  priv_nh.getParam ("ledger_update", fpe.ledger_update);
  priv_nh.getParam ("divideby1000", fpe.divideby1000);
  priv_nh.getParam ("MAXPLANEDISTANCE", fpe.MAXPLANEDISTANCE);
  priv_nh.getParam ("maxAngleDeviation", fpe.maxAngleDeviation);
  priv_nh.getParam ("camera_tilt", fpe.camera_tilt);

  priv_nh.getParam ("inputPath", fpe.inputPath);
  priv_nh.getParam ("outputFolder", fpe.outputFolder);

  // Create a ROS publisher for the output point cloud
  ros::spin ();
}
