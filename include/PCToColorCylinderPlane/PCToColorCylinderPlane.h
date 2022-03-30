// -*- C++ -*-
/*!
 * @file  PCToColorCylinderPlane.h
 * @brief Extract cylinders from point cloud with color
 * @date  $Date$
 *
 * $Id$
 */

#ifndef PCTOCOLORCYLINDERPLANE_H
#define PCTOCOLORCYLINDERPLANE_H

#include <cmath>
#include <vector>
#include <list>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>


#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "pointcloudStub.h"
#include "BasicDataTypeStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

/*!
 * @class PCToColorCylinderPlane
 * @brief Extract cylinders from point cloud with color
 *
 */
class PCToColorCylinderPlane
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  PCToColorCylinderPlane(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~PCToColorCylinderPlane();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  filterXMin
   * - DefaultValue: -0.5
   */
  float m_filterXMin;
  /*!
   * 
   * - Name:  filterXMax
   * - DefaultValue: +0.5
   */
  float m_filterXMax;
  /*!
   * 
   * - Name:  filterYMin
   * - DefaultValue: -1.0
   */
  float m_filterYMin;
  /*!
   * 
   * - Name:  filterYMax
   * - DefaultValue: +1.0
   */
  float m_filterYMax;
  /*!
   * 
   * - Name:  filterZMin
   * - DefaultValue: -1.0
   */
  float m_filterZMin;
  /*!
   * 
   * - Name:  filterZMax
   * - DefaultValue: -0.4
   */
  float m_filterZMax;
  /*!
   * 
   * - Name:  leafSize
   * - DefaultValue: 0.01
   */
  float m_leafSize;
  /*!
   * 
   * - Name:  findingIterationLimit
   * - DefaultValue: 10
   */
  int m_findingIterationLimit;
  /*!
   * 
   * - Name:  segmentationMaxIteration
   * - DefaultValue: 1000
   */
  int m_segmentationMaxIteration;
  /*!
   * 
   * - Name:  segmentationDistanceThreshold
   * - DefaultValue: 0.05
   */
  float m_segmentationDistanceThreshold;
  /*!
   * 
   * - Name:  segmentationRadiusMin
   * - DefaultValue: 0.01
   */
  float m_segmentationRadiusMin;
  /*!
   * 
   * - Name:  segmentationRadiusMax
   * - DefaultValue: 0.05
   */
  float m_segmentationRadiusMax;
  /*!
   * 
   * - Name:  cylinderPointSizeMin
   * - DefaultValue: 100
   */
  int m_cylinderPointSizeMin;
  /*!
   * 
   * - Name:  displayHistogram
   * - DefaultValue: 1
   */
  int m_displayHistogram;
  /*!
   * 
   * - Name:  histogramBinNumber
   * - DefaultValue: 16
   */
  int m_histogramBinNumber;
  /*!
   * 
   * - Name:  saturationMin
   * - DefaultValue: 127
   */
  int m_saturationMin;
  /*!
   * 
   * - Name:  saturationMax
   * - DefaultValue: 255
   */
  int m_saturationMax;
  /*!
   * 
   * - Name:  valueMin
   * - DefaultValue: 0
   */
  int m_valueMin;
  /*!
   * 
   * - Name:  valueMax
   * - DefaultValue: 255
   */
  int m_valueMax;
  /*!
   * 
   * - Name:  sameCylinderCenterDistanceLimit
   * - DefaultValue: 0.02
   */
  float m_sameCylinderCenterDistanceLimit;
  /*!
   * 
   * - Name:  sameCylinderRadiusDistanceLimit
   * - DefaultValue: 0.01
   */
  float m_sameCylinderRadiusDistanceLimit;
  /*!
   * 
   * - Name:  sameCylinderHueDistanceLimit
   * - DefaultValue: 23
   */
  float m_sameCylinderHueDistanceLimit;
  /*!
   * 
   * - Name:  cylinderAccumulationMin
   * - DefaultValue: 10
   */
  int m_cylinderAccumulationMin;
  /*!
   * 
   * - Name:  coordinateTransformationFile
   * - DefaultValue: coordinateTransformation.txt
   */
  std::string m_coordinateTransformationFile;
  /*!
   * 
   * - Name:  calibrationOffsetX
   * - DefaultValue: 0.0
   */
  float m_calibrationOffsetX;
  /*!
   * 
   * - Name:  calibrationOffsetY
   * - DefaultValue: 0.0
   */
  float m_calibrationOffsetY;
  /*!
   * 
   * - Name:  calibrationOffsetZ
   * - DefaultValue: 0.0
   */
  float m_calibrationOffsetZ;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  PointCloudTypes::PointCloud m_pc;
  /*!
   */
  RTC::InPort<PointCloudTypes::PointCloud> m_pcIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedDoubleSeq m_cylinder;
  /*!
   */
  RTC::OutPort<RTC::TimedDoubleSeq> m_cylinderOut;
  RTC::TimedDoubleSeq m_plane;
  /*!
   */
  RTC::OutPort<RTC::TimedDoubleSeq> m_planeOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>
     boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
     bool m_first;
     Eigen::Affine3f m_coordinateTransformation;
     Eigen::Vector4f plane_params_;

     struct Cylinder {
         Eigen::Vector3f center;
         float radius;
         float hue;
         Cylinder() { center = Eigen::Vector3f::Zero(); radius = 0; hue = 0; }
         Cylinder(Eigen::Vector3f c, float r, float h) { center = c; radius = r; hue = h; }
     };
     struct CylinderCandidate {
         Cylinder cylinder;
         int count;
         bool updated;
     };
     std::list<CylinderCandidate> m_cylinderCandidates;
     void selectCylinders(const std::vector<Cylinder>& cylinders, RTC::TimedDoubleSeq& seq);

     struct Plane {
         Eigen::Vector3f center;
         float radius;
         Plane() { center = Eigen::Vector3f::Zero(); radius = 0; }
         Plane(Eigen::Vector3f c, float r) { center = c; radius = r; }
     };
     struct PlaneCandidate {
         Plane plane;
         int count;
         bool updated;
     };
     std::list<PlaneCandidate> m_planeCandidates;
     void selectPlanes(const std::vector<Plane>& planes, RTC::TimedDoubleSeq& seq);


     bool m_rtcError;  //OpenRTM-aist-1.2.0のバグ回避

};


extern "C"
{
  DLL_EXPORT void PCToColorCylinderPlaneInit(RTC::Manager* manager);
};

#endif // PCTOCOLORCYLINDERPLANE_H
