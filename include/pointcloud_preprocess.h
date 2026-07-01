#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib.h"

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range)
                                  )
// clang-format on

namespace hesai_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (std::uint16_t, ring, ring)
)
// clang-format on

namespace robosense {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace robosense
   // clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace tw360{
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;

  float intensity;
  int channel;
  float angle;
  int echo;
  int block;           /*For duetto*/
  unsigned int t_sec;  /* The value represents seconds since 1900-01-01 00:00:00
                          (the UNIX epoch).*/
  unsigned int t_usec; /* remaining microseconds */
  float time;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace tw360
  // clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
    tw360::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (int, channel, channel)(float, angle, angle)(int, echo, echo)
    (int, block, block)
    (unsigned int, t_sec, t_sec)(unsigned int, t_usec, t_usec)
    (float, time, time))


namespace faster_lio {

enum class LidarType { AVIA = 1, VELO32, OUST64,  HESAIxt32, ROBOSENSE, TW}; //{1, 2, 3, 4}

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    bool &FeatureEnabled() { return feature_enabled_; }
    float &TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

   private:
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
    void Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void HesaiHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void TW360Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    bool feature_enabled_ = false;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;
};
}  // namespace faster_lio

#endif
