// Cloud builder: depth Image + CameraInfo → organized PointCloud<XYZ>.
//
// Bypasses the realsense2_camera wrapper's PointCloud2 topic — many bag
// recordings have `pointcloud.ordered_pc=false`, producing a NaN-
// stripped / unorganized cloud that breaks anything assuming the depth-
// image grid layout. Going through depth Image + CameraInfo gives us a
// guaranteed organized cloud in any environment.

#ifndef REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_
#define REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_

#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace realsense_fast_mesh_baseline
{

struct MeshBuilderParams
{
  // Depth-image pixel stride. stride=1 keeps every pixel; stride=N
  // takes every Nth pixel along both axes (N²-fold vertex reduction).
  // The output cloud stays organized so its width/height shrink to
  // (W/N, H/N). CameraInfo intrinsics are applied to the ORIGINAL
  // pixel coordinates, so depth back-projection stays geometrically
  // correct regardless of stride.
  std::uint32_t pixel_stride{1u};

  // Spherical distance cutoff from the camera origin (meters). Points
  // with sqrt(x²+y²+z²) > max_distance_m are set to NaN. RealSense
  // depth noise grows ∝ d², so far points are mostly noise — trimming
  // them here keeps downstream consumers from chewing on garbage.
  // Set to 0 or negative to disable.
  float max_distance_m{3.5f};
};

// Build an organized pcl::PointCloud<PointXYZ> directly from a depth
// image + intrinsics. Output is in the depth image's optical frame;
// the caller is responsible for header.frame_id when serializing /
// looking up TF.
//
// Supported encodings:
//   * 16UC1 / mono16 — depth in millimeters (RealSense default).
//   * 32FC1          — depth in meters.
// Throws std::runtime_error on other encodings.
//
// Output cloud:
//   width  = floor(image.width  / params.pixel_stride)
//   height = floor(image.height / params.pixel_stride)
// Invalid pixels (depth=0 or non-finite) and points outside
// max_distance_m are set to NaN.
pcl::PointCloud<pcl::PointXYZ>::Ptr build_organized_cloud_from_depth(
  const sensor_msgs::msg::Image & depth_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const MeshBuilderParams & params);

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_
