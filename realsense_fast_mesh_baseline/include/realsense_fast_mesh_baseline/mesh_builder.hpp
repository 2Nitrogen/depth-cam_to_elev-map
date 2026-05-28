// Thin ROS-free wrappers over PCL's IntegralImageNormalEstimation and
// OrganizedFastMesh. Both require organized point clouds (height > 1,
// width = depth-image columns); the caller is responsible for handing
// the cloud in WITHOUT removing NaN entries (which would destroy the
// organized structure both algorithms depend on).
//
// Coordinate-system note: IntegralImageNormalEstimation interprets the
// cloud's z axis as the optical depth direction for its discontinuity
// test (max_depth_change_factor). Therefore the normal estimation must
// run on the cloud BEFORE any TF rotation that moves z away from the
// optical axis. OrganizedFastMesh has no such assumption (its only 3D
// check is the max_edge_length cap), so the triangulation may run on
// either the camera-frame or the target-frame cloud — the choice here
// is to run it on the target-frame cloud so that mesh.cloud is already
// in publish coordinates.

#ifndef REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_
#define REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_

#include <cstdint>
#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace realsense_fast_mesh_baseline
{

enum class TriangulationType
{
  TriangleRightCut,
  TriangleLeftCut,
  TriangleAdaptiveCut,
};

// Parse a yaml string into the enum. Throws std::runtime_error on an
// invalid value. QUAD_MESH is intentionally rejected: the Marker output
// path is TRIANGLE_LIST only.
TriangulationType parse_triangulation_type(const std::string & s);
const char * triangulation_type_str(TriangulationType t);

// Base 3D edge length (m) per unit of pixel stride, used to compute
// MeshBuilderParams::triangle_max_edge_length at node construction.
// The node populates the field as `pixel_stride * kBaseEdgeLengthPerStride`
// so the threshold scales linearly with vertex spacing — vertices are
// stride× farther apart in 3D at a given depth, so the rejected-as-
// "spanning a discontinuity" threshold needs to grow proportionally.
//
// 0.10 m is generous (≥ ~6× the typical lateral spacing at 5 m depth
// for a RealSense-class focal length) while still cutting cleanly at
// step edges. Tune here if needed; this is intentionally NOT a yaml
// knob so that the stride / edge relationship stays a single decision.
inline constexpr float kBaseEdgeLengthPerStride = 0.10f;

struct MeshBuilderParams
{
  // Depth-image pixel stride used when reconstructing the organized
  // cloud. stride=1 keeps every pixel (densest mesh); stride=2 takes
  // every 2nd pixel along both axes (4x fewer vertices); stride=N
  // produces an N²-fold vertex reduction. The output cloud width /
  // height shrink accordingly but stays organized so OrganizedFastMesh
  // still works. CameraInfo intrinsics are applied to the ORIGINAL
  // pixel coordinates of each sampled pixel, so depth back-projection
  // remains geometrically correct regardless of stride.
  std::uint32_t pixel_stride{1u};

  // Per-pixel cross-product normal estimation.
  //   normal_smoothing_size  : gradient sample step in cloud-grid pixels
  //                            (1 = raw per-pixel, larger = smoother /
  //                            less sensor-noise sensitive). Note this
  //                            is in DOWNSAMPLED grid units when
  //                            pixel_stride > 1.
  //   max_depth_change_factor: relative depth-jump threshold for
  //                            discontinuity rejection (0 disables).
  float normal_smoothing_size{20.0f};
  float max_depth_change_factor{0.02f};

  // OrganizedFastMesh
  //   triangle_max_edge_length: maximum 3D edge length (m) for an
  //                             accepted triangle. Auto-computed by the
  //                             node as `pixel_stride *
  //                             kBaseEdgeLengthPerStride` — NOT a yaml
  //                             knob. See the constant's docstring for
  //                             rationale.
  float             triangle_max_edge_length{kBaseEdgeLengthPerStride};
  TriangulationType triangulation_type{TriangulationType::TriangleAdaptiveCut};
};

// Build an organized pcl::PointCloud<PointXYZ> directly from a depth
// image + intrinsics, bypassing the wrapper's PointCloud2 topic. Needed
// because many bag recordings (and some wrapper configs) publish the
// pre-filtered, NaN-stripped, *unorganized* /...depth/color/points,
// which the organized PCL algorithms cannot consume. Reconstructing
// from the raw depth image + CameraInfo gives us a guaranteed organized
// cloud regardless of upstream config.
//
// Supported depth encodings:
//   - 16UC1 / mono16 : depth in millimeters (RealSense default).
//   - 32FC1          : depth in meters.
// Throws std::runtime_error on other encodings.
//
// Output cloud has width = floor(image.width / params.pixel_stride),
// height = floor(image.height / params.pixel_stride), with invalid
// pixels (depth=0 or non-finite) set to NaN. The point's reference
// frame is implicitly the depth image's optical frame; the caller is
// responsible for header.frame_id when serializing / looking up TF.
pcl::PointCloud<pcl::PointXYZ>::Ptr build_organized_cloud_from_depth(
  const sensor_msgs::msg::Image & depth_image,
  const sensor_msgs::msg::CameraInfo & camera_info,
  const MeshBuilderParams & params);

// Estimate per-pixel normals on an organized cloud via cross product
// of (right - center) × (down - center). NaN-aware: any pixel whose
// 3-sample window has an invalid point or fails the discontinuity test
// is left as NaN. Output cloud has the same width/height as input.
pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const MeshBuilderParams & params);

// Run OrganizedFastMesh on an organized cloud. Coordinate system is
// not constrained — the only 3D operation is the max_edge_length check.
pcl::PolygonMesh build_fast_mesh(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const MeshBuilderParams & params);

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_
