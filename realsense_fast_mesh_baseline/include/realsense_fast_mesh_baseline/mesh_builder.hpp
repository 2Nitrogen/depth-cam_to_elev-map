// Thin ROS-free wrappers around the fast-mesh pipeline:
//   build_organized_cloud_from_depth : depth Image + CameraInfo → cloud
//   build_fast_mesh                  : OrganizedFastMesh triangulation
//   compute_face_normals             : per-triangle normal (post-mesh)
//
// Operates on organized point clouds (height > 1, width = depth-image
// columns / stride); the caller is responsible for handing the cloud in
// WITHOUT removing NaN entries — OrganizedFastMesh needs the organized
// grid structure to know which 2×2 cells to try emitting triangles for,
// and NaN corners simply suppress their incident triangles.
//
// Coordinate-system note: face normals are computed in whatever frame
// the input mesh is built in. Since the node builds the mesh on the
// already-transformed (target-frame) cloud, the resulting face normals
// are in target_frame and can be used directly for slope coloring
// (slope = arccos(|n_z|)).

#ifndef REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_
#define REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>

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

  // Spherical distance cutoff from the camera center, in meters.
  // Points with sqrt(x²+y²+z²) > max_distance_m (computed in camera
  // optical frame, where the camera is at the origin) are set to NaN
  // when the cloud is built. OrganizedFastMesh treats NaN points as
  // missing, so far points naturally disappear from the output mesh
  // without any extra plumbing. Set to 0 or negative to disable.
  float max_distance_m{3.5f};

  // OrganizedFastMesh
  //   triangle_max_edge_length: maximum 3D edge length (m) for an
  //                             accepted triangle. Auto-computed by the
  //                             node as `pixel_stride *
  //                             kBaseEdgeLengthPerStride` — NOT a yaml
  //                             knob. See the constant's docstring for
  //                             rationale.
  float             triangle_max_edge_length{kBaseEdgeLengthPerStride};
  TriangulationType triangulation_type{TriangulationType::TriangleAdaptiveCut};

  // Face-normal Laplacian smoothing on the mesh face graph (see
  // smooth_face_normals below). Operates AFTER compute_face_normals.
  //   normal_smoothing_iterations  : 0 disables; 1-2 typical.
  //   normal_smoothing_self_weight : α ∈ [0, 1]. 1 = keep self only
  //                                  (effectively K=0); 0 = pure
  //                                  neighbor average.
  std::uint32_t normal_smoothing_iterations{1u};
  float         normal_smoothing_self_weight{0.5f};
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

// Run OrganizedFastMesh on an organized cloud. Coordinate system is
// not constrained — the only 3D operation is the max_edge_length check.
pcl::PolygonMesh build_fast_mesh(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud,
  const MeshBuilderParams & params);

// Compute one unit normal per triangle in `mesh`:
//     n_f = normalize((b - a) × (c - a))
// for triangle (a, b, c) where indices reference mesh.cloud. The output
// vector has the same length and order as mesh.polygons, so caller-side
// iteration over polygons can index face_normals[i] directly.
//
// Degenerate triangles (zero-area or with NaN vertices) get a NaN
// normal; downstream consumers (e.g. mesh_marker) handle NaN gracefully.
//
// Because OrganizedFastMesh runs on the (already-transformed) target-
// frame cloud, the returned normals are in target_frame as well —
// slope angle = arccos(|n_z|) is then directly meaningful.
std::vector<Eigen::Vector3f> compute_face_normals(const pcl::PolygonMesh & mesh);

// Build a per-face 1-ring adjacency list: adjacency[i] = list of face
// indices that share an edge with face i. Computed edge-based (two
// faces are neighbors iff they share exactly the same vertex-index
// pair), so it does NOT depend on OrganizedFastMesh's internal cell
// layout. O(F) expected time using a hash map keyed on the canonical
// (min_vertex_idx, max_vertex_idx) edge.
//
// Note on edge-preserving behavior downstream: OrganizedFastMesh
// already drops triangles whose 3D edges exceed triangle_max_edge_length
// (e.g. step edges / depth discontinuities), so the resulting mesh is
// naturally cut into connected components at those boundaries. The
// adjacency graph inherits that property — neighbors only exist within
// a connected surface region — which makes Laplacian smoothing on this
// graph edge-preserving by construction.
std::vector<std::vector<int>> compute_face_adjacency(const pcl::PolygonMesh & mesh);

// Laplacian smoothing of face normals on the supplied adjacency graph.
// For each of K = `iterations` iterations, replace every face's normal
// with:
//     n_i' = normalize( α · n_i + (1-α) · mean_{j ∈ N(i)} n_j )
// where the neighbor mean ignores NaN normals. If face i itself has a
// NaN normal, it stays NaN. Returns a new vector (does not mutate the
// input — avoids in-place same-iteration cross-talk between sibling
// faces).
//
// iterations = 0 returns a copy of `face_normals` unchanged. Typical
// values: 1-2 for visible noise reduction without over-smoothing small
// features.
std::vector<Eigen::Vector3f> smooth_face_normals(
  const std::vector<Eigen::Vector3f> & face_normals,
  const std::vector<std::vector<int>> & adjacency,
  std::uint32_t iterations,
  float self_weight);

}  // namespace realsense_fast_mesh_baseline

#endif  // REALSENSE_FAST_MESH_BASELINE__MESH_BUILDER_HPP_
