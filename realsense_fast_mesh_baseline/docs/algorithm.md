# Fast Mesh Baseline — Algorithm Reference

This document describes the current per-callback data flow and
mathematical formulation of `realsense_fast_mesh_baseline`, as
implemented in
[`src/fast_mesh_node.cpp`](../src/fast_mesh_node.cpp),
[`src/mesh_builder.cpp`](../src/mesh_builder.cpp), and
[`src/mesh_marker.cpp`](../src/mesh_marker.cpp).

It is intentionally a snapshot of what runs *today*, not a design
proposal. Use it as a reference when annotating improvements
(multi-frame fusion, depth filtering, grouping, etc.).

---

## 1. Notation

| Symbol | Meaning |
|---|---|
| $I_d[v, u]$ | Depth image pixel value at row $v$, column $u$. 16UC1 → mm; 32FC1 → m. |
| $W, H$ | Original depth image width and height (px). |
| $s$ | `pixel_stride` (integer, ≥1). Subsamples the image grid. |
| $W', H'$ | Cloud width/height after stride: $W' = \lfloor W/s \rfloor$, $H' = \lfloor H/s \rfloor$. |
| $(u_o, v_o)$ | ORIGINAL pixel coordinates of a sampled pixel: $u_o = u'·s$, $v_o = v'·s$. |
| $f_x, f_y, c_x, c_y$ | Pinhole intrinsics from `CameraInfo.k` (apply to ORIGINAL coords). |
| $p_c = (x, y, z)$ | A 3D point in the camera optical frame. |
| $p_t$ | The same point in `target_frame` (gravity-aligned). |
| $n_c, n_t$ | Per-pixel unit normal in camera / target frame. |
| $R \in SO(3)$ | Rotation part of the TF $\text{target} \leftarrow \text{cam}$. |
| $T_{a \leftarrow b}$ | Full rigid TF: maps coordinates in frame $b$ to frame $a$. |
| $\hat z = (0, 0, 1)$ | World-up direction (target frame is gravity-aligned). |
| $R_{\max}$ | `max_distance_m` — spherical cutoff from camera origin. |
| $\theta$ | Slope angle: $\theta = \arccos(|n_z|)$ in target frame. |
| $\mathcal{M}$ | Output mesh, a set of triangles $\{(a_k, b_k, c_k)\}$ with vertex positions. |

Frames used:

- `cam` ≡ depth image's optical frame (e.g. `camera_depth_optical_frame`). Camera origin is at $(0,0,0)$ in this frame.
- `target_frame` (default `odom`): gravity-aligned, where the mesh is published.

---

## 2. Per-callback Pipeline

The `on_depth_image` callback ([`fast_mesh_node.cpp`](../src/fast_mesh_node.cpp)) executes the following stages in order on every depth Image message. `CameraInfo` is cached on its own callback so the depth callback always uses the latest available intrinsics.

```
sensor_msgs/Image (depth)            sensor_msgs/CameraInfo (cached)
        │                                       │
        ▼                                       ▼
[2.1]  Build organized cloud from depth + intrinsics
       — pinhole back-projection on stride-sampled pixels
       — spherical R-cutoff dropped to NaN
   │
   ▼
[2.2]  Per-pixel cross-product normal estimation (camera frame)
   │
   ▼
[2.3]  TF lookup  T_{target ← cam}
   │
   ▼
[2.4]  pcl::transformPointCloud (cloud) + R · n  (normals)
   │
   ▼
[2.5]  pcl::OrganizedFastMesh → triangle list
   │
   ▼
[2.6]  Serialize to MarkerArray (POINTS, TRIANGLE_LIST, LINE_LIST)
       publish on /local_fast_mesh/mesh
```

There is **no state carried between frames** — each callback is a pure function of `(depth_image, camera_info, TF buffer)`.

---

## 2.1. Build organized cloud — `build_organized_cloud_from_depth`

Reference: [`src/mesh_builder.cpp`](../src/mesh_builder.cpp), `build_organized_cloud_from_depth`.

### Stride subsampling

Cloud dimensions:

$$
W' = \left\lfloor \frac{W}{s} \right\rfloor, \qquad H' = \left\lfloor \frac{H}{s} \right\rfloor
$$

Output point at cloud index $(v', u')$ samples depth image pixel at:

$$
u_o = u' \cdot s, \quad v_o = v' \cdot s, \quad d_\text{px} = I_d[v_o, u_o]
$$

### Depth-encoding-aware unit conversion

$$
d \;=\;
\begin{cases}
d_\text{px} \cdot 10^{-3} & \text{if encoding } \in \{\texttt{16UC1}, \texttt{mono16}\}\;(\text{mm}) \\
d_\text{px}                & \text{if encoding} = \texttt{32FC1}\;(\text{m})
\end{cases}
$$

A point is marked invalid when $d_\text{px} = 0$ (for 16UC1) or $d$ is not finite / non-positive (for 32FC1).

### Pinhole back-projection (when $d$ is valid)

Intrinsics from `CameraInfo.k` are applied to the **ORIGINAL** pixel coords $(u_o, v_o)$ — never the downsampled index $(u', v')$ — so the back-projection is geometrically correct regardless of stride:

$$
\begin{aligned}
x &= \frac{(u_o - c_x)\,d}{f_x} \\
y &= \frac{(v_o - c_y)\,d}{f_y} \\
z &= d
\end{aligned}
$$

### Spherical range cutoff

Computed in the camera optical frame, where the sensor is at the origin:

$$
\|p_c\|^2 = x^2 + y^2 + z^2
$$

A point is set to NaN whenever:

$$
\|p_c\|^2 > R_{\max}^2 \qquad (R_{\max} = \texttt{max\_distance\_m})
$$

The squared form avoids a `sqrt` per pixel. When $R_{\max} \le 0$ the threshold becomes $+\infty$ and no point is ever culled.

### Invalid-point handling

Invalid pixels (zero depth, non-finite, or outside the sphere) set the point to $(\text{NaN}, \text{NaN}, \text{NaN})$. The `data[3]` SSE alignment field is always set to 1.0 — some PCL kernels read it and undefined values can trigger downstream Eigen assertions.

The output cloud retains its organized layout (`width=W'`, `height=H'`, `is_dense=false`).

---

## 2.2. Per-pixel normal estimation — `estimate_normals`

Reference: [`src/mesh_builder.cpp`](../src/mesh_builder.cpp), `estimate_normals`.

The PCL `IntegralImageNormalEstimation` triggers an Eigen assertion on the PCL 1.12 / Eigen 3.4 / 848×480 combo we run, regardless of estimation method. The fast mesh baseline therefore computes per-pixel normals manually via a cross product on the organized cloud:

Let $\delta = \texttt{normal\_smoothing\_size}$ (in cloud-grid pixels, ≥1). For each cloud index $(v', u')$ with $u' + \delta < W'$ and $v' + \delta < H'$:

$$
p_c   = p_{(v',\, u')}, \quad p_r = p_{(v',\, u' + \delta)}, \quad p_d = p_{(v' + \delta,\, u')}
$$

The pixel is rejected (normal = NaN) if any of $p_c, p_r, p_d$ has non-finite or non-positive $z$, or if the depth-discontinuity test fires (see below). Otherwise:

$$
n = (p_r - p_c) \times (p_d - p_c), \qquad n_{(v', u')} = \frac{n}{\|n\|}
$$

`curvature` is set to 0 — unused downstream.

### Depth-discontinuity rejection

Let $\alpha = \texttt{max\_depth\_change\_factor}$. When $\alpha > 0$, the pixel is rejected if either:

$$
|p_{r,z} - p_{c,z}| > \alpha \cdot p_{c,z} \quad \text{or} \quad |p_{d,z} - p_{c,z}| > \alpha \cdot p_{c,z}
$$

This is the same convention `IntegralImageNormalEstimation` would have used. Step edges thus get NaN normals on the upper-edge pixel.

### Notes on normal sign convention

The cross product $(p_r - p_c) \times (p_d - p_c)$ yields a normal along the camera optical $+z$ axis (INTO the scene) for surfaces facing the camera, which is the **opposite** of PCL's convention (normals toward the camera). Sign matters for vector reasoning but is irrelevant for slope coloring since slope = $\arccos(|n_z|)$ is sign-agnostic.

---

## 2.3. TF lookup

Reference: [`fast_mesh_node.cpp`](../src/fast_mesh_node.cpp), the `tf_buffer_->lookupTransform` call.

Lookup time policy is parameterized by `use_latest_tf`:

$$
t_\text{lookup} =
\begin{cases}
\texttt{TimePointZero} \;(\text{latest available}) & \text{if } \texttt{use\_latest\_tf} = \text{true} \\
\texttt{msg->header.stamp} & \text{otherwise}
\end{cases}
$$

We query:

$$
T_{\text{target} \leftarrow \text{cam}} = \texttt{tf\_buffer}.\text{lookupTransform}\bigl(\texttt{target\_frame},\; \texttt{msg->header.frame\_id},\; t_\text{lookup}\bigr)
$$

This transform encodes the static `camera_link → camera_*_optical_frame` chain (from the wrapper) AND the dynamic `odom → base_link → camera_link` chain (state estimator + camera mount static TF). Roll/pitch coming from the IMU are absorbed here — no per-pixel orientation adjustment is needed downstream.

**Failure handling**: any TF exception → log warn + early return for this frame (no Marker published).

**Out-stamp** for the published Marker:

$$
t_\text{out} =
\begin{cases}
\text{node clock now}() & \text{if } \texttt{use\_latest\_tf} = \text{true} \\
\texttt{msg->header.stamp} & \text{otherwise}
\end{cases}
$$

---

## 2.4. Transform cloud + rotate normals

Reference: [`fast_mesh_node.cpp`](../src/fast_mesh_node.cpp), `pcl::transformPointCloud` block.

Convert the TF result to an Eigen transform (via Matrix4f to avoid an Isometry → Affine cast chain that has caused trouble in this PCL/Eigen combo):

$$
T_4 \;=\; \texttt{tf2::transformToEigen}(T_{\text{target} \leftarrow \text{cam}}).\text{matrix}().\text{cast}<\text{float}>()
$$

Apply to every cloud point:

$$
p_t[i] = T_4 \cdot \tilde p_c[i] \qquad (\text{homogeneous})
$$

The cloud's organized structure (`width`, `height`) is preserved.

Normals are rotated by the rotation block only (no translation):

$$
R = T_4[1{:}3,\, 1{:}3], \qquad n_t = R \, n_c
$$

The pre-existing magnitude of $n_c$ (unit) is preserved by $R$ since $R \in SO(3)$. NaN normals stay NaN.

---

## 2.5. Triangulation — `pcl::OrganizedFastMesh`

Reference: [`src/mesh_builder.cpp`](../src/mesh_builder.cpp), `build_fast_mesh`.

`pcl::OrganizedFastMesh<pcl::PointXYZ>` is configured with two knobs:

- `triangulation_type`: `TRIANGLE_RIGHT_CUT` / `TRIANGLE_LEFT_CUT` / `TRIANGLE_ADAPTIVE_CUT`. Adaptive (the default) picks the diagonal that yields the smaller in-quad edge length.
- `triangle_max_edge_length` $= s \cdot k_\text{base}$, where $k_\text{base} = $ `kBaseEdgeLengthPerStride` $= 0.10\,\text{m}$ (constexpr in [`mesh_builder.hpp`](../include/realsense_fast_mesh_baseline/mesh_builder.hpp)). Auto-scaled by stride so the depth-jump rejection threshold tracks vertex spacing.

For each 2×2 cell of cloud indices, the algorithm:

1. Picks a diagonal (right-cut, left-cut, or adaptive).
2. Emits 2 triangles if all 4 corners are finite AND every emitted edge has 3D length $\le$ `triangle_max_edge_length`.
3. Skips the cell otherwise (one NaN corner → no triangles; long edge → no triangles).

Output `pcl::PolygonMesh`:

- `mesh.cloud` (pcl::PCLPointCloud2) — vertices in `target_frame` coordinates (since we transformed before triangulating). Same number of vertices as the cloud; NaN ones are simply unreferenced.
- `mesh.polygons` (`std::vector<pcl::Vertices>`) — each entry has exactly 3 vertex indices.

---

## 2.6. MarkerArray output — `mesh_to_marker_array`

Reference: [`src/mesh_marker.cpp`](../src/mesh_marker.cpp).

The output `visualization_msgs/MarkerArray` always contains three markers (action=ADD, fixed ids, RViz overwrites previous publish):

### Common: slope-color ramp

For a vertex with normal $n_t$ in target frame:

$$
\theta = \arccos\!\big(\min(1,\, |n_{t,z}|)\big)
$$

$$
t = \mathrm{clamp}\!\left(\frac{\theta - \texttt{color\_min\_slope\_rad}}{\texttt{color\_max\_slope\_rad} - \texttt{color\_min\_slope\_rad}},\; 0,\; 1\right)
$$

$$
(r, g, b) = (t,\; 1 - t,\; 0)
$$

NaN normals → $\theta = 0$ → green (flat) by default.

### id=0 POINTS — opaque vertex dots

One point per finite cloud vertex (skips NaN). Properties:

- `type = POINTS`
- `scale = (point\_size\_m, point\_size\_m)` — width, height in meters (world-scale, RViz billboarding).
- Per-vertex color: $(r, g, b, 1.0)$ from the slope ramp.

### id=1 TRIANGLE_LIST — faint slope-colored faces

Three vertices per triangle in `mesh.polygons`, with `colors` interleaved per vertex:

- `type = TRIANGLE_LIST`
- `scale = (1, 1, 1)` (ignored but must be non-zero)
- Per-vertex color: $(r, g, b, \texttt{face\_alpha})$.

RViz alpha-blends across the triangle face.

### id=2 LINE_LIST — light-gray wireframe edges

Per triangle $(a, b, c)$, emit three line segments $(a, b), (b, c), (c, a)$ as pairs of consecutive points in `marker.points`. Shared edges between adjacent triangles get drawn twice (acceptable for visualization).

- `type = LINE_LIST`
- `scale.x = edge_width_m` — line width in meters.
- `color = (0.7, 0.7, 0.7, \texttt{edge\_alpha})` (per-marker, not per-vertex).

---

## 3. Parameter glossary

All defaults in [`config/params.yaml`](../config/params.yaml).

### Topics & frames
| Param | Default | Role |
|---|---|---|
| `depth_image_topic` | `/camera/camera/depth/image_rect_raw` | Input depth Image |
| `camera_info_topic` | `/camera/camera/depth/camera_info` | Camera intrinsics |
| `mesh_marker_topic` | `/local_fast_mesh/mesh` | Output MarkerArray |
| `target_frame` | `odom` | Gravity-aligned frame the mesh is published in |

### Timing / TF
| Param | Default | Role |
|---|---|---|
| `tf_timeout_sec` | 0.1 | Per-lookup TF wait timeout |
| `use_latest_tf` | true | Latest TF vs. cloud-stamp TF (see §2.3) |
| `cloud_queue_size` | 5 | Subscription QoS depth |

### Cloud reconstruction
| Param | Default | Role |
|---|---|---|
| `pixel_stride` | configurable | Take every Nth pixel along both axes (see §2.1) |
| `max_distance_m` | 3.5 | Spherical cutoff radius $R_{\max}$ from camera (see §2.1) |

### Normal estimation
| Param | Default | Role |
|---|---|---|
| `normal_smoothing_size` | 20 | Cross-product sample step $\delta$ in cloud-grid pixels (see §2.2) |
| `max_depth_change_factor` | 0.02 | Relative depth-jump threshold $\alpha$ for discontinuity rejection |

### Triangulation
| Param | Default | Role |
|---|---|---|
| `triangulation_type` | `TRIANGLE_ADAPTIVE_CUT` | OrganizedFastMesh diagonal choice |
| **(not yaml)** `kBaseEdgeLengthPerStride` | 0.10 m | Constexpr base, multiplied by `pixel_stride` to get the actual edge-length cap |

### Visualization
| Param | Default | Role |
|---|---|---|
| `color_min_slope_rad` | 0.0 | Slope → 0 in color ramp (green) |
| `color_max_slope_rad` | 0.785 (≈45°) | Slope → 1 in color ramp (red) |
| `point_size_m` | 0.01 | POINTS marker size (m) |
| `face_alpha` | 0.3 | TRIANGLE_LIST face transparency |
| `edge_width_m` | 0.002 | LINE_LIST line width (m) |
| `edge_alpha` | 0.5 | LINE_LIST transparency |

---

## 4. Implementation notes (design choices)

These are decisions or simplifications baked into the current code. They're called out here so that "what's missing" is visible when considering improvements such as multi-frame fusion.

### 4.1. Single-frame, stateless

Each callback is a pure function of `(depth_image, camera_info, TF buffer)`. No deque of recent frames, no accumulated per-pixel mean/variance, no temporal smoothing. A vertex that flickered NaN ↔ valid across two frames just produces flickering mesh — there is no "stable estimate" to fall back to.

This is the most fundamental limitation against which multi-frame fusion has to be measured.

### 4.2. No depth uncertainty propagated

Per-pixel depth noise $\sigma_z \propto z^2$ for RGB-D, but the current pipeline carries no variance / confidence per vertex. Far points have the same visual weight as near points (modulo the $R_{\max}$ hard cutoff). Per-vertex variance would let multi-frame fusion weight observations principally.

### 4.3. Manual cross-product normals

The PCL `IntegralImageNormalEstimation` would give $O(N)$ smoothed normals via integral images, but it aborts on Eigen assertions in this PCL/Eigen combo. The manual cross-product is also $O(N)$ but uses a fixed 3-sample window — it has none of the integral-image smoothing benefits and inherits more per-pixel sensor noise. A multi-frame pass that averages normals across observations could partially recover this loss.

### 4.4. Normal computed BEFORE transform

`estimate_normals` runs on the camera-frame cloud. After rotation, normals are rotated by the same $R$ as the cloud. This is mathematically equivalent to computing normals in the target frame (the cross product is rotation-equivariant), but order matters once a depth-discontinuity test that *uses z as the depth axis* is introduced — `max_depth_change_factor` only makes sense in the optical frame.

### 4.5. Spherical cutoff is sensor-centered, not robot-centered

$R_{\max}$ is applied in the camera frame, so the kept region is a ball around the camera, not the robot's `track_point_frame`. For modest mount translations this is indistinguishable; for tall masts or arm-mounted cameras the kept ball is offset.

### 4.6. No multi-pixel point per vertex

Each cloud index $(v', u')$ contributes at most one vertex. With `pixel_stride > 1` we throw away the in-between depth samples entirely — there's no within-stride averaging that would reduce per-vertex noise.

### 4.7. Triangle-edge cap is uniform, not depth-aware

`triangle_max_edge_length = pixel_stride * 0.10 m` is a single scalar. At the working distance of the camera, 2 adjacent vertices on a flat surface have 3D spacing $\approx d \cdot s / f$, which grows linearly with depth. The 0.10 m cap is thus *over*-permissive close to the camera (lets a lot of garbage through near depth-jumps) and tight far away. A depth-aware cap (e.g. $\propto d \cdot s / f$) would track the expected vertex spacing more cleanly.

### 4.8. Output is visualization-only

The MarkerArray output is consumed by RViz. There is no structured downstream API — no `geometry_msgs/Mesh`, no `nav_msgs/OccupancyGrid`, no normal cloud topic. Downstream consumers (foothold scoring, collision checks, …) would need a dedicated output channel.

### 4.9. Per-vertex color is purely slope-based

The slope ramp is monotone in $|n_z|$. There is no encoding of confidence, frame count, or temporal stability in the color or alpha. Visually, a freshly noisy vertex and a long-stable vertex look identical.

### 4.10. LINE_LIST duplicates shared edges

Adjacent triangles share an edge, and the wireframe emits both copies. For the current vertex counts (a few thousand) this is harmless, but a multi-frame pipeline that accumulates many surfaces would benefit from edge dedup.

### 4.11. TF lookup is per-frame, not interpolated

With `use_latest_tf=true`, the cloud transform uses `TimePointZero` (latest available). For a static state estimator + stationary scene this is fine; for any time-varying state estimator (Madgwick is one — IMU output changes per sample), the depth image's actual capture time and the TF stamp may differ by a few ms. Multi-frame fusion that wanted to register frames precisely would need to switch to exact-stamp lookups (`use_latest_tf=false`) and accept that TF buffer warmup affects the first ~0.5 s.

---

## 5. Code reference index

Quick map from formula → code location.

| Stage | File | Key function / line |
|---|---|---|
| Build organized cloud | [`src/mesh_builder.cpp`](../src/mesh_builder.cpp) | `build_organized_cloud_from_depth` |
| Pinhole back-projection + R-cutoff | [`src/mesh_builder.cpp`](../src/mesh_builder.cpp) | same function, inner loops |
| Per-pixel cross-product normals | [`src/mesh_builder.cpp`](../src/mesh_builder.cpp) | `estimate_normals` |
| TF lookup + use_latest_tf policy | [`src/fast_mesh_node.cpp`](../src/fast_mesh_node.cpp) | `on_depth_image`, top half |
| Cloud transform + normal rotation | [`src/fast_mesh_node.cpp`](../src/fast_mesh_node.cpp) | `on_depth_image`, middle |
| OrganizedFastMesh wrapper | [`src/mesh_builder.cpp`](../src/mesh_builder.cpp) | `build_fast_mesh` |
| Auto-scaled edge length constant | [`include/realsense_fast_mesh_baseline/mesh_builder.hpp`](../include/realsense_fast_mesh_baseline/mesh_builder.hpp) | `kBaseEdgeLengthPerStride` |
| MarkerArray POINTS / TRIANGLE_LIST / LINE_LIST | [`src/mesh_marker.cpp`](../src/mesh_marker.cpp) | `mesh_to_marker_array` |
| Slope→color ramp | [`src/mesh_marker.cpp`](../src/mesh_marker.cpp) | `slope_color`, `slope_to_ramp_t` |
