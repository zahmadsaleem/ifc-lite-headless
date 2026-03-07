// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! CSG (Constructive Solid Geometry) Operations
//!
//! Fast triangle clipping and boolean operations.

use crate::error::Result;
use crate::mesh::Mesh;
use crate::triangulation::{calculate_polygon_normal, project_to_2d, triangulate_polygon};
use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;
use smallvec::SmallVec;

/// Type alias for small triangle collections (typically 1-2 triangles from clipping)
pub type TriangleVec = SmallVec<[Triangle; 4]>;

/// Plane definition for clipping
#[derive(Debug, Clone, Copy)]
pub struct Plane {
    /// Point on the plane
    pub point: Point3<f64>,
    /// Normal vector (must be normalized)
    pub normal: Vector3<f64>,
}

impl Plane {
    /// Create a new plane
    pub fn new(point: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self {
            point,
            normal: normal.normalize(),
        }
    }

    /// Calculate signed distance from point to plane
    /// Positive = in front, Negative = behind
    pub fn signed_distance(&self, point: &Point3<f64>) -> f64 {
        (point - self.point).dot(&self.normal)
    }

    /// Check if point is in front of plane
    pub fn is_front(&self, point: &Point3<f64>) -> bool {
        self.signed_distance(point) >= 0.0
    }
}

/// Triangle clipping result
#[derive(Debug, Clone)]
pub enum ClipResult {
    /// Triangle is completely in front (keep it)
    AllFront(Triangle),
    /// Triangle is completely behind (discard it)
    AllBehind,
    /// Triangle intersects plane - returns new triangles (uses SmallVec to avoid heap allocation)
    Split(TriangleVec),
}

/// Triangle definition
#[derive(Debug, Clone)]
pub struct Triangle {
    pub v0: Point3<f64>,
    pub v1: Point3<f64>,
    pub v2: Point3<f64>,
}

impl Triangle {
    /// Create a new triangle
    #[inline]
    pub fn new(v0: Point3<f64>, v1: Point3<f64>, v2: Point3<f64>) -> Self {
        Self { v0, v1, v2 }
    }

    /// Calculate triangle normal
    #[inline]
    pub fn normal(&self) -> Vector3<f64> {
        let edge1 = self.v1 - self.v0;
        let edge2 = self.v2 - self.v0;
        edge1.cross(&edge2).normalize()
    }

    /// Calculate the cross product of edges, which is twice the area vector.
    ///
    /// Returns a `Vector3<f64>` perpendicular to the triangle plane.
    /// For degenerate/collinear triangles, returns the zero vector.
    /// Use `is_degenerate()` or `try_normalize()` on the result if you need
    /// to detect and handle degenerate cases.
    #[inline]
    pub fn cross_product(&self) -> Vector3<f64> {
        let edge1 = self.v1 - self.v0;
        let edge2 = self.v2 - self.v0;
        edge1.cross(&edge2)
    }

    /// Calculate triangle area (half the magnitude of the cross product).
    #[inline]
    pub fn area(&self) -> f64 {
        self.cross_product().norm() * 0.5
    }

    /// Check if triangle is degenerate (zero area, collinear vertices).
    ///
    /// Uses `try_normalize` on the cross product with the specified epsilon.
    /// Returns `true` if the cross product cannot be normalized (i.e., degenerate).
    #[inline]
    pub fn is_degenerate(&self, epsilon: f64) -> bool {
        self.cross_product().try_normalize(epsilon).is_none()
    }
}

/// Maximum combined polygon count for CSG operations.
/// The csgrs BSP tree can infinite-recurse on certain polygon configurations
/// (coplanar/near-coplanar faces cause repeated splitting with exponential growth).
/// This limit prevents stack overflow in both native and WASM builds.
/// Set to 5000 to support walls with many openings (e.g. 7+ windows).
const MAX_CSG_POLYGONS: usize = 5000;

/// CSG Clipping Processor
pub struct ClippingProcessor {
    /// Epsilon for floating point comparisons
    pub epsilon: f64,
}

/// Create a box mesh from AABB min/max bounds
/// Returns a mesh with 12 triangles (2 per face, 6 faces)
fn aabb_to_mesh(min: Point3<f64>, max: Point3<f64>) -> Mesh {
    let mut mesh = Mesh::with_capacity(8, 36);

    // Define the 8 vertices of the box
    let v0 = Point3::new(min.x, min.y, min.z); // 0: front-bottom-left
    let v1 = Point3::new(max.x, min.y, min.z); // 1: front-bottom-right
    let v2 = Point3::new(max.x, max.y, min.z); // 2: front-top-right
    let v3 = Point3::new(min.x, max.y, min.z); // 3: front-top-left
    let v4 = Point3::new(min.x, min.y, max.z); // 4: back-bottom-left
    let v5 = Point3::new(max.x, min.y, max.z); // 5: back-bottom-right
    let v6 = Point3::new(max.x, max.y, max.z); // 6: back-top-right
    let v7 = Point3::new(min.x, max.y, max.z); // 7: back-top-left

    // Add triangles for each face (counter-clockwise winding when viewed from outside)
    // Front face (z = min.z) - normal points toward -Z
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v0, v2, v1));
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v0, v3, v2));

    // Back face (z = max.z) - normal points toward +Z
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v4, v5, v6));
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v4, v6, v7));

    // Left face (x = min.x) - normal points toward -X
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v0, v4, v7));
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v0, v7, v3));

    // Right face (x = max.x) - normal points toward +X
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v1, v2, v6));
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v1, v6, v5));

    // Bottom face (y = min.y) - normal points toward -Y
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v0, v1, v5));
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v0, v5, v4));

    // Top face (y = max.y) - normal points toward +Y
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v3, v7, v6));
    add_triangle_to_mesh(&mut mesh, &Triangle::new(v3, v6, v2));

    mesh
}

impl ClippingProcessor {
    /// Create a new clipping processor
    pub fn new() -> Self {
        Self { epsilon: 1e-6 }
    }

    /// Clip a triangle against a plane
    /// Returns triangles that are in front of the plane
    pub fn clip_triangle(&self, triangle: &Triangle, plane: &Plane) -> ClipResult {
        // Calculate signed distances for all vertices
        let d0 = plane.signed_distance(&triangle.v0);
        let d1 = plane.signed_distance(&triangle.v1);
        let d2 = plane.signed_distance(&triangle.v2);

        // Count vertices in front of plane
        let mut front_count = 0;
        if d0 >= -self.epsilon {
            front_count += 1;
        }
        if d1 >= -self.epsilon {
            front_count += 1;
        }
        if d2 >= -self.epsilon {
            front_count += 1;
        }

        match front_count {
            // All vertices behind - discard triangle
            0 => ClipResult::AllBehind,

            // All vertices in front - keep triangle
            3 => ClipResult::AllFront(triangle.clone()),

            // One vertex in front - create 1 smaller triangle
            1 => {
                let (front, back1, back2) = if d0 >= -self.epsilon {
                    (triangle.v0, triangle.v1, triangle.v2)
                } else if d1 >= -self.epsilon {
                    (triangle.v1, triangle.v2, triangle.v0)
                } else {
                    (triangle.v2, triangle.v0, triangle.v1)
                };

                // Interpolate to find intersection points
                let d_front = if d0 >= -self.epsilon {
                    d0
                } else if d1 >= -self.epsilon {
                    d1
                } else {
                    d2
                };
                let d_back1 = if d0 >= -self.epsilon {
                    d1
                } else if d1 >= -self.epsilon {
                    d2
                } else {
                    d0
                };
                let d_back2 = if d0 >= -self.epsilon {
                    d2
                } else if d1 >= -self.epsilon {
                    d0
                } else {
                    d1
                };

                let t1 = d_front / (d_front - d_back1);
                let t2 = d_front / (d_front - d_back2);

                let p1 = front + (back1 - front) * t1;
                let p2 = front + (back2 - front) * t2;

                ClipResult::Split(smallvec::smallvec![Triangle::new(front, p1, p2)])
            }

            // Two vertices in front - create 2 triangles
            2 => {
                let (front1, front2, back) = if d0 < -self.epsilon {
                    (triangle.v1, triangle.v2, triangle.v0)
                } else if d1 < -self.epsilon {
                    (triangle.v2, triangle.v0, triangle.v1)
                } else {
                    (triangle.v0, triangle.v1, triangle.v2)
                };

                // Interpolate to find intersection points
                let d_back = if d0 < -self.epsilon {
                    d0
                } else if d1 < -self.epsilon {
                    d1
                } else {
                    d2
                };
                let d_front1 = if d0 < -self.epsilon {
                    d1
                } else if d1 < -self.epsilon {
                    d2
                } else {
                    d0
                };
                let d_front2 = if d0 < -self.epsilon {
                    d2
                } else if d1 < -self.epsilon {
                    d0
                } else {
                    d1
                };

                let t1 = d_front1 / (d_front1 - d_back);
                let t2 = d_front2 / (d_front2 - d_back);

                let p1 = front1 + (back - front1) * t1;
                let p2 = front2 + (back - front2) * t2;

                ClipResult::Split(smallvec::smallvec![
                    Triangle::new(front1, front2, p1),
                    Triangle::new(front2, p2, p1),
                ])
            }

            _ => unreachable!(),
        }
    }

    /// Box subtraction - removes everything inside the box from the mesh
    /// Uses proper CSG difference operation via subtract_mesh
    pub fn subtract_box(&self, mesh: &Mesh, min: Point3<f64>, max: Point3<f64>) -> Result<Mesh> {
        // Fast path: if mesh is empty, return empty mesh
        if mesh.is_empty() {
            return Ok(Mesh::new());
        }

        // Create a box mesh from the AABB bounds
        let box_mesh = aabb_to_mesh(min, max);

        // Use the CSG difference operation (mesh - box)
        self.subtract_mesh(mesh, &box_mesh)
    }

    /// Extract opening profile from mesh (find largest face)
    /// Returns profile points as an ordered contour and the face normal
    /// Uses boundary extraction via edge counting to produce stable results
    #[allow(dead_code)]
    fn extract_opening_profile(
        &self,
        opening_mesh: &Mesh,
    ) -> Option<(Vec<Point3<f64>>, Vector3<f64>)> {
        if opening_mesh.is_empty() {
            return None;
        }

        // Group triangles by normal to find faces
        let mut face_groups: FxHashMap<u64, Vec<(Point3<f64>, Point3<f64>, Point3<f64>)>> =
            FxHashMap::default();
        let normal_epsilon = 0.01; // Tolerance for normal comparison

        for i in (0..opening_mesh.indices.len()).step_by(3) {
            let i0 = opening_mesh.indices[i] as usize;
            let i1 = opening_mesh.indices[i + 1] as usize;
            let i2 = opening_mesh.indices[i + 2] as usize;

            let v0 = Point3::new(
                opening_mesh.positions[i0 * 3] as f64,
                opening_mesh.positions[i0 * 3 + 1] as f64,
                opening_mesh.positions[i0 * 3 + 2] as f64,
            );
            let v1 = Point3::new(
                opening_mesh.positions[i1 * 3] as f64,
                opening_mesh.positions[i1 * 3 + 1] as f64,
                opening_mesh.positions[i1 * 3 + 2] as f64,
            );
            let v2 = Point3::new(
                opening_mesh.positions[i2 * 3] as f64,
                opening_mesh.positions[i2 * 3 + 1] as f64,
                opening_mesh.positions[i2 * 3 + 2] as f64,
            );

            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            // Use try_normalize to handle degenerate triangles
            let normal = match edge1.cross(&edge2).try_normalize(1e-10) {
                Some(n) => n,
                None => continue, // Skip degenerate triangles
            };

            // Quantize normal for grouping (round to nearest 0.01)
            let nx = (normal.x / normal_epsilon).round() as i32;
            let ny = (normal.y / normal_epsilon).round() as i32;
            let nz = (normal.z / normal_epsilon).round() as i32;
            let key = ((nx as u64) << 32) | ((ny as u32 as u64) << 16) | (nz as u32 as u64);

            face_groups.entry(key).or_default().push((v0, v1, v2));
        }

        // Find largest face group (most triangles = largest face)
        let largest_face = face_groups
            .iter()
            .max_by_key(|(_, triangles)| triangles.len())?;

        let triangles = largest_face.1;
        if triangles.is_empty() {
            return None;
        }

        // Build edge count map to find boundary edges
        // An edge is a boundary if it appears exactly once (not shared between triangles)
        // Use quantized vertex positions as keys
        let quantize = |p: &Point3<f64>| -> (i64, i64, i64) {
            let scale = 1e6; // Quantize to micrometer precision
            (
                (p.x * scale).round() as i64,
                (p.y * scale).round() as i64,
                (p.z * scale).round() as i64,
            )
        };

        // Edge key: ordered pair of quantized vertices (smaller first for consistency)
        let make_edge_key =
            |a: (i64, i64, i64), b: (i64, i64, i64)| -> ((i64, i64, i64), (i64, i64, i64)) {
                if a < b {
                    (a, b)
                } else {
                    (b, a)
                }
            };

        // Count edges and store original vertices
        let mut edge_count: FxHashMap<
            ((i64, i64, i64), (i64, i64, i64)),
            (usize, Point3<f64>, Point3<f64>),
        > = FxHashMap::default();

        for (v0, v1, v2) in triangles {
            let q0 = quantize(v0);
            let q1 = quantize(v1);
            let q2 = quantize(v2);

            // Three edges per triangle
            for (qa, qb, pa, pb) in [
                (q0, q1, *v0, *v1),
                (q1, q2, *v1, *v2),
                (q2, q0, *v2, *v0),
            ] {
                let key = make_edge_key(qa, qb);
                edge_count
                    .entry(key)
                    .and_modify(|(count, _, _)| *count += 1)
                    .or_insert((1, pa, pb));
            }
        }

        // Collect boundary edges (count == 1)
        let mut boundary_edges: Vec<(Point3<f64>, Point3<f64>)> = Vec::new();
        for (_, (count, pa, pb)) in &edge_count {
            if *count == 1 {
                boundary_edges.push((*pa, *pb));
            }
        }

        if boundary_edges.is_empty() {
            // No boundary found (closed surface with no edges) - fall back to using centroid
            return None;
        }

        // Build vertex adjacency map for boundary traversal
        let mut adjacency: FxHashMap<(i64, i64, i64), Vec<(i64, i64, i64, Point3<f64>)>> =
            FxHashMap::default();
        for (pa, pb) in &boundary_edges {
            let qa = quantize(pa);
            let qb = quantize(pb);
            adjacency.entry(qa).or_default().push((qb.0, qb.1, qb.2, *pb));
            adjacency.entry(qb).or_default().push((qa.0, qa.1, qa.2, *pa));
        }

        // Build ordered contour by walking the boundary
        let mut contour: Vec<Point3<f64>> = Vec::new();
        let mut visited: FxHashMap<(i64, i64, i64), bool> = FxHashMap::default();

        // Start from first boundary edge
        if let Some((start_p, _)) = boundary_edges.first() {
            let start_q = quantize(start_p);
            contour.push(*start_p);
            visited.insert(start_q, true);

            let mut current_q = start_q;

            // Walk around the boundary
            loop {
                let neighbors = match adjacency.get(&current_q) {
                    Some(n) => n,
                    None => break,
                };

                // Find unvisited neighbor
                let mut found_next = false;
                for (nqx, nqy, nqz, np) in neighbors {
                    let nq = (*nqx, *nqy, *nqz);
                    if !visited.get(&nq).unwrap_or(&false) {
                        contour.push(*np);
                        visited.insert(nq, true);
                        current_q = nq;
                        found_next = true;
                        break;
                    }
                }

                if !found_next {
                    break; // Closed loop or no more unvisited neighbors
                }
            }
        }

        if contour.len() < 3 {
            // Not enough points for a valid polygon
            return None;
        }

        // Calculate normal from the ordered contour
        let normal = calculate_polygon_normal(&contour);

        // Normalize the result
        let normalized_normal = match normal.try_normalize(1e-10) {
            Some(n) => n,
            None => return None, // Degenerate polygon
        };

        Some((contour, normalized_normal))
    }

    /// Convert our Mesh format to csgrs Mesh format
    fn mesh_to_csgrs(mesh: &Mesh) -> Result<csgrs::mesh::Mesh<()>> {
        use csgrs::mesh::{polygon::Polygon, vertex::Vertex, Mesh as CSGMesh};
        use std::sync::OnceLock;

        if mesh.is_empty() {
            return Ok(CSGMesh {
                polygons: Vec::new(),
                bounding_box: OnceLock::new(),
                metadata: None,
            });
        }

        // Validate mesh has enough indices for at least one triangle
        if mesh.indices.len() < 3 {
            return Ok(CSGMesh {
                polygons: Vec::new(),
                bounding_box: OnceLock::new(),
                metadata: None,
            });
        }

        let vertex_count = mesh.positions.len() / 3;
        let triangle_count = mesh.indices.len() / 3;

        // Pre-allocate for expected number of triangles (avoids reallocations)
        let mut polygons = Vec::with_capacity(triangle_count);

        // Process each triangle using chunks_exact to ensure bounds safety
        // (handles the case where indices.len() is not divisible by 3)
        for chunk in mesh.indices.chunks_exact(3) {
            let i0 = chunk[0] as usize;
            let i1 = chunk[1] as usize;
            let i2 = chunk[2] as usize;

            // Bounds check for vertex indices - skip invalid triangles
            if i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count {
                continue;
            }

            // Get triangle vertices
            // Note: bounds are guaranteed by the vertex_count check above
            let p0_idx = i0 * 3;
            let p1_idx = i1 * 3;
            let p2_idx = i2 * 3;

            let v0 = Point3::new(
                mesh.positions[p0_idx] as f64,
                mesh.positions[p0_idx + 1] as f64,
                mesh.positions[p0_idx + 2] as f64,
            );
            let v1 = Point3::new(
                mesh.positions[p1_idx] as f64,
                mesh.positions[p1_idx + 1] as f64,
                mesh.positions[p1_idx + 2] as f64,
            );
            let v2 = Point3::new(
                mesh.positions[p2_idx] as f64,
                mesh.positions[p2_idx + 1] as f64,
                mesh.positions[p2_idx + 2] as f64,
            );

            // Skip triangles with NaN or Infinity values
            if !v0.x.is_finite()
                || !v0.y.is_finite()
                || !v0.z.is_finite()
                || !v1.x.is_finite()
                || !v1.y.is_finite()
                || !v1.z.is_finite()
                || !v2.x.is_finite()
                || !v2.y.is_finite()
                || !v2.z.is_finite()
            {
                continue;
            }

            // Calculate face normal from triangle edges
            // Use try_normalize to handle degenerate (zero-area/collinear) triangles
            let edge1 = v1 - v0;
            let edge2 = v2 - v0;
            let face_normal = match edge1.cross(&edge2).try_normalize(1e-10) {
                Some(n) => n,
                None => continue, // Skip degenerate triangles to avoid NaN propagation
            };
            // Note: try_normalize returns a unit vector, which is always finite

            // Create csgrs vertices (use face normal for all vertices)
            let vertices = vec![
                Vertex::new(v0, face_normal),
                Vertex::new(v1, face_normal),
                Vertex::new(v2, face_normal),
            ];

            polygons.push(Polygon::new(vertices, None));
        }

        Ok(CSGMesh::from_polygons(&polygons, None))
    }

    /// Convert csgrs Mesh format back to our Mesh format
    fn csgrs_to_mesh(csg_mesh: &csgrs::mesh::Mesh<()>) -> Result<Mesh> {
        let mut mesh = Mesh::new();

        for polygon in &csg_mesh.polygons {
            let vertices = &polygon.vertices;
            if vertices.len() < 3 {
                continue;
            }

            // Extract 3D positions
            let points_3d: Vec<Point3<f64>> = vertices
                .iter()
                .map(|v| Point3::new(v.pos[0], v.pos[1], v.pos[2]))
                .collect();

            // Get the CSG polygon's intended normal (from first vertex)
            // Validate and normalize to avoid NaN propagation in project_to_2d
            let raw_normal = Vector3::new(
                vertices[0].normal[0],
                vertices[0].normal[1],
                vertices[0].normal[2],
            );

            // Try to normalize the CSG normal; if it fails (zero or NaN), compute from points
            let csg_normal = match raw_normal.try_normalize(1e-10) {
                Some(n) if n.x.is_finite() && n.y.is_finite() && n.z.is_finite() => n,
                _ => {
                    // Fall back to computing normal from polygon points
                    let computed = calculate_polygon_normal(&points_3d);
                    match computed.try_normalize(1e-10) {
                        Some(n) => n,
                        None => continue, // Skip degenerate polygon
                    }
                }
            };

            // FAST PATH: Triangle - no triangulation needed
            if points_3d.len() == 3 {
                let base_idx = mesh.vertex_count();
                for v in vertices {
                    mesh.add_vertex(v.pos, v.normal);
                }
                mesh.add_triangle(
                    base_idx as u32,
                    (base_idx + 1) as u32,
                    (base_idx + 2) as u32,
                );
                continue;
            }

            // Project 3D polygon to 2D using CSG normal (preserves winding intent)
            let (points_2d, _, _, _) = project_to_2d(&points_3d, &csg_normal);

            // Triangulate (handles convex AND concave polygons)
            let indices = match triangulate_polygon(&points_2d) {
                Ok(idx) => idx,
                Err(_) => continue, // Skip degenerate polygons
            };

            // Add vertices and create triangles (winding is correct from projection)
            let base_idx = mesh.vertex_count();
            for v in vertices {
                mesh.add_vertex(v.pos, v.normal);
            }

            for tri in indices.chunks(3) {
                if tri.len() == 3 {
                    mesh.add_triangle(
                        (base_idx + tri[0]) as u32,
                        (base_idx + tri[1]) as u32,
                        (base_idx + tri[2]) as u32,
                    );
                }
            }
        }

        Ok(mesh)
    }

    /// Check if two meshes' bounding boxes overlap
    fn bounds_overlap(host_mesh: &Mesh, opening_mesh: &Mesh) -> bool {
        let (host_min, host_max) = host_mesh.bounds();
        let (open_min, open_max) = opening_mesh.bounds();

        // Check for overlap in all three dimensions
        let overlap_x = open_min.x < host_max.x && open_max.x > host_min.x;
        let overlap_y = open_min.y < host_max.y && open_max.y > host_min.y;
        let overlap_z = open_min.z < host_max.z && open_max.z > host_min.z;

        overlap_x && overlap_y && overlap_z
    }

    /// Subtract opening mesh from host mesh using csgrs CSG boolean operations
    pub fn subtract_mesh(&self, host_mesh: &Mesh, opening_mesh: &Mesh) -> Result<Mesh> {
        use csgrs::traits::CSG;

        // Validate input meshes - early exit for empty host (no clone needed)
        if host_mesh.is_empty() {
            return Ok(Mesh::new());
        }

        if opening_mesh.is_empty() {
            return Ok(host_mesh.clone());
        }

        // Check bounds overlap - early exit if no intersection possible
        if !Self::bounds_overlap(host_mesh, opening_mesh) {
            return Ok(host_mesh.clone());
        }

        // Convert meshes to csgrs format
        let host_csg = match Self::mesh_to_csgrs(host_mesh) {
            Ok(csg) => csg,
            Err(_) => return Ok(host_mesh.clone()),
        };

        let opening_csg = match Self::mesh_to_csgrs(opening_mesh) {
            Ok(csg) => csg,
            Err(_) => return Ok(host_mesh.clone()),
        };

        // Validate CSG meshes have enough polygons for a valid operation
        // Empty or near-empty meshes can cause panics in csgrs
        if host_csg.polygons.is_empty() || opening_csg.polygons.is_empty() {
            return Ok(host_mesh.clone());
        }

        // Additional validation: check for degenerate polygons that could cause panics
        // Skip CSG if either mesh has suspicious polygon counts (too few for a solid)
        if host_csg.polygons.len() < 4 || opening_csg.polygons.len() < 4 {
            return Ok(host_mesh.clone());
        }

        // Safety: skip CSG if combined polygon count risks BSP infinite recursion
        if host_csg.polygons.len() + opening_csg.polygons.len() > MAX_CSG_POLYGONS {
            return Ok(host_mesh.clone());
        }

        // Perform CSG difference (host - opening)
        let result_csg = host_csg.difference(&opening_csg);

        // Check if result is empty
        if result_csg.polygons.is_empty() {
            return Ok(host_mesh.clone());
        }

        // Convert back to our Mesh format
        match Self::csgrs_to_mesh(&result_csg) {
            Ok(result) => Ok(Self::remove_csg_artifacts(&result, host_mesh)),
            Err(_) => Ok(host_mesh.clone())
        }
    }

    /// CSG subtract without artifact removal — for use in multi-opening sequences
    /// where cleanup should happen once at the end, not after each step.
    pub fn subtract_mesh_raw(&self, host_mesh: &Mesh, opening_mesh: &Mesh) -> Result<Mesh> {
        use csgrs::traits::CSG;

        if host_mesh.is_empty() { return Ok(Mesh::new()); }
        if opening_mesh.is_empty() { return Ok(host_mesh.clone()); }
        if !Self::bounds_overlap(host_mesh, opening_mesh) { return Ok(host_mesh.clone()); }

        let host_csg = match Self::mesh_to_csgrs(host_mesh) {
            Ok(csg) => csg, Err(_) => return Ok(host_mesh.clone()),
        };
        let opening_csg = match Self::mesh_to_csgrs(opening_mesh) {
            Ok(csg) => csg, Err(_) => return Ok(host_mesh.clone()),
        };

        if host_csg.polygons.len() < 4 || opening_csg.polygons.len() < 4 {
            return Ok(host_mesh.clone());
        }
        if host_csg.polygons.len() + opening_csg.polygons.len() > MAX_CSG_POLYGONS {
            return Ok(host_mesh.clone());
        }

        let result_csg = host_csg.difference(&opening_csg);
        if result_csg.polygons.is_empty() { return Ok(host_mesh.clone()); }

        match Self::csgrs_to_mesh(&result_csg) {
            Ok(result) => Ok(result),
            Err(_) => Ok(host_mesh.clone())
        }
    }

    /// Run artifact removal as a standalone post-process step.
    pub fn clean_artifacts(&self, mesh: &Mesh, host_mesh: &Mesh) -> Mesh {
        Self::remove_csg_artifacts(mesh, host_mesh)
    }

    /// Remove CSG spike artifacts — degenerate triangles with extreme aspect ratios.
    ///
    /// CSG can produce long thin "spike" triangles at intersection boundaries.
    /// These have a very long edge relative to their area. Normal triangles from
    /// wall/opening geometry have bounded aspect ratios.
    ///
    /// Also removes triangles that extend far beyond the host mesh bounds.
    fn remove_csg_artifacts(mesh: &Mesh, host_mesh: &Mesh) -> Mesh {
        let (host_min, host_max) = host_mesh.bounds();
        let hmin = [host_min.x as f64, host_min.y as f64, host_min.z as f64];
        let hmax = [host_max.x as f64, host_max.y as f64, host_max.z as f64];

        let dims = [hmax[0] - hmin[0], hmax[1] - hmin[1], hmax[2] - hmin[2]];
        let min_dim = dims[0].min(dims[1]).min(dims[2]).max(0.01);
        let expansion = min_dim.max(1.0);

        let mut cleaned = Mesh::with_capacity(mesh.vertex_count(), mesh.indices.len());

        for i in (0..mesh.indices.len()).step_by(3) {
            let ids = [
                mesh.indices[i] as usize,
                mesh.indices[i + 1] as usize,
                mesh.indices[i + 2] as usize,
            ];

            let verts: [(f64, f64, f64); 3] = [
                (mesh.positions[ids[0] * 3] as f64, mesh.positions[ids[0] * 3 + 1] as f64, mesh.positions[ids[0] * 3 + 2] as f64),
                (mesh.positions[ids[1] * 3] as f64, mesh.positions[ids[1] * 3 + 1] as f64, mesh.positions[ids[1] * 3 + 2] as f64),
                (mesh.positions[ids[2] * 3] as f64, mesh.positions[ids[2] * 3 + 1] as f64, mesh.positions[ids[2] * 3 + 2] as f64),
            ];

            // Check 1: vertex far outside host bounds
            let mut far = false;
            for &(px, py, pz) in &verts {
                if px < hmin[0] - expansion || px > hmax[0] + expansion
                    || py < hmin[1] - expansion || py > hmax[1] + expansion
                    || pz < hmin[2] - expansion || pz > hmax[2] + expansion
                {
                    far = true;
                    break;
                }
            }
            if far {
                continue;
            }

            // Check 2: spike detection — longest edge vs area
            // A spike has longest_edge² / area >> normal geometry
            let e0 = ((verts[1].0 - verts[0].0).powi(2) + (verts[1].1 - verts[0].1).powi(2) + (verts[1].2 - verts[0].2).powi(2)).sqrt();
            let e1 = ((verts[2].0 - verts[1].0).powi(2) + (verts[2].1 - verts[1].1).powi(2) + (verts[2].2 - verts[1].2).powi(2)).sqrt();
            let e2 = ((verts[0].0 - verts[2].0).powi(2) + (verts[0].1 - verts[2].1).powi(2) + (verts[0].2 - verts[2].2).powi(2)).sqrt();
            let longest = e0.max(e1).max(e2);

            let edge1 = Vector3::new(verts[1].0 - verts[0].0, verts[1].1 - verts[0].1, verts[1].2 - verts[0].2);
            let edge2 = Vector3::new(verts[2].0 - verts[0].0, verts[2].1 - verts[0].1, verts[2].2 - verts[0].2);
            let area = edge1.cross(&edge2).norm() * 0.5;

            // Only filter if longest edge exceeds min host dimension AND
            // the triangle is extremely thin (spike-like)
            if longest > min_dim && area > 0.0 {
                let aspect = longest * longest / area;
                // Normal triangles: aspect ~2-8. Spikes: aspect > 1000
                if aspect > 500.0 {
                    continue;
                }
            }

            let base = cleaned.vertex_count() as u32;
            for &idx in &ids {
                let p = Point3::new(
                    mesh.positions[idx * 3] as f64,
                    mesh.positions[idx * 3 + 1] as f64,
                    mesh.positions[idx * 3 + 2] as f64,
                );
                let n = Vector3::new(
                    mesh.normals[idx * 3] as f64,
                    mesh.normals[idx * 3 + 1] as f64,
                    mesh.normals[idx * 3 + 2] as f64,
                );
                cleaned.add_vertex(p, n);
            }
            cleaned.add_triangle(base, base + 1, base + 2);
        }

        // Remove disconnected small fragments via connected-component analysis.
        // CSG can produce small floating pieces (caps, slivers) disconnected from
        // the main body. Keep only the largest connected component.
        let deduped = Self::remove_small_components(&cleaned);
        Self::weld_vertices(&deduped)
    }

    /// Find connected components by shared vertices and keep the largest.
    ///
    /// Two triangles are "connected" if they share a vertex position (within epsilon).
    /// Uses union-find for O(n·α(n)) performance.
    fn remove_small_components(mesh: &Mesh) -> Mesh {
        let tri_count = mesh.triangle_count();
        if tri_count <= 1 {
            return mesh.clone();
        }

        // Quantize vertex positions to grid cells for fast neighbor lookup.
        // Use epsilon-based hashing: positions within 0.01 units share a cell.
        use rustc_hash::FxHashMap;
        let eps = 0.01_f32;
        let inv_eps = 1.0 / eps;

        // Map: quantized position key -> list of triangle indices that use it
        let mut pos_to_tris: FxHashMap<(i64, i64, i64), Vec<usize>> = FxHashMap::default();

        for tri_idx in 0..tri_count {
            let base = tri_idx * 3;
            for k in 0..3 {
                let vi = mesh.indices[base + k] as usize;
                let px = mesh.positions[vi * 3];
                let py = mesh.positions[vi * 3 + 1];
                let pz = mesh.positions[vi * 3 + 2];
                let key = (
                    (px * inv_eps).round() as i64,
                    (py * inv_eps).round() as i64,
                    (pz * inv_eps).round() as i64,
                );
                pos_to_tris.entry(key).or_default().push(tri_idx);
            }
        }

        // Union-Find
        let mut parent: Vec<usize> = (0..tri_count).collect();
        let mut rank: Vec<u8> = vec![0; tri_count];

        fn find(parent: &mut [usize], x: usize) -> usize {
            let mut r = x;
            while parent[r] != r { r = parent[r]; }
            let mut c = x;
            while c != r { let next = parent[c]; parent[c] = r; c = next; }
            r
        }

        fn union(parent: &mut [usize], rank: &mut [u8], a: usize, b: usize) {
            let ra = find(parent, a);
            let rb = find(parent, b);
            if ra == rb { return; }
            if rank[ra] < rank[rb] { parent[ra] = rb; }
            else if rank[ra] > rank[rb] { parent[rb] = ra; }
            else { parent[rb] = ra; rank[ra] += 1; }
        }

        // Union triangles that share a vertex position
        for tris in pos_to_tris.values() {
            if tris.len() > 1 {
                let first = tris[0];
                for &t in &tris[1..] {
                    union(&mut parent, &mut rank, first, t);
                }
            }
        }

        // Count component sizes
        let mut comp_size: FxHashMap<usize, usize> = FxHashMap::default();
        for i in 0..tri_count {
            *comp_size.entry(find(&mut parent, i)).or_default() += 1;
        }

        // If only one component, nothing to remove
        if comp_size.len() <= 1 {
            return mesh.clone();
        }

        // Find the largest component
        let &largest_root = comp_size.iter().max_by_key(|(_, &sz)| sz).unwrap().0;

        // Build output mesh with only triangles from the largest component
        let mut result = Mesh::with_capacity(mesh.vertex_count(), mesh.indices.len());
        for tri_idx in 0..tri_count {
            if find(&mut parent, tri_idx) != largest_root {
                continue;
            }
            let base_in = tri_idx * 3;
            let base_out = result.vertex_count() as u32;
            for k in 0..3 {
                let vi = mesh.indices[base_in + k] as usize;
                let p = Point3::new(
                    mesh.positions[vi * 3] as f64,
                    mesh.positions[vi * 3 + 1] as f64,
                    mesh.positions[vi * 3 + 2] as f64,
                );
                let n = Vector3::new(
                    mesh.normals[vi * 3] as f64,
                    mesh.normals[vi * 3 + 1] as f64,
                    mesh.normals[vi * 3 + 2] as f64,
                );
                result.add_vertex(p, n);
            }
            result.add_triangle(base_out, base_out + 1, base_out + 2);
        }

        result
    }

    /// Weld coincident vertices and recompute smooth normals.
    ///
    /// CSG output has 3 unique vertices per triangle (unshared), causing visible
    /// shading seams on flat surfaces. This pass:
    /// 1. Merges vertices at the same position
    /// 2. Groups adjacent faces into smooth groups by normal angle threshold
    /// 3. Splits vertices at sharp edges so each smooth group gets its own vertex
    /// 4. Computes area-weighted smooth normals per group
    fn weld_vertices(mesh: &Mesh) -> Mesh {
        let tri_count = mesh.triangle_count();
        if tri_count == 0 {
            return mesh.clone();
        }

        // Tight epsilon: only merge truly coincident CSG duplicate vertices.
        // Too large risks merging across thin wall features at opening corners.
        let eps = 0.001_f32;
        let inv_eps = 1.0 / eps;
        let eps_sq = eps * eps;
        // 60° threshold: CSG float drift can produce >35° variation on flat
        // surfaces. Real sharp edges (wall corners, openings) are ~90°.
        let sharp_cos = (60.0_f32).to_radians().cos(); // 0.5

        // Phase 1: Deduplicate vertices by spatial grid with neighbor search.
        // Each cell stores a list of vertex indices to handle boundary cases
        // where nearby vertices land in adjacent cells.
        use rustc_hash::FxHashMap;
        let vert_count = mesh.vertex_count();
        let mut cell_verts: FxHashMap<(i64, i64, i64), Vec<u32>> = FxHashMap::default();
        let mut welded_pos: Vec<f32> = Vec::with_capacity(vert_count * 3);
        let mut old_to_new: Vec<u32> = Vec::with_capacity(vert_count);

        for vi in 0..vert_count {
            let px = mesh.positions[vi * 3];
            let py = mesh.positions[vi * 3 + 1];
            let pz = mesh.positions[vi * 3 + 2];
            let qx = (px * inv_eps).floor() as i64;
            let qy = (py * inv_eps).floor() as i64;
            let qz = (pz * inv_eps).floor() as i64;

            // Search 3x3x3 neighborhood for any existing vertex within epsilon
            let mut found = None;
            'search: for dx in -1i64..=1 {
                for dy in -1i64..=1 {
                    for dz in -1i64..=1 {
                        if let Some(indices) = cell_verts.get(&(qx + dx, qy + dy, qz + dz)) {
                            for &idx in indices {
                                let i = idx as usize;
                                let d2 = (px - welded_pos[i * 3]).powi(2)
                                    + (py - welded_pos[i * 3 + 1]).powi(2)
                                    + (pz - welded_pos[i * 3 + 2]).powi(2);
                                if d2 <= eps_sq {
                                    found = Some(idx);
                                    break 'search;
                                }
                            }
                        }
                    }
                }
            }

            let new_idx = match found {
                Some(idx) => idx,
                None => {
                    let idx = (welded_pos.len() / 3) as u32;
                    cell_verts.entry((qx, qy, qz)).or_default().push(idx);
                    welded_pos.push(px);
                    welded_pos.push(py);
                    welded_pos.push(pz);
                    idx
                }
            };
            old_to_new.push(new_idx);
        }

        // Remap indices to welded vertex space
        let new_indices: Vec<u32> = mesh.indices.iter().map(|&i| old_to_new[i as usize]).collect();

        // Phase 2: Compute face normals (unnormalized cross product = area-weighted).
        let mut face_normals: Vec<[f32; 3]> = Vec::with_capacity(tri_count);
        let mut face_normals_unit: Vec<[f32; 3]> = Vec::with_capacity(tri_count);
        for ti in 0..tri_count {
            let i0 = new_indices[ti * 3] as usize;
            let i1 = new_indices[ti * 3 + 1] as usize;
            let i2 = new_indices[ti * 3 + 2] as usize;
            let (ax, ay, az) = (welded_pos[i0 * 3], welded_pos[i0 * 3 + 1], welded_pos[i0 * 3 + 2]);
            let (bx, by, bz) = (welded_pos[i1 * 3], welded_pos[i1 * 3 + 1], welded_pos[i1 * 3 + 2]);
            let (cx, cy, cz) = (welded_pos[i2 * 3], welded_pos[i2 * 3 + 1], welded_pos[i2 * 3 + 2]);
            let (e1x, e1y, e1z) = (bx - ax, by - ay, bz - az);
            let (e2x, e2y, e2z) = (cx - ax, cy - ay, cz - az);
            let n = [
                e1y * e2z - e1z * e2y,
                e1z * e2x - e1x * e2z,
                e1x * e2y - e1y * e2x,
            ];
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            face_normals.push(n);
            if len > 1e-12 {
                face_normals_unit.push([n[0] / len, n[1] / len, n[2] / len]);
            } else {
                face_normals_unit.push([0.0, 1.0, 0.0]);
            }
        }

        // Phase 3: Build vertex -> corner adjacency.
        // A "corner" is a specific index slot (tri_idx * 3 + k).
        let welded_vert_count = welded_pos.len() / 3;
        let mut vert_corners: Vec<Vec<usize>> = vec![Vec::new(); welded_vert_count];
        for ti in 0..tri_count {
            for k in 0..3 {
                let corner = ti * 3 + k;
                vert_corners[new_indices[corner] as usize].push(corner);
            }
        }

        // Phase 4: For each vertex, split corners into smooth groups by normal angle.
        // Each group becomes a separate output vertex with averaged normal.
        let mut corner_to_out: Vec<u32> = vec![0; tri_count * 3];
        let mut out_pos: Vec<f32> = Vec::with_capacity(welded_vert_count * 3);
        let mut out_normals: Vec<f32> = Vec::with_capacity(welded_vert_count * 3);

        for vi in 0..welded_vert_count {
            let corners = &vert_corners[vi];
            if corners.is_empty() { continue; }

            let mut grouped = vec![false; corners.len()];
            for seed_i in 0..corners.len() {
                if grouped[seed_i] { continue; }
                grouped[seed_i] = true;

                let fn0 = &face_normals[corners[seed_i] / 3];
                let mut acc = [fn0[0], fn0[1], fn0[2]];
                let mut group_corners = vec![corners[seed_i]];

                // Flood-fill: keep re-scanning ungrouped corners against the
                // running group normal. This handles transitive coplanarity
                // where A~B and B~C but A≁C due to slight CSG float drift.
                let mut changed = true;
                while changed {
                    changed = false;
                    let glen = (acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]).sqrt();
                    if glen < 1e-12 { break; }
                    let gn = [acc[0] / glen, acc[1] / glen, acc[2] / glen];
                    for other_i in 0..corners.len() {
                        if grouped[other_i] { continue; }
                        let on = &face_normals_unit[corners[other_i] / 3];
                        let dot = gn[0] * on[0] + gn[1] * on[1] + gn[2] * on[2];
                        if dot >= sharp_cos {
                            grouped[other_i] = true;
                            let fn_ = &face_normals[corners[other_i] / 3];
                            acc[0] += fn_[0]; acc[1] += fn_[1]; acc[2] += fn_[2];
                            group_corners.push(corners[other_i]);
                            changed = true;
                        }
                    }
                }

                let len = (acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]).sqrt();
                let norm = if len > 1e-12 {
                    [acc[0] / len, acc[1] / len, acc[2] / len]
                } else {
                    face_normals_unit[corners[seed_i] / 3]
                };

                let out_idx = (out_pos.len() / 3) as u32;
                out_pos.push(welded_pos[vi * 3]);
                out_pos.push(welded_pos[vi * 3 + 1]);
                out_pos.push(welded_pos[vi * 3 + 2]);
                out_normals.push(norm[0]);
                out_normals.push(norm[1]);
                out_normals.push(norm[2]);

                for &corner in &group_corners {
                    corner_to_out[corner] = out_idx;
                }
            }
        }

        // Phase 5: Build output mesh
        let out_vert_count = out_pos.len() / 3;
        let mut result = Mesh::with_capacity(out_vert_count, new_indices.len());
        for vi in 0..out_vert_count {
            result.add_vertex(
                Point3::new(out_pos[vi * 3] as f64, out_pos[vi * 3 + 1] as f64, out_pos[vi * 3 + 2] as f64),
                Vector3::new(out_normals[vi * 3] as f64, out_normals[vi * 3 + 1] as f64, out_normals[vi * 3 + 2] as f64),
            );
        }
        result.indices = corner_to_out;

        result
    }

    /// Union two meshes together using csgrs CSG boolean operations
    pub fn union_mesh(&self, mesh_a: &Mesh, mesh_b: &Mesh) -> Result<Mesh> {
        use csgrs::traits::CSG;

        // Fast paths
        if mesh_a.is_empty() {
            return Ok(mesh_b.clone());
        }
        if mesh_b.is_empty() {
            return Ok(mesh_a.clone());
        }

        // Convert meshes to csgrs format
        let csg_a = Self::mesh_to_csgrs(mesh_a)?;
        let csg_b = Self::mesh_to_csgrs(mesh_b)?;

        // Validate CSG meshes - fall back to simple merge if invalid
        if csg_a.polygons.is_empty() || csg_b.polygons.is_empty() {
            let mut merged = mesh_a.clone();
            merged.merge(mesh_b);
            return Ok(merged);
        }

        // Skip CSG if either mesh has too few polygons for a valid solid
        if csg_a.polygons.len() < 4 || csg_b.polygons.len() < 4 {
            let mut merged = mesh_a.clone();
            merged.merge(mesh_b);
            return Ok(merged);
        }

        // Safety: skip CSG if combined polygon count risks BSP infinite recursion
        if csg_a.polygons.len() + csg_b.polygons.len() > MAX_CSG_POLYGONS {
            let mut merged = mesh_a.clone();
            merged.merge(mesh_b);
            return Ok(merged);
        }

        // Perform CSG union
        let result_csg = csg_a.union(&csg_b);

        // Convert back to our Mesh format
        Self::csgrs_to_mesh(&result_csg)
    }

    /// Intersect two meshes using csgrs CSG boolean operations
    ///
    /// Returns the intersection of two meshes (the volume where both overlap).
    pub fn intersection_mesh(&self, mesh_a: &Mesh, mesh_b: &Mesh) -> Result<Mesh> {
        use csgrs::traits::CSG;

        // Fast paths: intersection with empty mesh is empty
        if mesh_a.is_empty() || mesh_b.is_empty() {
            return Ok(Mesh::new());
        }

        // Convert meshes to csgrs format
        let csg_a = Self::mesh_to_csgrs(mesh_a)?;
        let csg_b = Self::mesh_to_csgrs(mesh_b)?;

        // Validate CSG meshes - return empty if invalid
        if csg_a.polygons.is_empty() || csg_b.polygons.is_empty() {
            return Ok(Mesh::new());
        }

        // Skip CSG if either mesh has too few polygons for a valid solid
        if csg_a.polygons.len() < 4 || csg_b.polygons.len() < 4 {
            return Ok(Mesh::new());
        }

        // Safety: skip CSG if combined polygon count risks BSP infinite recursion
        if csg_a.polygons.len() + csg_b.polygons.len() > MAX_CSG_POLYGONS {
            return Ok(mesh_a.clone());
        }

        // Perform CSG intersection
        let result_csg = csg_a.intersection(&csg_b);

        // Convert back to our Mesh format
        Self::csgrs_to_mesh(&result_csg)
    }

    /// Union multiple meshes together
    ///
    /// Convenience method that sequentially unions all non-empty meshes.
    /// Skips empty meshes to avoid unnecessary CSG operations.
    pub fn union_meshes(&self, meshes: &[Mesh]) -> Result<Mesh> {
        if meshes.is_empty() {
            return Ok(Mesh::new());
        }

        if meshes.len() == 1 {
            return Ok(meshes[0].clone());
        }

        // Start with first non-empty mesh
        let mut result = Mesh::new();
        let mut found_first = false;

        for mesh in meshes {
            if mesh.is_empty() {
                continue;
            }

            if !found_first {
                result = mesh.clone();
                found_first = true;
                continue;
            }

            result = self.union_mesh(&result, mesh)?;
        }

        Ok(result)
    }

    /// Subtract multiple meshes efficiently
    ///
    /// When void count exceeds threshold, unions all voids first
    /// then performs a single subtraction. This is much more efficient
    /// for elements with many openings (e.g., floors with many penetrations).
    ///
    /// # Arguments
    /// * `host` - The host mesh to subtract from
    /// * `voids` - List of void meshes to subtract
    ///
    /// # Returns
    /// The host mesh with all voids subtracted
    pub fn subtract_meshes_batched(&self, host: &Mesh, voids: &[Mesh]) -> Result<Mesh> {
        // Filter out empty meshes
        let non_empty_voids: Vec<&Mesh> = voids.iter().filter(|m| !m.is_empty()).collect();

        if non_empty_voids.is_empty() {
            return Ok(host.clone());
        }

        if non_empty_voids.len() == 1 {
            return self.subtract_mesh(host, non_empty_voids[0]);
        }

        // Threshold for batching: if more than 10 voids, union them first
        const BATCH_THRESHOLD: usize = 10;

        if non_empty_voids.len() > BATCH_THRESHOLD {
            // Union all voids into a single mesh first
            let void_refs: Vec<Mesh> = non_empty_voids.iter().map(|m| (*m).clone()).collect();
            let combined = self.union_meshes(&void_refs)?;

            // Single subtraction
            self.subtract_mesh(host, &combined)
        } else {
            // Sequential subtraction for small counts
            let mut result = host.clone();

            for void in non_empty_voids {
                result = self.subtract_mesh(&result, void)?;
            }

            Ok(result)
        }
    }

    /// Subtract meshes with fallback on failure
    ///
    /// Attempts batched subtraction, but if it fails, returns the host mesh
    /// unchanged rather than propagating the error. This provides graceful
    /// degradation for problematic void geometries.
    pub fn subtract_meshes_with_fallback(&self, host: &Mesh, voids: &[Mesh]) -> Mesh {
        match self.subtract_meshes_batched(host, voids) {
            Ok(result) => {
                // Validate result
                if result.is_empty() || !self.validate_mesh(&result) {
                    host.clone()
                } else {
                    result
                }
            }
            Err(_) => host.clone(),
        }
    }

    /// Validate mesh for common issues
    fn validate_mesh(&self, mesh: &Mesh) -> bool {
        // Check for NaN/Inf in positions
        if mesh.positions.iter().any(|v| !v.is_finite()) {
            return false;
        }

        // Check for NaN/Inf in normals
        if mesh.normals.iter().any(|v| !v.is_finite()) {
            return false;
        }

        // Check for valid triangle indices
        let vertex_count = mesh.vertex_count();
        for idx in &mesh.indices {
            if *idx as usize >= vertex_count {
                return false;
            }
        }

        true
    }

    /// Clip mesh using bounding box (6 planes) - DEPRECATED: use subtract_box() instead
    /// Subtracts everything inside the box from the mesh
    #[deprecated(note = "Use subtract_box() for better performance")]
    pub fn clip_mesh_with_box(
        &self,
        mesh: &Mesh,
        min: Point3<f64>,
        max: Point3<f64>,
    ) -> Result<Mesh> {
        self.subtract_box(mesh, min, max)
    }

    /// Clip an entire mesh against a plane
    pub fn clip_mesh(&self, mesh: &Mesh, plane: &Plane) -> Result<Mesh> {
        let mut result = Mesh::new();

        // Process each triangle
        for i in (0..mesh.indices.len()).step_by(3) {
            let i0 = mesh.indices[i] as usize;
            let i1 = mesh.indices[i + 1] as usize;
            let i2 = mesh.indices[i + 2] as usize;

            // Get triangle vertices
            let v0 = Point3::new(
                mesh.positions[i0 * 3] as f64,
                mesh.positions[i0 * 3 + 1] as f64,
                mesh.positions[i0 * 3 + 2] as f64,
            );
            let v1 = Point3::new(
                mesh.positions[i1 * 3] as f64,
                mesh.positions[i1 * 3 + 1] as f64,
                mesh.positions[i1 * 3 + 2] as f64,
            );
            let v2 = Point3::new(
                mesh.positions[i2 * 3] as f64,
                mesh.positions[i2 * 3 + 1] as f64,
                mesh.positions[i2 * 3 + 2] as f64,
            );

            let triangle = Triangle::new(v0, v1, v2);

            // Clip triangle
            match self.clip_triangle(&triangle, plane) {
                ClipResult::AllFront(tri) => {
                    // Keep original triangle
                    add_triangle_to_mesh(&mut result, &tri);
                }
                ClipResult::AllBehind => {
                    // Discard triangle
                }
                ClipResult::Split(triangles) => {
                    // Add clipped triangles
                    for tri in triangles {
                        add_triangle_to_mesh(&mut result, &tri);
                    }
                }
            }
        }

        Ok(result)
    }
}

impl Default for ClippingProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// Add a triangle to a mesh
fn add_triangle_to_mesh(mesh: &mut Mesh, triangle: &Triangle) {
    let base_idx = mesh.vertex_count() as u32;

    // Calculate normal
    let normal = triangle.normal();

    // Add vertices
    mesh.add_vertex(triangle.v0, normal);
    mesh.add_vertex(triangle.v1, normal);
    mesh.add_vertex(triangle.v2, normal);

    // Add triangle
    mesh.add_triangle(base_idx, base_idx + 1, base_idx + 2);
}

/// Calculate smooth normals for a mesh
#[inline]
pub fn calculate_normals(mesh: &mut Mesh) {
    let vertex_count = mesh.vertex_count();
    if vertex_count == 0 {
        return;
    }

    let positions_len = mesh.positions.len();

    // Initialize normals to zero
    let mut normals = vec![Vector3::zeros(); vertex_count];

    // Accumulate face normals
    for i in (0..mesh.indices.len()).step_by(3) {
        // Bounds check for indices array
        if i + 2 >= mesh.indices.len() {
            break;
        }

        let i0 = mesh.indices[i] as usize;
        let i1 = mesh.indices[i + 1] as usize;
        let i2 = mesh.indices[i + 2] as usize;

        // Bounds check for vertex indices - skip invalid triangles
        if i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count {
            continue;
        }
        if i0 * 3 + 2 >= positions_len || i1 * 3 + 2 >= positions_len || i2 * 3 + 2 >= positions_len
        {
            continue;
        }

        // Get triangle vertices
        let v0 = Point3::new(
            mesh.positions[i0 * 3] as f64,
            mesh.positions[i0 * 3 + 1] as f64,
            mesh.positions[i0 * 3 + 2] as f64,
        );
        let v1 = Point3::new(
            mesh.positions[i1 * 3] as f64,
            mesh.positions[i1 * 3 + 1] as f64,
            mesh.positions[i1 * 3 + 2] as f64,
        );
        let v2 = Point3::new(
            mesh.positions[i2 * 3] as f64,
            mesh.positions[i2 * 3 + 1] as f64,
            mesh.positions[i2 * 3 + 2] as f64,
        );

        // Calculate face normal
        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let normal = edge1.cross(&edge2);

        // Accumulate normal for each vertex
        normals[i0] += normal;
        normals[i1] += normal;
        normals[i2] += normal;
    }

    // Normalize and write back
    mesh.normals.clear();
    mesh.normals.reserve(vertex_count * 3);

    for normal in normals {
        let normalized = normal.try_normalize(1e-6).unwrap_or_else(|| Vector3::new(0.0, 0.0, 1.0));
        mesh.normals.push(normalized.x as f32);
        mesh.normals.push(normalized.y as f32);
        mesh.normals.push(normalized.z as f32);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plane_signed_distance() {
        let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        assert_eq!(plane.signed_distance(&Point3::new(0.0, 0.0, 5.0)), 5.0);
        assert_eq!(plane.signed_distance(&Point3::new(0.0, 0.0, -5.0)), -5.0);
        assert_eq!(plane.signed_distance(&Point3::new(5.0, 5.0, 0.0)), 0.0);
    }

    #[test]
    fn test_clip_triangle_all_front() {
        let processor = ClippingProcessor::new();
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(0.5, 1.0, 1.0),
        );
        let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        match processor.clip_triangle(&triangle, &plane) {
            ClipResult::AllFront(_) => {}
            _ => panic!("Expected AllFront"),
        }
    }

    #[test]
    fn test_clip_triangle_all_behind() {
        let processor = ClippingProcessor::new();
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.5, 1.0, -1.0),
        );
        let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        match processor.clip_triangle(&triangle, &plane) {
            ClipResult::AllBehind => {}
            _ => panic!("Expected AllBehind"),
        }
    }

    #[test]
    fn test_clip_triangle_split_one_front() {
        let processor = ClippingProcessor::new();
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 1.0),  // Front
            Point3::new(1.0, 0.0, -1.0), // Behind
            Point3::new(0.5, 1.0, -1.0), // Behind
        );
        let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        match processor.clip_triangle(&triangle, &plane) {
            ClipResult::Split(triangles) => {
                assert_eq!(triangles.len(), 1);
            }
            _ => panic!("Expected Split"),
        }
    }

    #[test]
    fn test_clip_triangle_split_two_front() {
        let processor = ClippingProcessor::new();
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 1.0),  // Front
            Point3::new(1.0, 0.0, 1.0),  // Front
            Point3::new(0.5, 1.0, -1.0), // Behind
        );
        let plane = Plane::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));

        match processor.clip_triangle(&triangle, &plane) {
            ClipResult::Split(triangles) => {
                assert_eq!(triangles.len(), 2);
            }
            _ => panic!("Expected Split with 2 triangles"),
        }
    }

    #[test]
    fn test_triangle_normal() {
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );

        let normal = triangle.normal();
        assert!((normal.z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_triangle_area() {
        let triangle = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );

        let area = triangle.area();
        assert!((area - 0.5).abs() < 1e-6);
    }
}
