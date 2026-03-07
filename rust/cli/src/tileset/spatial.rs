use super::types::{Aabb, Slice, Zone, ZoneVolume};

/// Compute AABB centroid from mesh positions (flat f32 array: [x,y,z,x,y,z,...])
pub fn mesh_centroid(positions: &[f32]) -> [f64; 3] {
    let vertex_count = positions.len() / 3;
    if vertex_count == 0 {
        return [0.0; 3];
    }
    let mut sum = [0.0f64; 3];
    for i in 0..vertex_count {
        sum[0] += positions[i * 3] as f64;
        sum[1] += positions[i * 3 + 1] as f64;
        sum[2] += positions[i * 3 + 2] as f64;
    }
    let n = vertex_count as f64;
    [sum[0] / n, sum[1] / n, sum[2] / n]
}

/// Compute AABB from mesh positions
pub fn mesh_aabb(positions: &[f32]) -> Aabb {
    let mut aabb = Aabb::empty();
    let vertex_count = positions.len() / 3;
    for i in 0..vertex_count {
        aabb.expand_point([
            positions[i * 3] as f64,
            positions[i * 3 + 1] as f64,
            positions[i * 3 + 2] as f64,
        ]);
    }
    aabb
}

/// 2D point-in-polygon test via ray casting algorithm.
/// Casts a ray in +X direction and counts edge crossings.
pub fn point_in_polygon(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let n = polygon.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let (px, py) = (point[0], point[1]);
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (polygon[i][0], polygon[i][1]);
        let (xj, yj) = (polygon[j][0], polygon[j][1]);
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Test if a 3D point is inside a slice (Z-range + 2D polygon)
fn point_in_slice(point: [f64; 3], slice: &Slice) -> bool {
    if point[2] < slice.z_min || point[2] >= slice.z_max {
        return false;
    }
    point_in_polygon([point[0], point[1]], &slice.polygon)
}

/// Test if a 3D point is inside a zone volume (any slice matches)
pub fn point_in_zone_volume(point: [f64; 3], volume: &ZoneVolume) -> bool {
    let slices = volume.normalized();
    slices.iter().any(|s| point_in_slice(point, s))
}

/// Walk the zone tree depth-first and return the path of the deepest containing zone.
/// Returns None if the point is outside all zones.
pub fn assign_to_zone(centroid: [f64; 3], zones: &[Zone]) -> Option<String> {
    assign_to_zone_recursive(centroid, zones, "")
}

fn assign_to_zone_recursive(
    centroid: [f64; 3],
    zones: &[Zone],
    parent_path: &str,
) -> Option<String> {
    for zone in zones {
        if point_in_zone_volume(centroid, &zone.volume) {
            let current_path = if parent_path.is_empty() {
                sanitize_zone_name(&zone.name)
            } else {
                format!("{}/{}", parent_path, sanitize_zone_name(&zone.name))
            };
            // Try to find a deeper match in children
            if let Some(child_path) =
                assign_to_zone_recursive(centroid, &zone.children, &current_path)
            {
                return Some(child_path);
            }
            // No child matched; assign to this zone
            return Some(current_path);
        }
    }
    None
}

/// Sanitize a zone name for use as a directory name
pub fn sanitize_zone_name(name: &str) -> String {
    name.chars()
        .map(|c| match c {
            ' ' | '\t' => '_',
            '/' | '\\' | ':' | '*' | '?' | '"' | '<' | '>' | '|' => '_',
            c => c,
        })
        .collect()
}

/// Compute 2D convex hull of points (Andrew's monotone chain algorithm).
/// Returns polygon vertices in counter-clockwise order.
pub fn convex_hull_2d(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut pts = points.to_vec();
    pts.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap().then(a[1].partial_cmp(&b[1]).unwrap()));
    pts.dedup();

    if pts.len() < 3 {
        return pts;
    }

    let mut hull: Vec<[f64; 2]> = Vec::with_capacity(pts.len() * 2);

    // Lower hull
    for p in &pts {
        while hull.len() >= 2 && cross(&hull[hull.len() - 2], &hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(*p);
    }

    // Upper hull
    let lower_len = hull.len() + 1;
    for p in pts.iter().rev() {
        while hull.len() >= lower_len && cross(&hull[hull.len() - 2], &hull[hull.len() - 1], p) <= 0.0
        {
            hull.pop();
        }
        hull.push(*p);
    }

    hull.pop(); // Remove last point (same as first)
    hull
}

fn cross(o: &[f64; 2], a: &[f64; 2], b: &[f64; 2]) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_in_polygon_square() {
        let poly = vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];
        assert!(point_in_polygon([5.0, 5.0], &poly));
        assert!(!point_in_polygon([15.0, 5.0], &poly));
        assert!(!point_in_polygon([-1.0, 5.0], &poly));
    }

    #[test]
    fn test_point_in_polygon_triangle() {
        let poly = vec![[0.0, 0.0], [10.0, 0.0], [5.0, 10.0]];
        assert!(point_in_polygon([5.0, 3.0], &poly));
        assert!(!point_in_polygon([0.0, 10.0], &poly));
    }

    #[test]
    fn test_zone_assignment_nested() {
        let zones = vec![Zone {
            name: "Building A".to_string(),
            global_id: None,
            volume: ZoneVolume::Single {
                polygon: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]],
                z_min: -1.0,
                z_max: 10.0,
            },
            children: vec![
                Zone {
                    name: "Level 0".to_string(),
                    global_id: None,
                    volume: ZoneVolume::Single {
                        polygon: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]],
                        z_min: -1.0,
                        z_max: 3.5,
                    },
                    children: vec![],
                },
                Zone {
                    name: "Level 1".to_string(),
                    global_id: None,
                    volume: ZoneVolume::Single {
                        polygon: vec![[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]],
                        z_min: 3.5,
                        z_max: 10.0,
                    },
                    children: vec![],
                },
            ],
        }];

        // Inside Level 0
        assert_eq!(
            assign_to_zone([5.0, 5.0, 1.0], &zones),
            Some("Building_A/Level_0".to_string())
        );
        // Inside Level 1
        assert_eq!(
            assign_to_zone([5.0, 5.0, 5.0], &zones),
            Some("Building_A/Level_1".to_string())
        );
        // Outside all zones
        assert_eq!(assign_to_zone([100.0, 100.0, 1.0], &zones), None);
    }

    #[test]
    fn test_convex_hull() {
        let points = vec![
            [0.0, 0.0],
            [1.0, 0.0],
            [0.5, 0.5],
            [1.0, 1.0],
            [0.0, 1.0],
        ];
        let hull = convex_hull_2d(&points);
        assert_eq!(hull.len(), 4); // Square without interior point
    }

    #[test]
    fn test_mesh_centroid() {
        let positions = vec![0.0f32, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 10.0, 0.0, 0.0, 10.0, 0.0];
        let c = mesh_centroid(&positions);
        assert!((c[0] - 5.0).abs() < 1e-6);
        assert!((c[1] - 5.0).abs() < 1e-6);
        assert!(c[2].abs() < 1e-6);
    }
}
