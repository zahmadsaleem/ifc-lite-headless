use super::spatial::{convex_hull_2d, mesh_aabb};
use super::types::{Aabb, CoordinateSystem, ReferenceFile, Zone, ZoneVolume};
use crate::config::{ConvertConfig, Verbosity};
use crate::processor;
use ifc_lite_core::{build_entity_index, EntityDecoder, EntityScanner};
use rustc_hash::FxHashMap;
use std::path::Path;
use std::sync::Arc;

struct StoreyInfo {
    id: u32,
    global_id: String,
    name: String,
    elevation: f64,
}

/// Extract spatial structure from an IFC file and write reference.json.
pub fn extract_reference(
    input: &Path,
    output: &Path,
    _leaf_type: &str,
    volume_mode: &str,
    verbosity: Verbosity,
) -> Result<(), Box<dyn std::error::Error>> {
    let content = std::fs::read_to_string(input)?;
    let quiet = verbosity == Verbosity::Quiet;

    if !quiet {
        eprintln!("Extracting spatial structure from {}...", input.display());
    }

    let entity_index = Arc::new(build_entity_index(&content));
    let mut decoder = EntityDecoder::with_arc_index(&content, entity_index.clone());

    // Scan for spatial structure entities and relationships
    let mut scanner = EntityScanner::new(&content);
    let mut storeys: Vec<StoreyInfo> = Vec::new();
    let mut agg_relating_to_related: FxHashMap<u32, Vec<u32>> = FxHashMap::default();
    let mut containment: FxHashMap<u32, Vec<u32>> = FxHashMap::default();
    let mut buildings: Vec<(u32, String, String)> = Vec::new();
    let mut sites: Vec<(u32, String, String)> = Vec::new();
    let mut project_id: Option<u32> = None;

    while let Some((id, type_name, start, end)) = scanner.next_entity() {
        match type_name {
            "IFCBUILDINGSTOREY" => {
                if let Ok(entity) = decoder.decode_at(start, end) {
                    let global_id = entity.get_string(0).unwrap_or("").to_string();
                    let name = entity.get_string(2).unwrap_or("").to_string();
                    let elevation = entity.get_float(9).unwrap_or(0.0);
                    storeys.push(StoreyInfo {
                        id,
                        global_id,
                        name,
                        elevation,
                    });
                }
            }
            "IFCBUILDING" => {
                if let Ok(entity) = decoder.decode_at(start, end) {
                    let global_id = entity.get_string(0).unwrap_or("").to_string();
                    let name = entity.get_string(2).unwrap_or("").to_string();
                    buildings.push((id, global_id, name));
                }
            }
            "IFCSITE" => {
                if let Ok(entity) = decoder.decode_at(start, end) {
                    let global_id = entity.get_string(0).unwrap_or("").to_string();
                    let name = entity.get_string(2).unwrap_or("").to_string();
                    sites.push((id, global_id, name));
                }
            }
            "IFCPROJECT" => {
                project_id = Some(id);
            }
            "IFCRELAGGREGATES" => {
                if let Ok(entity) = decoder.decode_at(start, end) {
                    if let Some(relating) = entity.get_ref(4) {
                        if let Some(related_list) = entity.get_list(5) {
                            let related: Vec<u32> =
                                related_list.iter().filter_map(|v| v.as_entity_ref()).collect();
                            agg_relating_to_related
                                .entry(relating)
                                .or_default()
                                .extend(related);
                        }
                    }
                }
            }
            "IFCRELCONTAINEDINSPATIALSTRUCTURE" => {
                if let Ok(entity) = decoder.decode_at(start, end) {
                    if let Some(structure_id) = entity.get_ref(5) {
                        if let Some(elements_list) = entity.get_list(4) {
                            let elements: Vec<u32> =
                                elements_list.iter().filter_map(|v| v.as_entity_ref()).collect();
                            containment
                                .entry(structure_id)
                                .or_default()
                                .extend(elements);
                        }
                    }
                }
            }
            _ => {}
        }
    }

    // Extract unit scale
    let unit_scale = if let Some(pid) = project_id {
        ifc_lite_core::extract_length_unit_scale(&mut decoder, pid).unwrap_or(1.0)
    } else {
        1.0
    };

    // Apply unit scale to storey elevations
    for s in &mut storeys {
        s.elevation *= unit_scale;
    }

    // Sort storeys by elevation
    storeys.sort_by(|a, b| a.elevation.partial_cmp(&b.elevation).unwrap());

    if !quiet {
        eprintln!("Found {} storeys", storeys.len());
        for s in &storeys {
            eprintln!("  {} (elevation: {:.3}m)", s.name, s.elevation);
        }
    }

    // Infer storey heights
    let storey_heights: Vec<f64> = storeys
        .windows(2)
        .map(|w| w[1].elevation - w[0].elevation)
        .chain(std::iter::once(4.0)) // default height for last storey
        .collect();

    // Build zone polygons based on volume_mode
    let storey_zones: Vec<Zone> = match volume_mode {
        "bbox" | "contained" => {
            // For both modes, we need element geometry to compute zones
            // Process elements to get their positions
            build_zones_from_elements(
                &content,
                &storeys,
                &storey_heights,
                &containment,
                volume_mode,
                verbosity,
            )
        }
        _ => {
            // Default: use elevation-only with a large bounding polygon
            // This is the simplest mode - zones are horizontal slabs covering full XY extent
            build_zones_elevation_only(&storeys, &storey_heights)
        }
    };

    // Build the zone tree
    // If we have buildings, wrap storeys under building zones
    let zones = if buildings.len() == 1 {
        let (_, ref gid, ref name) = buildings[0];
        // Compute building volume as union of storey volumes
        let building_volume = union_zone_volumes(&storey_zones);
        vec![Zone {
            name: name.clone(),
            global_id: Some(gid.clone()),
            volume: building_volume,
            children: storey_zones,
        }]
    } else {
        // No single building wrapper; storeys at root level
        storey_zones
    };

    let reference = ReferenceFile {
        version: 1,
        source: input
            .file_name()
            .map(|f| f.to_string_lossy().to_string())
            .unwrap_or_default(),
        zones,
        coordinate_system: CoordinateSystem {
            up_axis: "Z".to_string(),
            unit_scale,
            rtc_offset: [0.0, 0.0, 0.0],
        },
    };

    let json = serde_json::to_string_pretty(&reference)?;
    std::fs::write(output, &json)?;

    if !quiet {
        eprintln!("Wrote reference to {}", output.display());
    }

    Ok(())
}

fn build_zones_elevation_only(storeys: &[StoreyInfo], heights: &[f64]) -> Vec<Zone> {
    // Use a very large polygon covering +-10km
    let large_poly = vec![
        [-10000.0, -10000.0],
        [10000.0, -10000.0],
        [10000.0, 10000.0],
        [-10000.0, 10000.0],
    ];

    storeys
        .iter()
        .zip(heights.iter())
        .map(|(s, &h)| Zone {
            name: s.name.clone(),
            global_id: Some(s.global_id.clone()),
            volume: ZoneVolume::Single {
                polygon: large_poly.clone(),
                z_min: s.elevation,
                z_max: s.elevation + h,
            },
            children: vec![],
        })
        .collect()
}

fn build_zones_from_elements(
    content: &str,
    storeys: &[StoreyInfo],
    heights: &[f64],
    containment: &FxHashMap<u32, Vec<u32>>,
    volume_mode: &str,
    verbosity: Verbosity,
) -> Vec<Zone> {
    let quiet = verbosity == Verbosity::Quiet;

    // Process the IFC to get geometry for contained elements
    let config = ConvertConfig {
        verbosity,
        no_progress: true,
        ..Default::default()
    };
    let meshes = processor::process_ifc(content, &config);

    // Build a map from step_id to mesh index for quick lookup
    let step_id_to_mesh: FxHashMap<u32, usize> = meshes
        .iter()
        .enumerate()
        .map(|(i, m)| (m.step_id, i))
        .collect();

    storeys
        .iter()
        .zip(heights.iter())
        .map(|(storey, &height)| {
            // Collect XY points of all elements contained in this storey
            let mut xy_points: Vec<[f64; 2]> = Vec::new();
            let mut z_min_actual = f64::MAX;
            let mut z_max_actual = f64::MIN;

            if let Some(element_ids) = containment.get(&storey.id) {
                for &eid in element_ids {
                    if let Some(&mesh_idx) = step_id_to_mesh.get(&eid) {
                        let aabb = mesh_aabb(&meshes[mesh_idx].mesh.positions);
                        if aabb.is_valid() {
                            // Collect corner XY points for convex hull
                            xy_points.push([aabb.min[0], aabb.min[1]]);
                            xy_points.push([aabb.max[0], aabb.min[1]]);
                            xy_points.push([aabb.max[0], aabb.max[1]]);
                            xy_points.push([aabb.min[0], aabb.max[1]]);
                            z_min_actual = z_min_actual.min(aabb.min[2]);
                            z_max_actual = z_max_actual.max(aabb.max[2]);
                        }
                    }
                }
            }

            let (polygon, z_min, z_max) = if xy_points.is_empty() {
                // No elements found; use large polygon with elevation range
                if !quiet {
                    eprintln!(
                        "  Warning: No geometry found for storey '{}', using elevation range",
                        storey.name
                    );
                }
                (
                    vec![
                        [-10000.0, -10000.0],
                        [10000.0, -10000.0],
                        [10000.0, 10000.0],
                        [-10000.0, 10000.0],
                    ],
                    storey.elevation,
                    storey.elevation + height,
                )
            } else if volume_mode == "bbox" {
                // AABB as 4-point polygon
                let mut aabb = Aabb::empty();
                for p in &xy_points {
                    aabb.expand_point([p[0], p[1], 0.0]);
                }
                (
                    vec![
                        [aabb.min[0], aabb.min[1]],
                        [aabb.max[0], aabb.min[1]],
                        [aabb.max[0], aabb.max[1]],
                        [aabb.min[0], aabb.max[1]],
                    ],
                    storey.elevation,
                    storey.elevation + height,
                )
            } else {
                // Convex hull
                let hull = convex_hull_2d(&xy_points);
                (hull, storey.elevation, storey.elevation + height)
            };

            Zone {
                name: storey.name.clone(),
                global_id: Some(storey.global_id.clone()),
                volume: ZoneVolume::Single {
                    polygon,
                    z_min,
                    z_max,
                },
                children: vec![],
            }
        })
        .collect()
}

fn union_zone_volumes(zones: &[Zone]) -> ZoneVolume {
    if zones.is_empty() {
        return ZoneVolume::Single {
            polygon: vec![],
            z_min: 0.0,
            z_max: 0.0,
        };
    }

    // Collect all polygon points and z ranges
    let mut all_xy: Vec<[f64; 2]> = Vec::new();
    let mut z_min = f64::MAX;
    let mut z_max = f64::MIN;

    for zone in zones {
        for slice in zone.volume.normalized() {
            all_xy.extend_from_slice(&slice.polygon);
            z_min = z_min.min(slice.z_min);
            z_max = z_max.max(slice.z_max);
        }
    }

    let hull = convex_hull_2d(&all_xy);
    ZoneVolume::Single {
        polygon: hull,
        z_min,
        z_max,
    }
}
