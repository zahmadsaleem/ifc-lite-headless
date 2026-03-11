use super::spatial::{assign_to_zone, mesh_aabb, mesh_centroid, sanitize_zone_name};
use super::types::*;
use crate::config::{ConvertConfig, Verbosity};
use crate::glb;
use crate::processor::{self, GuidMesh};
use rustc_hash::FxHashMap;
use std::path::{Path, PathBuf};

/// Generate a 3D Tiles 1.1 tileset from one or more IFC models against a reference.
pub fn generate_tileset(
    reference_path: &Path,
    models: &[(PathBuf, String)], // (ifc_path, model_name)
    output_dir: &Path,
    config: &ConvertConfig,
) -> Result<(), Box<dyn std::error::Error>> {
    let quiet = config.verbosity == Verbosity::Quiet;

    // Load reference
    let ref_content = std::fs::read_to_string(reference_path)?;
    let reference: ReferenceFile = serde_json::from_str(&ref_content)?;

    if !quiet {
        eprintln!("Loaded reference: {} zones", count_zones(&reference.zones));
    }

    std::fs::create_dir_all(output_dir)?;

    let mut model_tiles: Vec<TileNode> = Vec::new();
    let mut root_aabb = Aabb::empty();

    for (ifc_path, model_name) in models {
        if !quiet {
            eprintln!("\nProcessing model '{}' from {}...", model_name, ifc_path.display());
        }

        let model_dir = output_dir.join(model_name);
        std::fs::create_dir_all(&model_dir)?;

        let raw = std::fs::read(ifc_path)?;
        let content = String::from_utf8_lossy(&raw).into_owned();
        let meshes = processor::process_ifc(&content, config);

        if !quiet {
            eprintln!("  {} elements extracted", meshes.len());
        }

        // Assign each element to a zone
        let mut groups: FxHashMap<(String, String), Vec<usize>> = FxHashMap::default();

        for (i, mesh) in meshes.iter().enumerate() {
            let centroid = mesh_centroid(&mesh.mesh.positions);
            let zone_path = assign_to_zone(centroid, &reference.zones)
                .unwrap_or_else(|| "_unzoned".to_string());
            let ifc_class = mesh.ifc_type.clone();
            groups
                .entry((zone_path, ifc_class))
                .or_default()
                .push(i);
        }

        if !quiet {
            eprintln!("  {} groups (zone x class)", groups.len());
        }

        // Write GLBs and build tile groups
        let mut tile_groups: Vec<TileGroup> = Vec::new();

        for ((zone_path, ifc_class), indices) in &groups {
            let group_meshes: Vec<&GuidMesh> = indices.iter().map(|&i| &meshes[i]).collect();

            // Compute group AABB
            let mut aabb = Aabb::empty();
            for m in &group_meshes {
                let mesh_bb = mesh_aabb(&m.mesh.positions);
                if mesh_bb.is_valid() {
                    aabb = aabb.union(&mesh_bb);
                }
            }

            if !aabb.is_valid() {
                continue;
            }

            // Write GLB
            let glb_dir = model_dir.join(zone_path);
            std::fs::create_dir_all(&glb_dir)?;
            let glb_path = glb_dir.join(format!("{}.glb", ifc_class));

            // Collect owned GuidMeshes for write_glb
            let mesh_refs: Vec<GuidMesh> = indices
                .iter()
                .map(|&i| {
                    let m = &meshes[i];
                    GuidMesh {
                        global_id: m.global_id.clone(),
                        name: m.name.clone(),
                        ifc_type: m.ifc_type.clone(),
                        step_id: m.step_id,
                        mesh: m.mesh.clone(),
                        color: m.color,
                    }
                })
                .collect();

            glb::write_glb(&mesh_refs, &glb_path, config)?;

            tile_groups.push(TileGroup {
                zone_path: zone_path.clone(),
                ifc_class: ifc_class.clone(),
                mesh_indices: indices.clone(),
                aabb,
            });
        }

        // Build per-model tileset
        let model_tile = build_model_tileset(
            &reference,
            &tile_groups,
            model_name,
            ifc_path,
            &model_dir,
        )?;

        if model_tile.bounding_volume.bbox.iter().any(|v| v.is_finite()) {
            let model_aabb = bbox_to_aabb(&model_tile.bounding_volume.bbox);
            root_aabb = root_aabb.union(&model_aabb);
        }

        let model_aabb_for_ge = bbox_to_aabb(&model_tile.bounding_volume.bbox);
        let model_ge = model_aabb_for_ge.bounding_sphere_radius() * 0.5;
        model_tiles.push(TileNode {
            bounding_volume: model_tile.bounding_volume.clone(),
            geometric_error: model_ge,
            refine: None,
            metadata: Some(TileMetadata {
                class: "Model".to_string(),
                properties: serde_json::json!({
                    "name": model_name,
                    "source": ifc_path.file_name().map(|f| f.to_string_lossy().to_string()).unwrap_or_default()
                }),
            }),
            content: Some(TileContent {
                uri: format!("{}/tileset.json", model_name),
            }),
            children: vec![],
        });

        if !quiet {
            eprintln!("  Wrote per-model tileset to {}/tileset.json", model_name);
        }
    }

    // Build root federation tileset
    if !root_aabb.is_valid() {
        root_aabb = Aabb {
            min: [0.0; 3],
            max: [1.0; 3],
        };
    }

    let root_ge = root_aabb.bounding_sphere_radius();
    let mut root_node = TileNode {
        bounding_volume: BoundingVolume {
            bbox: root_aabb.to_3dtiles_box(),
        },
        geometric_error: root_ge,
        refine: Some("ADD".to_string()),
        metadata: None,
        content: None,
        children: model_tiles,
    };
    enforce_ge_monotonicity(&mut root_node);

    let root_tileset = Tileset {
        asset: TilesetAsset {
            version: "1.1".to_string(),
            generator: Some("ifc-lite-headless".to_string()),
            gltf_up_axis: Some("z".to_string()),
        },
        geometric_error: root_ge,
        schema: Some(TilesetSchema {
            id: "ifc-lite-schema".to_string(),
            classes: tileset_schema_classes(),
        }),
        root: root_node,
    };

    let root_json = serde_json::to_string_pretty(&root_tileset)?;
    std::fs::write(output_dir.join("tileset.json"), &root_json)?;

    if !quiet {
        eprintln!("\nWrote root tileset to {}/tileset.json", output_dir.display());
    }

    Ok(())
}

fn build_model_tileset(
    reference: &ReferenceFile,
    groups: &[TileGroup],
    model_name: &str,
    _source_path: &Path,
    model_dir: &Path,
) -> Result<TileNode, Box<dyn std::error::Error>> {
    // Build a map from zone_path to its groups
    let mut zone_groups: FxHashMap<&str, Vec<&TileGroup>> = FxHashMap::default();
    for g in groups {
        zone_groups.entry(&g.zone_path).or_default().push(g);
    }

    // Build tile tree mirroring zone tree
    let mut children: Vec<TileNode> = Vec::new();
    let mut model_aabb = Aabb::empty();

    // Process zones from reference
    for zone in &reference.zones {
        if let Some(node) = build_zone_tile(zone, "", &zone_groups, model_name) {
            let zone_aabb = bbox_to_aabb(&node.bounding_volume.bbox);
            model_aabb = model_aabb.union(&zone_aabb);
            children.push(node);
        }
    }

    // Process _unzoned
    if let Some(unzoned_groups) = zone_groups.get("_unzoned") {
        let (node, aabb) = build_leaf_zone_tile("_unzoned", "_unzoned", unzoned_groups, model_name, 0);
        model_aabb = model_aabb.union(&aabb);
        children.push(node);
    }

    if !model_aabb.is_valid() {
        model_aabb = Aabb {
            min: [0.0; 3],
            max: [1.0; 3],
        };
    }

    let model_ge = model_aabb.bounding_sphere_radius() * 0.5;
    let model_root = TileNode {
        bounding_volume: BoundingVolume {
            bbox: model_aabb.to_3dtiles_box(),
        },
        geometric_error: model_ge,
        refine: Some("ADD".to_string()),
        metadata: None,
        content: None,
        children,
    };

    let mut model_root_fixed = model_root.clone();
    enforce_ge_monotonicity(&mut model_root_fixed);

    let model_tileset = Tileset {
        asset: TilesetAsset {
            version: "1.1".to_string(),
            generator: Some("ifc-lite-headless".to_string()),
            gltf_up_axis: Some("z".to_string()),
        },
        geometric_error: model_ge,
        schema: None,
        root: model_root_fixed,
    };

    let json = serde_json::to_string_pretty(&model_tileset)?;
    std::fs::write(model_dir.join("tileset.json"), &json)?;

    Ok(model_root)
}

fn build_zone_tile(
    zone: &Zone,
    parent_path: &str,
    zone_groups: &FxHashMap<&str, Vec<&TileGroup>>,
    model_name: &str,
) -> Option<TileNode> {
    let current_path = if parent_path.is_empty() {
        sanitize_zone_name(&zone.name)
    } else {
        format!("{}/{}", parent_path, sanitize_zone_name(&zone.name))
    };

    let depth = current_path.matches('/').count();
    let mut children: Vec<TileNode> = Vec::new();
    let mut zone_aabb = Aabb::empty();

    // Recurse into child zones
    for child in &zone.children {
        if let Some(child_node) = build_zone_tile(child, &current_path, zone_groups, model_name) {
            let child_aabb = bbox_to_aabb(&child_node.bounding_volume.bbox);
            zone_aabb = zone_aabb.union(&child_aabb);
            children.push(child_node);
        }
    }

    // Add leaf tiles for groups directly in this zone
    if let Some(groups) = zone_groups.get(current_path.as_str()) {
        for g in groups {
            zone_aabb = zone_aabb.union(&g.aabb);
            children.push(TileNode {
                bounding_volume: BoundingVolume {
                    bbox: g.aabb.to_3dtiles_box(),
                },
                geometric_error: 0.0,
                refine: None,
                metadata: Some(TileMetadata {
                    class: "IfcClassGroup".to_string(),
                    properties: serde_json::json!({
                        "ifcClass": g.ifc_class,
                        "zone": current_path,
                        "model": model_name,
                        "elementCount": g.mesh_indices.len()
                    }),
                }),
                content: Some(TileContent {
                    uri: format!("{}/{}.glb", current_path, g.ifc_class),
                }),
                children: vec![],
            });
        }
    }

    if children.is_empty() {
        return None; // No content in this zone
    }

    if !zone_aabb.is_valid() {
        return None;
    }

    // Contentless zone nodes must have high GE so the renderer always
    // traverses down to the leaf content tiles (GLBs).
    // Use a very large value; enforce_ge_monotonicity will cap it relative to parent.
    let zone_ge = 1e6;
    Some(TileNode {
        bounding_volume: BoundingVolume {
            bbox: zone_aabb.to_3dtiles_box(),
        },
        geometric_error: zone_ge,
        refine: Some("ADD".to_string()),
        metadata: Some(TileMetadata {
            class: "Zone".to_string(),
            properties: serde_json::json!({
                "name": zone.name,
                "path": current_path,
                "depth": depth
            }),
        }),
        content: None,
        children,
    })
}

fn build_leaf_zone_tile(
    name: &str,
    path: &str,
    groups: &[&TileGroup],
    model_name: &str,
    depth: usize,
) -> (TileNode, Aabb) {
    let mut zone_aabb = Aabb::empty();
    let mut children: Vec<TileNode> = Vec::new();

    for g in groups {
        zone_aabb = zone_aabb.union(&g.aabb);
        children.push(TileNode {
            bounding_volume: BoundingVolume {
                bbox: g.aabb.to_3dtiles_box(),
            },
            geometric_error: 0.0,
            refine: None,
            metadata: Some(TileMetadata {
                class: "IfcClassGroup".to_string(),
                properties: serde_json::json!({
                    "ifcClass": g.ifc_class,
                    "zone": path,
                    "model": model_name,
                    "elementCount": g.mesh_indices.len()
                }),
            }),
            content: Some(TileContent {
                uri: format!("{}/{}.glb", path, g.ifc_class),
            }),
            children: vec![],
        });
    }

    let zone_ge = 1e6;
    let node = TileNode {
        bounding_volume: BoundingVolume {
            bbox: zone_aabb.to_3dtiles_box(),
        },
        geometric_error: zone_ge,
        refine: Some("ADD".to_string()),
        metadata: Some(TileMetadata {
            class: "Zone".to_string(),
            properties: serde_json::json!({
                "name": name,
                "path": path,
                "depth": depth
            }),
        }),
        content: None,
        children,
    };

    (node, zone_aabb)
}

/// Enforce geometric error monotonicity: every child must have GE < parent.
/// Recursively caps children at `parent_ge * 0.5` if they exceed the parent.
fn enforce_ge_monotonicity(node: &mut TileNode) {
    let parent_ge = node.geometric_error;
    for child in &mut node.children {
        if child.geometric_error >= parent_ge {
            child.geometric_error = parent_ge * 0.5;
        }
        enforce_ge_monotonicity(child);
    }
}

fn bbox_to_aabb(bbox: &[f64; 12]) -> Aabb {
    let cx = bbox[0];
    let cy = bbox[1];
    let cz = bbox[2];
    let hx = bbox[3];
    let hy = bbox[7];
    let hz = bbox[11];
    Aabb {
        min: [cx - hx, cy - hy, cz - hz],
        max: [cx + hx, cy + hy, cz + hz],
    }
}

fn count_zones(zones: &[Zone]) -> usize {
    zones
        .iter()
        .map(|z| 1 + count_zones(&z.children))
        .sum()
}

fn tileset_schema_classes() -> serde_json::Value {
    serde_json::json!({
        "Model": {
            "properties": {
                "name": { "type": "STRING" },
                "source": { "type": "STRING" }
            }
        },
        "Zone": {
            "properties": {
                "name": { "type": "STRING" },
                "path": { "type": "STRING" },
                "depth": { "type": "UINT32" }
            }
        },
        "IfcClassGroup": {
            "properties": {
                "ifcClass": { "type": "STRING" },
                "zone": { "type": "STRING" },
                "model": { "type": "STRING" },
                "elementCount": { "type": "UINT32" }
            }
        }
    })
}
