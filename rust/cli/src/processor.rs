// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! IFC processing: parse, extract geometry, and associate GUIDs.

use crate::config::{ConvertConfig, ElementNaming};
use ifc_lite_core::{build_entity_index, EntityDecoder, EntityScanner, IfcType};
use ifc_lite_geometry::{calculate_normals, GeometryRouter, Mesh};
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use std::sync::Arc;

/// A mesh associated with its IFC GlobalId and color.
pub struct GuidMesh {
    pub global_id: String,
    pub name: String,
    pub ifc_type: String,
    pub step_id: u32,
    pub mesh: Mesh,
    pub color: [f32; 4],
}

impl GuidMesh {
    /// Get the display name based on the naming config.
    pub fn display_name(&self, naming: ElementNaming) -> String {
        match naming {
            ElementNaming::Guids => self.global_id.clone(),
            ElementNaming::Names => {
                if self.name.is_empty() {
                    self.global_id.clone()
                } else {
                    self.name.clone()
                }
            }
            ElementNaming::StepIds => format!("#{}", self.step_id),
            ElementNaming::Types => self.ifc_type.clone(),
        }
    }
}

struct EntityJob {
    id: u32,
    global_id: String,
    name: String,
    ifc_type: IfcType,
    start: usize,
    end: usize,
}

fn get_refs_from_list(
    entity: &ifc_lite_core::DecodedEntity,
    index: usize,
) -> Option<Vec<u32>> {
    let list = entity.get_list(index)?;
    let refs: Vec<u32> = list.iter().filter_map(|v| v.as_entity_ref()).collect();
    if refs.is_empty() { None } else { Some(refs) }
}

/// Process an IFC file and return meshes with their GlobalIds.
pub fn process_ifc(content: &str, config: &ConvertConfig) -> Vec<GuidMesh> {
    let entity_index = Arc::new(build_entity_index(content));
    let mut decoder = EntityDecoder::with_arc_index(content, entity_index.clone());

    // Build style index
    let style_index = Arc::new(build_style_indices(content, &mut decoder));

    // Scan entities
    let mut scanner = EntityScanner::new(content);
    let mut faceted_brep_ids: Vec<u32> = Vec::new();
    let mut void_index: FxHashMap<u32, Vec<u32>> = FxHashMap::default();
    let mut entity_jobs: Vec<EntityJob> = Vec::with_capacity(2000);

    while let Some((id, type_name, start, end)) = scanner.next_entity() {
        if type_name == "IFCFACETEDBREP" {
            faceted_brep_ids.push(id);
        } else if type_name == "IFCRELVOIDSELEMENT" {
            if let Ok(entity) = decoder.decode_at(start, end) {
                if let (Some(host), Some(opening)) = (entity.get_ref(4), entity.get_ref(5)) {
                    void_index.entry(host).or_default().push(opening);
                }
            }
        }

        if ifc_lite_core::has_geometry_by_name(type_name) {
            // Apply include/exclude filters
            if !config.should_process(type_name) {
                continue;
            }

            if let Ok(entity) = decoder.decode_at(start, end) {
                let global_id = entity
                    .get_string(0)
                    .unwrap_or("")
                    .to_string();
                if !global_id.is_empty() {
                    let name = entity.get_string(2).unwrap_or("").to_string();
                    entity_jobs.push(EntityJob {
                        id,
                        global_id,
                        name,
                        ifc_type: entity.ifc_type,
                        start,
                        end,
                    });
                }
            }
        }
    }

    // Setup geometry router
    let router = GeometryRouter::with_units(content, &mut decoder);
    if !faceted_brep_ids.is_empty() {
        router.preprocess_faceted_breps(&faceted_brep_ids, &mut decoder);
    }

    // Detect RTC offset
    let jobs_for_rtc: Vec<(u32, usize, usize, IfcType)> = entity_jobs
        .iter()
        .map(|j| (j.id, j.start, j.end, j.ifc_type))
        .collect();
    let rtc_offset = router.detect_rtc_offset_from_jobs(&jobs_for_rtc, &mut decoder);

    let content_arc = Arc::new(content.to_string());
    let entity_index_arc = entity_index;
    let unit_scale = router.unit_scale();
    let void_index_arc = Arc::new(void_index);

    // Parallel geometry processing
    entity_jobs
        .into_par_iter()
        .filter_map(|job| {
            let mut local_decoder =
                EntityDecoder::with_arc_index(&content_arc, entity_index_arc.clone());

            let entity = local_decoder.decode_at(job.start, job.end).ok()?;

            // Skip entities without representation
            let has_representation = entity.get(6).is_some_and(|a| !a.is_null());
            if !has_representation {
                return None;
            }

            let local_router = if rtc_offset.0.abs() > 1.0
                || rtc_offset.1.abs() > 1.0
                || rtc_offset.2.abs() > 1.0
            {
                GeometryRouter::with_scale_and_rtc(unit_scale, rtc_offset)
            } else {
                GeometryRouter::with_scale(unit_scale)
            };

            let mut mesh = local_router
                .process_element_with_voids(&entity, &mut local_decoder, void_index_arc.as_ref())
                .ok()?;

            if mesh.is_empty() {
                return None;
            }

            if mesh.normals.is_empty() {
                calculate_normals(&mut mesh);
            }

            let color = style_index
                .get(&job.id)
                .copied()
                .unwrap_or_else(|| get_default_color(&job.ifc_type));

            Some(GuidMesh {
                global_id: job.global_id,
                name: job.name,
                ifc_type: job.ifc_type.name().to_string(),
                step_id: job.id,
                mesh,
                color,
            })
        })
        .collect()
}

fn build_style_indices(
    content: &str,
    decoder: &mut EntityDecoder,
) -> FxHashMap<u32, [f32; 4]> {
    let mut geometry_styles: FxHashMap<u32, [f32; 4]> = FxHashMap::default();
    let mut element_repr_ids: Vec<(u32, u32)> = Vec::with_capacity(2000);
    let mut scanner = EntityScanner::new(content);

    while let Some((id, type_name, start, end)) = scanner.next_entity() {
        if type_name == "IFCSTYLEDITEM" {
            if let Ok(styled_item) = decoder.decode_at(start, end) {
                if let Some(geometry_id) = styled_item.get_ref(0) {
                    if !geometry_styles.contains_key(&geometry_id) {
                        if let Some(color) = extract_color_from_styled_item(&styled_item, decoder) {
                            geometry_styles.insert(geometry_id, color);
                        }
                    }
                }
            }
        } else if ifc_lite_core::has_geometry_by_name(type_name) {
            if let Ok(element) = decoder.decode_at(start, end) {
                if let Some(repr_id) = element.get_ref(6) {
                    element_repr_ids.push((id, repr_id));
                }
            }
        }
    }

    let mut element_styles: FxHashMap<u32, [f32; 4]> = FxHashMap::default();
    for (element_id, repr_id) in element_repr_ids {
        if let Some(color) = find_color_in_representation(repr_id, &geometry_styles, decoder) {
            element_styles.insert(element_id, color);
        }
    }

    element_styles
}

fn find_color_in_representation(
    repr_id: u32,
    geometry_styles: &FxHashMap<u32, [f32; 4]>,
    decoder: &mut EntityDecoder,
) -> Option<[f32; 4]> {
    let repr = decoder.decode_by_id(repr_id).ok()?;
    let repr_list = get_refs_from_list(&repr, 2)?;

    for shape_repr_id in repr_list {
        if let Ok(shape_repr) = decoder.decode_by_id(shape_repr_id) {
            if let Some(items) = get_refs_from_list(&shape_repr, 3) {
                for item_id in items {
                    if let Some(color) = geometry_styles.get(&item_id) {
                        return Some(*color);
                    }
                    if let Ok(item) = decoder.decode_by_id(item_id) {
                        if item.ifc_type == IfcType::IfcMappedItem {
                            if let Some(source_id) = item.get_ref(0) {
                                if let Ok(source) = decoder.decode_by_id(source_id) {
                                    if let Some(mapped_repr_id) = source.get_ref(1) {
                                        if let Ok(mapped_repr) =
                                            decoder.decode_by_id(mapped_repr_id)
                                        {
                                            if let Some(mapped_items) =
                                                get_refs_from_list(&mapped_repr, 3)
                                            {
                                                for mapped_item_id in mapped_items {
                                                    if let Some(color) =
                                                        geometry_styles.get(&mapped_item_id)
                                                    {
                                                        return Some(*color);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    None
}

fn extract_color_from_styled_item(
    styled_item: &ifc_lite_core::DecodedEntity,
    decoder: &mut EntityDecoder,
) -> Option<[f32; 4]> {
    let style_refs = get_refs_from_list(styled_item, 1)?;
    for style_id in style_refs {
        if let Ok(style) = decoder.decode_by_id(style_id) {
            if let Some(inner_refs) = get_refs_from_list(&style, 0) {
                for inner_id in inner_refs {
                    if let Some(color) = extract_surface_style_color(inner_id, decoder) {
                        return Some(color);
                    }
                }
            }
            if let Some(color) = extract_surface_style_color(style_id, decoder) {
                return Some(color);
            }
        }
    }
    None
}

fn extract_surface_style_color(style_id: u32, decoder: &mut EntityDecoder) -> Option<[f32; 4]> {
    let style = decoder.decode_by_id(style_id).ok()?;
    let rendering_refs = get_refs_from_list(&style, 2)?;

    for rendering_id in rendering_refs {
        if let Ok(rendering) = decoder.decode_by_id(rendering_id) {
            if let Some(color_id) = rendering.get_ref(0) {
                if let Ok(color) = decoder.decode_by_id(color_id) {
                    let r = color.get_float(1).unwrap_or(0.8) as f32;
                    let g = color.get_float(2).unwrap_or(0.8) as f32;
                    let b = color.get_float(3).unwrap_or(0.8) as f32;
                    let alpha = 1.0 - rendering.get_float(1).unwrap_or(0.0) as f32;
                    return Some([r, g, b, alpha.max(0.0).min(1.0)]);
                }
            }
        }
    }
    None
}

fn get_default_color(ifc_type: &IfcType) -> [f32; 4] {
    match ifc_type {
        IfcType::IfcWall | IfcType::IfcWallStandardCase => [0.85, 0.85, 0.85, 1.0],
        IfcType::IfcSlab => [0.7, 0.7, 0.7, 1.0],
        IfcType::IfcRoof => [0.6, 0.5, 0.4, 1.0],
        IfcType::IfcColumn | IfcType::IfcBeam | IfcType::IfcMember => [0.6, 0.65, 0.7, 1.0],
        IfcType::IfcWindow => [0.6, 0.8, 1.0, 0.4],
        IfcType::IfcDoor => [0.6, 0.45, 0.3, 1.0],
        IfcType::IfcStair | IfcType::IfcStairFlight => [0.75, 0.75, 0.75, 1.0],
        IfcType::IfcRailing => [0.4, 0.4, 0.45, 1.0],
        IfcType::IfcPlate | IfcType::IfcCovering => [0.8, 0.8, 0.8, 1.0],
        IfcType::IfcFurnishingElement => [0.5, 0.35, 0.2, 1.0],
        IfcType::IfcSpace => [0.2, 0.85, 1.0, 0.3],
        IfcType::IfcOpeningElement => [1.0, 0.42, 0.29, 0.4],
        IfcType::IfcSite => [0.4, 0.8, 0.3, 1.0],
        IfcType::IfcBuildingElementProxy => [0.6, 0.6, 0.6, 1.0],
        _ => [0.8, 0.8, 0.8, 1.0],
    }
}
