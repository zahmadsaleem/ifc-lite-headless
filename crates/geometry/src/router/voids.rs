// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Void (opening) subtraction using full 3D mesh CSG.
//!
//! Aligned with ThatOpen/engine_web-ifc approach:
//! 1. Get host element mesh (wall, slab, etc.)
//! 2. For each related IfcOpeningElement, get its full 3D mesh
//! 3. CSG subtract each opening mesh from the host mesh
//!
//! This replaces the previous AABB clipping approach which produced
//! visual artifacts (triangular wedges, mangled geometry).

use super::GeometryRouter;
use crate::csg::ClippingProcessor;
use crate::{Mesh, Result};
use ifc_lite_core::{DecodedEntity, EntityDecoder, IfcType};
use rustc_hash::FxHashMap;

impl GeometryRouter {
    /// Process an element with void subtraction (openings).
    ///
    /// Uses full 3D mesh CSG subtraction (like web-ifc):
    /// 1. Process the host element to get its mesh
    /// 2. For each IfcOpeningElement, get its individual representation items
    /// 3. CSG subtract each item mesh from the host separately
    ///
    /// Opening elements with multiple representation items (e.g. two extrusions
    /// approaching from opposite sides) are processed item-by-item to avoid
    /// non-manifold merged meshes that confuse CSG.
    pub fn process_element_with_voids(
        &self,
        element: &DecodedEntity,
        decoder: &mut EntityDecoder,
        void_index: &FxHashMap<u32, Vec<u32>>,
    ) -> Result<Mesh> {
        // Check if this element has any openings
        let opening_ids = match void_index.get(&element.id) {
            Some(ids) if !ids.is_empty() => ids,
            _ => return self.process_element(element, decoder),
        };

        // Get host element mesh
        let mut result = self.process_element(element, decoder)?;
        if result.is_empty() {
            return Ok(result);
        }

        // Validate host mesh before CSG
        if !result.positions.iter().all(|&v| v.is_finite()) || result.triangle_count() < 4 {
            return Ok(result);
        }

        let clipper = ClippingProcessor::new();

        // Save original host for final artifact cleanup bounds
        let original_host = result.clone();

        // Process each opening with raw CSG (no per-step artifact removal).
        // Artifact removal between steps can discard valid wall sections that
        // become temporarily disconnected during sequential subtracts.
        for &opening_id in opening_ids.iter() {
            let opening_entity = match decoder.decode_by_id(opening_id) {
                Ok(e) => e,
                Err(_) => continue,
            };

            let opening_meshes = self.get_opening_item_meshes(&opening_entity, decoder);

            for opening_mesh in opening_meshes {
                if !opening_mesh.positions.iter().all(|&v| v.is_finite())
                    || opening_mesh.positions.len() < 9
                {
                    continue;
                }

                match clipper.subtract_mesh_raw(&result, &opening_mesh) {
                    Ok(csg_result)
                        if !csg_result.is_empty() && csg_result.triangle_count() >= 4 =>
                    {
                        result = csg_result;
                    }
                    _ => {}
                }
            }
        }

        // Single artifact cleanup pass using original host bounds.
        // This avoids the problem of per-step cleanup discarding valid
        // wall sections that are temporarily disconnected mid-sequence.
        Ok(clipper.clean_artifacts(&result, &original_host))
    }

    /// Get individual representation item meshes for an opening element.
    ///
    /// Instead of merging all items into one mesh (which creates non-manifold
    /// geometry when items share faces), returns each item as a separate mesh.
    /// This allows CSG to process each watertight solid individually.
    fn get_opening_item_meshes(
        &self,
        opening: &DecodedEntity,
        decoder: &mut EntityDecoder,
    ) -> Vec<Mesh> {
        let mut meshes = Vec::new();

        // Get representation
        let repr_attr = match opening.get(6) {
            Some(attr) if !attr.is_null() => attr,
            _ => return meshes,
        };

        let repr = match decoder.resolve_ref(repr_attr) {
            Ok(Some(r)) if r.ifc_type == IfcType::IfcProductDefinitionShape => r,
            _ => return meshes,
        };

        let repr_list_attr = match repr.get(2) {
            Some(attr) => attr,
            None => return meshes,
        };

        let representations = match decoder.resolve_ref_list(repr_list_attr) {
            Ok(r) => r,
            Err(_) => return meshes,
        };

        // Get placement transform
        let placement_transform = self
            .get_placement_transform_from_element(opening, decoder)
            .ok();

        for shape_rep in &representations {
            if shape_rep.ifc_type != IfcType::IfcShapeRepresentation {
                continue;
            }

            // Only process Body/SweptSolid/Tessellation etc.
            if let Some(rep_type_attr) = shape_rep.get(2) {
                if let Some(rep_type) = rep_type_attr.as_string() {
                    if !matches!(
                        rep_type,
                        "Body"
                            | "SweptSolid"
                            | "SolidModel"
                            | "Brep"
                            | "CSG"
                            | "Clipping"
                            | "Tessellation"
                    ) {
                        continue;
                    }
                }
            }

            let items_attr = match shape_rep.get(3) {
                Some(attr) => attr,
                None => continue,
            };

            let items = match decoder.resolve_ref_list(items_attr) {
                Ok(i) => i,
                Err(_) => continue,
            };

            // Process each item individually
            for item in &items {
                if let Ok(mut mesh) = self.process_representation_item(item, decoder) {
                    if !mesh.is_empty() {
                        // Apply placement
                        if let Some(ref transform) = placement_transform {
                            let mut scaled = *transform;
                            self.scale_transform(&mut scaled);
                            self.transform_mesh(&mut mesh, &scaled);
                        }
                        meshes.push(mesh);
                    }
                }
            }
        }

        // If we got multiple items, return them individually.
        // If only one item (common case), just return it as-is.
        meshes
    }
}
