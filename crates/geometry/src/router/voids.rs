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
use ifc_lite_core::{DecodedEntity, EntityDecoder};
use rustc_hash::FxHashMap;

impl GeometryRouter {
    /// Process an element with void subtraction (openings).
    ///
    /// Uses full 3D mesh CSG subtraction (like web-ifc):
    /// 1. Process the host element to get its mesh
    /// 2. For each IfcOpeningElement, process it to get its mesh
    /// 3. CSG subtract each opening mesh from the host
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

        // Process each opening and CSG subtract
        for &opening_id in opening_ids.iter() {
            let opening_entity = match decoder.decode_by_id(opening_id) {
                Ok(e) => e,
                Err(_) => continue,
            };

            let opening_mesh = match self.process_element(&opening_entity, decoder) {
                Ok(m) if !m.is_empty() => m,
                _ => continue,
            };

            // Validate opening mesh
            if !opening_mesh.positions.iter().all(|&v| v.is_finite())
                || opening_mesh.positions.len() < 9
            {
                continue;
            }

            // CSG subtract opening from host
            match clipper.subtract_mesh(&result, &opening_mesh) {
                Ok(csg_result) if !csg_result.is_empty() && csg_result.triangle_count() >= 4 => {
                    result = csg_result;
                }
                _ => {
                    // CSG failed or produced degenerate result — keep previous
                }
            }
        }

        Ok(result)
    }
}
