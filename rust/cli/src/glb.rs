// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! GLB (Binary glTF 2.0) writer.
//!
//! Produces a single .glb file where each IFC element is a named node
//! with `node.name = GlobalId`.

use crate::config::{ConvertConfig, UpAxis};
use crate::processor::GuidMesh;
use std::io::Write;
use std::path::Path;

/// Write meshes to a GLB file. Returns the file size in bytes.
pub fn write_glb(
    meshes: &[GuidMesh],
    path: &Path,
    config: &ConvertConfig,
) -> Result<usize, Box<dyn std::error::Error>> {
    // Phase 1: Build the binary buffer (interleaved positions + normals, then indices)
    let mut bin_buffer: Vec<u8> = Vec::new();

    struct MeshMeta {
        name: String,
        ifc_type: String,
        color: [f32; 4],
        positions_offset: usize,
        positions_byte_length: usize,
        normals_offset: usize,
        normals_byte_length: usize,
        indices_offset: usize,
        indices_byte_length: usize,
        vertex_count: usize,
        index_count: usize,
        min_pos: [f32; 3],
        max_pos: [f32; 3],
    }

    let mut metas: Vec<MeshMeta> = Vec::with_capacity(meshes.len());

    let y_up = config.up_axis == UpAxis::Y;

    for gm in meshes {
        let mesh = &gm.mesh;
        let vertex_count = mesh.positions.len() / 3;
        let index_count = mesh.indices.len();

        // Compute bounding box
        let mut min_pos = [f32::MAX; 3];
        let mut max_pos = [f32::MIN; 3];
        for i in 0..vertex_count {
            // If Y-up, swap Y and Z: (x, y, z) -> (x, z, -y)
            let (x, y, z) = if y_up {
                (
                    mesh.positions[i * 3],
                    mesh.positions[i * 3 + 2],
                    -mesh.positions[i * 3 + 1],
                )
            } else {
                (
                    mesh.positions[i * 3],
                    mesh.positions[i * 3 + 1],
                    mesh.positions[i * 3 + 2],
                )
            };
            let pos = [x, y, z];
            for j in 0..3 {
                if pos[j] < min_pos[j] {
                    min_pos[j] = pos[j];
                }
                if pos[j] > max_pos[j] {
                    max_pos[j] = pos[j];
                }
            }
        }

        // Positions
        let positions_offset = bin_buffer.len();
        if y_up {
            for i in 0..vertex_count {
                let x = mesh.positions[i * 3];
                let y = mesh.positions[i * 3 + 2];
                let z = -mesh.positions[i * 3 + 1];
                bin_buffer.extend_from_slice(&x.to_le_bytes());
                bin_buffer.extend_from_slice(&y.to_le_bytes());
                bin_buffer.extend_from_slice(&z.to_le_bytes());
            }
        } else {
            for &v in &mesh.positions {
                bin_buffer.extend_from_slice(&v.to_le_bytes());
            }
        }
        let positions_byte_length = bin_buffer.len() - positions_offset;

        // Normals
        let normals_offset = bin_buffer.len();
        if y_up {
            let normal_count = mesh.normals.len() / 3;
            for i in 0..normal_count {
                let x = mesh.normals[i * 3];
                let y = mesh.normals[i * 3 + 2];
                let z = -mesh.normals[i * 3 + 1];
                bin_buffer.extend_from_slice(&x.to_le_bytes());
                bin_buffer.extend_from_slice(&y.to_le_bytes());
                bin_buffer.extend_from_slice(&z.to_le_bytes());
            }
        } else {
            for &v in &mesh.normals {
                bin_buffer.extend_from_slice(&v.to_le_bytes());
            }
        }
        let normals_byte_length = bin_buffer.len() - normals_offset;

        // Indices (align to 4 bytes)
        while bin_buffer.len() % 4 != 0 {
            bin_buffer.push(0);
        }
        let indices_offset = bin_buffer.len();
        for &idx in &mesh.indices {
            bin_buffer.extend_from_slice(&idx.to_le_bytes());
        }
        let indices_byte_length = bin_buffer.len() - indices_offset;

        metas.push(MeshMeta {
            name: gm.display_name(config.naming),
            ifc_type: gm.ifc_type.clone(),
            color: gm.color,
            positions_offset,
            positions_byte_length,
            normals_offset,
            normals_byte_length,
            indices_offset,
            indices_byte_length,
            vertex_count,
            index_count,
            min_pos,
            max_pos,
        });
    }

    // Pad bin buffer to 4-byte alignment
    while bin_buffer.len() % 4 != 0 {
        bin_buffer.push(0);
    }

    // Phase 2: Build glTF JSON
    // Each mesh gets: 1 node, 1 mesh, 3 accessors (pos, normal, indices), 3 buffer views, 1 material
    let mut buffer_views = Vec::new();
    let mut accessors = Vec::new();
    let mut gltf_meshes = Vec::new();
    let mut nodes = Vec::new();
    let mut materials = Vec::new();

    for (i, meta) in metas.iter().enumerate() {
        let bv_base = i * 3;
        let acc_base = i * 3;

        // Buffer views: positions, normals, indices
        buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": meta.positions_offset,
            "byteLength": meta.positions_byte_length,
            "target": 34962 // ARRAY_BUFFER
        }));
        buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": meta.normals_offset,
            "byteLength": meta.normals_byte_length,
            "target": 34962
        }));
        buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": meta.indices_offset,
            "byteLength": meta.indices_byte_length,
            "target": 34963 // ELEMENT_ARRAY_BUFFER
        }));

        // Accessors: positions, normals, indices
        accessors.push(serde_json::json!({
            "bufferView": bv_base,
            "componentType": 5126, // FLOAT
            "count": meta.vertex_count,
            "type": "VEC3",
            "min": meta.min_pos,
            "max": meta.max_pos
        }));
        accessors.push(serde_json::json!({
            "bufferView": bv_base + 1,
            "componentType": 5126,
            "count": meta.vertex_count,
            "type": "VEC3"
        }));
        accessors.push(serde_json::json!({
            "bufferView": bv_base + 2,
            "componentType": 5125, // UNSIGNED_INT
            "count": meta.index_count,
            "type": "SCALAR"
        }));

        // Material with color
        let [r, g, b, a] = meta.color;
        let mut mat = serde_json::json!({
            "pbrMetallicRoughness": {
                "baseColorFactor": [r, g, b, a],
                "metallicFactor": 0.0,
                "roughnessFactor": 0.8
            }
        });
        if a < 1.0 {
            mat["alphaMode"] = serde_json::json!("BLEND");
        }
        materials.push(mat);

        // Mesh
        gltf_meshes.push(serde_json::json!({
            "primitives": [{
                "attributes": {
                    "POSITION": acc_base,
                    "NORMAL": acc_base + 1
                },
                "indices": acc_base + 2,
                "material": i
            }]
        }));

        // Node with GlobalId as name
        nodes.push(serde_json::json!({
            "name": meta.name,
            "mesh": i,
            "extras": {
                "ifcType": meta.ifc_type
            }
        }));
    }

    let node_indices: Vec<usize> = (0..nodes.len()).collect();

    let gltf = serde_json::json!({
        "asset": {
            "version": "2.0",
            "generator": "ifc2glb"
        },
        "scene": 0,
        "scenes": [{
            "name": "IFC Model",
            "nodes": node_indices
        }],
        "nodes": nodes,
        "meshes": gltf_meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{
            "byteLength": bin_buffer.len()
        }]
    });

    let json_bytes = serde_json::to_vec(&gltf)?;

    // Pad JSON to 4-byte alignment with spaces
    let json_padding = (4 - (json_bytes.len() % 4)) % 4;
    let json_chunk_length = json_bytes.len() + json_padding;

    // GLB structure:
    // Header: magic(4) + version(4) + length(4) = 12
    // JSON chunk: length(4) + type(4) + data(json_chunk_length)
    // BIN chunk: length(4) + type(4) + data(bin_buffer.len())
    let total_length =
        12 + 8 + json_chunk_length + 8 + bin_buffer.len();

    let mut file = std::fs::File::create(path)?;

    // Header
    file.write_all(b"glTF")?; // magic
    file.write_all(&2u32.to_le_bytes())?; // version
    file.write_all(&(total_length as u32).to_le_bytes())?;

    // JSON chunk
    file.write_all(&(json_chunk_length as u32).to_le_bytes())?;
    file.write_all(&0x4E4F534Au32.to_le_bytes())?; // "JSON"
    file.write_all(&json_bytes)?;
    for _ in 0..json_padding {
        file.write_all(b" ")?;
    }

    // BIN chunk
    file.write_all(&(bin_buffer.len() as u32).to_le_bytes())?;
    file.write_all(&0x004E4942u32.to_le_bytes())?; // "BIN\0"
    file.write_all(&bin_buffer)?;

    Ok(total_length)
}
