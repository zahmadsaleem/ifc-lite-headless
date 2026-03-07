use serde::{Deserialize, Serialize};

// --- Reference file types ---

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReferenceFile {
    pub version: u32,
    pub source: String,
    pub zones: Vec<Zone>,
    pub coordinate_system: CoordinateSystem,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Zone {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub global_id: Option<String>,
    pub volume: ZoneVolume,
    #[serde(default)]
    pub children: Vec<Zone>,
}

/// Zone volume as one or more vertical slices.
/// Supports shorthand (flat polygon/z_min/z_max) or explicit slices array.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ZoneVolume {
    Slices {
        slices: Vec<Slice>,
    },
    /// Shorthand: single slice specified directly
    Single {
        polygon: Vec<[f64; 2]>,
        z_min: f64,
        z_max: f64,
    },
}

impl ZoneVolume {
    /// Convert to normalized form (always returns a Vec of slices)
    pub fn normalized(&self) -> Vec<Slice> {
        match self {
            ZoneVolume::Slices { slices } => slices.clone(),
            ZoneVolume::Single {
                polygon,
                z_min,
                z_max,
            } => vec![Slice {
                polygon: polygon.clone(),
                z_min: *z_min,
                z_max: *z_max,
            }],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Slice {
    pub polygon: Vec<[f64; 2]>,
    pub z_min: f64,
    pub z_max: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinateSystem {
    pub up_axis: String,
    pub unit_scale: f64,
    pub rtc_offset: [f64; 3],
}

// --- Tileset.json types ---

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tileset {
    pub asset: TilesetAsset,
    #[serde(rename = "geometricError")]
    pub geometric_error: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub schema: Option<TilesetSchema>,
    pub root: TileNode,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TilesetAsset {
    pub version: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub generator: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TilesetSchema {
    pub id: String,
    pub classes: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TileNode {
    #[serde(rename = "boundingVolume")]
    pub bounding_volume: BoundingVolume,
    #[serde(rename = "geometricError")]
    pub geometric_error: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refine: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<TileMetadata>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<TileContent>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub children: Vec<TileNode>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoundingVolume {
    #[serde(rename = "box")]
    pub bbox: [f64; 12],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TileMetadata {
    pub class: String,
    pub properties: serde_json::Value,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TileContent {
    pub uri: String,
}

// --- Internal working types ---

/// AABB represented as min/max corners
#[derive(Debug, Clone, Copy)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb {
    pub fn empty() -> Self {
        Self {
            min: [f64::MAX; 3],
            max: [f64::MIN; 3],
        }
    }

    pub fn expand_point(&mut self, p: [f64; 3]) {
        for i in 0..3 {
            if p[i] < self.min[i] {
                self.min[i] = p[i];
            }
            if p[i] > self.max[i] {
                self.max[i] = p[i];
            }
        }
    }

    pub fn union(&self, other: &Aabb) -> Aabb {
        Aabb {
            min: [
                self.min[0].min(other.min[0]),
                self.min[1].min(other.min[1]),
                self.min[2].min(other.min[2]),
            ],
            max: [
                self.max[0].max(other.max[0]),
                self.max[1].max(other.max[1]),
                self.max[2].max(other.max[2]),
            ],
        }
    }

    pub fn is_valid(&self) -> bool {
        self.min[0] <= self.max[0] && self.min[1] <= self.max[1] && self.min[2] <= self.max[2]
    }

    /// Convert to 3D Tiles box format: [cx, cy, cz, hx, 0, 0, 0, hy, 0, 0, 0, hz]
    pub fn to_3dtiles_box(&self) -> [f64; 12] {
        let cx = (self.min[0] + self.max[0]) / 2.0;
        let cy = (self.min[1] + self.max[1]) / 2.0;
        let cz = (self.min[2] + self.max[2]) / 2.0;
        let hx = (self.max[0] - self.min[0]) / 2.0;
        let hy = (self.max[1] - self.min[1]) / 2.0;
        let hz = (self.max[2] - self.min[2]) / 2.0;
        [cx, cy, cz, hx, 0.0, 0.0, 0.0, hy, 0.0, 0.0, 0.0, hz]
    }
}

/// A group of elements sharing the same zone path and IFC class
pub struct TileGroup {
    pub zone_path: String,
    pub ifc_class: String,
    pub mesh_indices: Vec<usize>,
    pub aabb: Aabb,
}
