# 3D Tiles 1.1 Pipeline Spec

## Overview

Multi-model IFC federation via 3D Tiles 1.1. Processes multiple IFC files into a tileset hierarchy grouped by **spatial zones** (from a reference model) and **IFC class**, with metadata-based culling built into the tileset.json structure.

Zones are generic named volumes — not hardcoded to building storeys. They work for buildings (storey-based), infrastructure (road segments, pole spans), or any arbitrary spatial partitioning.

The pipeline has two stages:
1. **Extract** spatial structure from a reference IFC into a reusable `reference.json`
2. **Tile** one or more IFC models against that reference, producing a directory of GLBs + tileset.json

---

## CLI

New subcommands, separate from IfcConvert-compatible `convert` path.

### `extract-ref`

Extract spatial structure from an IFC file into a reusable reference.

```
ifc-lite-headless extract-ref <input.ifc> -o <reference.json> [options]
```

| Flag | Description |
|------|-------------|
| `-o, --output` | Output reference.json path (default: `<input>.reference.json`) |
| `--root-type` | Start from this spatial type (default: auto-detect). e.g., `IfcBuilding`, `IfcSite` |
| `--leaf-type` | Stop decomposing at this type (default: `IfcBuildingStorey`). Everything below becomes content, not zones |
| `--depth` | Max nesting depth (default: unlimited) |
| `--volume-mode` | How to compute zone volumes: `contained` (convex hull of contained elements, default), `geometry` (from spatial element geometry), `bbox` (AABB as 4-point polygon) |
| `-v` | Verbosity |

Future (lower priority):
| `--grid <meters>` | Auto-generate a regular grid of zones along the longest axis |

### `tileset`

Generate a 3D Tiles 1.1 tileset from one or more IFC models.

```
ifc-lite-headless tileset --ref <reference.json> --model <a.ifc> [--model <b.ifc>] -o <output_dir/> [options]
```

| Flag | Description |
|------|-------------|
| `--ref` | Path to reference.json (from `extract-ref`) |
| `--model` | IFC model to tile (repeatable). Name inferred from filename stem |
| `--model-name` | Override model name for previous `--model` (optional) |
| `-o, --output` | Output directory (default: `./tileset/`) |
| `--exclude` | IFC types to exclude (default: IfcOpeningElement IfcSpace) |
| `--include` | IFC types to include (whitelist mode) |
| `--y-up` | Y-up axis (default: Z-up) |
| `-j, --threads` | Thread count |
| `--deflection` | Curve tessellation tolerance |
| `-v` | Verbosity |

---

## Reference File Format

`reference.json` captures a tree of spatial zones for reuse across tiling runs. Zones are generic named volumes — the tiler doesn't care whether they came from storeys, spaces, or hand-authored regions.

### Zone volume: polygonal extrusion with slices

Each zone's volume is one or more **vertical slices**: a 2D polygon extruded between z_min and z_max. The union of slices defines the zone volume.

- **Single slice** (common case — 90%+): a polygon + z-range
- **Multiple slices**: handles vertical L-shapes, setbacks, stepped volumes

Containment test:
1. For each slice: check `z_min <= centroid.z < z_max`
2. If Z is in range: 2D point-in-polygon (ray casting) on centroid XY against slice polygon
3. First matching slice → element is inside this zone

### Example: Building with storeys

```json
{
  "version": 1,
  "source": "building.ifc",
  "zones": [
    {
      "name": "Building A",
      "global_id": "2O2Fr$t4X7Zf8NOew3FLOH",
      "volume": {
        "slices": [
          {
            "polygon": [[0,0], [40,0], [40,30], [0,30]],
            "z_min": -1.0,
            "z_max": 25.0
          }
        ]
      },
      "children": [
        {
          "name": "Level 0",
          "global_id": "3cUkl32yn9qRSPvBJJ3s$w",
          "volume": {
            "slices": [
              {
                "polygon": [[0,0], [40,0], [40,30], [0,30]],
                "z_min": -0.5,
                "z_max": 3.5
              }
            ]
          },
          "children": []
        },
        {
          "name": "Level 1",
          "global_id": "1xRRf$t4X7Zf8NOew3FLOH",
          "volume": {
            "slices": [
              {
                "polygon": [[0,0], [40,0], [40,30], [0,30]],
                "z_min": 3.5,
                "z_max": 6.7
              }
            ]
          },
          "children": []
        }
      ]
    }
  ],
  "coordinate_system": {
    "up_axis": "Z",
    "unit_scale": 1.0,
    "rtc_offset": [0.0, 0.0, 0.0]
  }
}
```

### Single-slice shorthand

For convenience, a zone with one slice can use the flat form:

```json
{
  "name": "Level 0",
  "volume": {
    "polygon": [[0,0], [40,0], [40,30], [0,30]],
    "z_min": -0.5,
    "z_max": 3.5
  }
}
```

The tiler normalizes this internally to `slices: [{ polygon, z_min, z_max }]`.

### Example: Infrastructure (flat, no nesting)

```json
{
  "version": 1,
  "source": "road.ifc",
  "zones": [
    {
      "name": "Seg_001",
      "volume": {
        "polygon": [[0,0], [250,2], [252,12], [2,10]],
        "z_min": -5.0,
        "z_max": 25.0
      },
      "children": []
    },
    {
      "name": "Seg_002",
      "volume": {
        "polygon": [[250,2], [500,4], [502,14], [252,12]],
        "z_min": -5.0,
        "z_max": 25.0
      },
      "children": []
    }
  ]
}
```

### Example: Vertical L-shape (multi-slice)

A zone with a setback — full width below 11m, half width above:

```json
{
  "name": "Level 3 (L-shaped)",
  "volume": {
    "slices": [
      {
        "polygon": [[0,0], [40,0], [40,30], [0,30]],
        "z_min": 9.0,
        "z_max": 11.0
      },
      {
        "polygon": [[0,0], [20,0], [20,30], [0,30]],
        "z_min": 11.0,
        "z_max": 13.0
      }
    ]
  }
}
```

---

## Zone Assignment

### Spanning elements caveat

Elements that physically span multiple sibling zones (e.g., a column running through Level 0 and Level 1) are assigned based on centroid only. If the centroid lands in Level 0, the element lives entirely in Level 0's GLB — even if it visually extends into Level 1. This means hiding Level 0 would hide the column entirely.

Phase 1 accepts this trade-off for simplicity. Future options if it becomes a problem:
- **`_shared` at each zone level**: elements whose AABB intersects multiple siblings go into a shared group (requires AABB-vs-polygon intersection)
- **Duplication**: element appears in every zone it intersects (increases file size, client may double-render)

### Algorithm

Elements are assigned to zones by **centroid containment**, deepest match wins:

1. Compute the AABB centroid of the element's mesh
2. Walk the zone tree depth-first
3. For each zone, test if the centroid is inside any of the zone's slices (Z-range + 2D point-in-polygon)
4. If inside, recurse into children to find a more specific match
5. Assign to the deepest zone that contains the centroid
6. If centroid is inside a parent but no child → assigned to the parent zone
7. If centroid is outside all zones → assigned to `_unzoned`

### Overlap policy

**One reference = one grouping dimension.** Sibling zones should not overlap. If you need to tile the same models by two different spatial partitionings (e.g., storeys AND work zones), run `tileset` twice with different references into different output directories.

Overlapping siblings within a single reference file is treated as a warning. First match wins (zone order in the file).

### Zone polygon extraction from IFC (`extract-ref`)

Fallback chain for computing zone polygons:

1. **Contained elements** (`--volume-mode contained`, default): For each spatial element, find all elements assigned to it via `IfcRelContainedInSpatialStructure`. Compute the convex hull of their XY-projected positions. Use the min/max Z of those elements for z_min/z_max.

2. **Spatial element geometry** (`--volume-mode geometry`): Extract the XY polygon from the spatial element's own representation (e.g., IfcSpace often has SweptSolid geometry with a flat profile).

3. **AABB fallback** (`--volume-mode bbox`): Compute the AABB of contained elements and represent it as a 4-point polygon.

For `IfcBuildingStorey` specifically (which rarely has its own geometry):
- Use elevation from the storey entity
- Infer height as `next_storey.elevation - this_storey.elevation` (last storey: scan model bounds or default 4.0m)
- Get polygon from contained elements or building footprint

The user can hand-edit `reference.json` after extraction to adjust polygons, restructure the zone tree, add custom zones, or fix edge cases.

---

## Output Directory Structure

The directory tree mirrors the zone tree. Zone names are sanitized for filesystem safety (spaces to underscores, special characters removed).

```
output/
  tileset.json                    # root federation tileset
  {model_name}/
    tileset.json                  # per-model tileset
    {zone_path}/
      {IfcClass}.glb              # one GLB per class per zone
    _unzoned/
      {IfcClass}.glb              # elements outside all zones
```

`{zone_path}` reflects the zone tree nesting:
- Flat zones: `Seg_001/IfcBeam.glb`
- Nested zones: `Building_A/Level_0/IfcWall.glb`

Elements assigned to intermediate zones (parent but not any child) get their GLBs at that level:
- `Building_A/IfcColumn.glb` — column assigned to building but not any specific level

### Example: Building

```
output/
  tileset.json
  architectural/
    tileset.json
    Building_A/
      Level_0/
        IfcWall.glb
        IfcSlab.glb
        IfcDoor.glb
      Level_1/
        IfcWall.glb
        IfcSlab.glb
      IfcColumn.glb              # spans levels, assigned to Building_A
    _unzoned/
      IfcSite.glb
```

### Example: Infrastructure

```
output/
  tileset.json
  road_model/
    tileset.json
    Seg_001/
      IfcBeam.glb
      IfcColumn.glb
    Seg_002/
      IfcBeam.glb
      IfcColumn.glb
    _unzoned/
      IfcSlab.glb
```

### Naming conventions

| Token | Source |
|-------|--------|
| `{model_name}` | Filename stem of input IFC, or `--model-name` override |
| `{zone_path}` | Sanitized zone names joined by `/`, mirroring zone tree depth |
| `{IfcClass}` | IFC type name, e.g. `IfcWall`, `IfcSlab` |
| `_unzoned` | Reserved directory for elements outside all zones |

---

## Tileset.json Structure

### Root tileset (federation)

```json
{
  "asset": {
    "version": "1.1",
    "generator": "ifc-lite-headless"
  },
  "schema": {
    "id": "ifc-lite-schema",
    "classes": {
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
    }
  },
  "geometricError": 100.0,
  "root": {
    "boundingVolume": { "box": [ /* 12 floats */ ] },
    "geometricError": 100.0,
    "refine": "ADD",
    "children": [
      {
        "boundingVolume": { "box": [ /* ... */ ] },
        "geometricError": 50.0,
        "metadata": {
          "class": "Model",
          "properties": {
            "name": "architectural",
            "source": "architectural.ifc"
          }
        },
        "content": {
          "uri": "architectural/tileset.json"
        }
      }
    ]
  }
}
```

### Per-model tileset

The tileset tree mirrors the zone tree. Each zone becomes a tile node, each (zone, class) group becomes a leaf tile.

```json
{
  "asset": { "version": "1.1" },
  "geometricError": 50.0,
  "root": {
    "boundingVolume": { "box": [ /* ... */ ] },
    "geometricError": 50.0,
    "refine": "ADD",
    "children": [
      {
        "boundingVolume": { "box": [ /* ... */ ] },
        "geometricError": 10.0,
        "metadata": {
          "class": "Zone",
          "properties": {
            "name": "Building A",
            "path": "Building_A",
            "depth": 0
          }
        },
        "children": [
          {
            "boundingVolume": { "box": [ /* ... */ ] },
            "geometricError": 5.0,
            "metadata": {
              "class": "Zone",
              "properties": {
                "name": "Level 0",
                "path": "Building_A/Level_0",
                "depth": 1
              }
            },
            "children": [
              {
                "boundingVolume": { "box": [ /* ... */ ] },
                "geometricError": 0.0,
                "metadata": {
                  "class": "IfcClassGroup",
                  "properties": {
                    "ifcClass": "IfcWall",
                    "zone": "Building_A/Level_0",
                    "model": "architectural",
                    "elementCount": 42
                  }
                },
                "content": {
                  "uri": "Building_A/Level_0/IfcWall.glb"
                }
              }
            ]
          },
          {
            "boundingVolume": { "box": [ /* ... */ ] },
            "geometricError": 0.0,
            "metadata": {
              "class": "IfcClassGroup",
              "properties": {
                "ifcClass": "IfcColumn",
                "zone": "Building_A",
                "model": "architectural",
                "elementCount": 8
              }
            },
            "content": {
              "uri": "Building_A/IfcColumn.glb"
            }
          }
        ]
      },
      {
        "boundingVolume": { "box": [ /* ... */ ] },
        "geometricError": 0.0,
        "metadata": {
          "class": "Zone",
          "properties": {
            "name": "_unzoned",
            "path": "_unzoned",
            "depth": 0
          }
        },
        "children": [ /* leaf tiles for unzoned classes */ ]
      }
    ]
  }
}
```

---

## Bounding Volumes

All bounding volumes use the 3D Tiles `box` format: 12 floats — center (3) + x half-axis (3) + y half-axis (3) + z half-axis (3). Phase 1 uses AABB, so half-axes are `[hx,0,0], [0,hy,0], [0,0,hz]`.

Computed bottom-up:
1. **Leaf tile (class GLB)**: union AABB of all element meshes in that GLB
2. **Zone tile**: union AABB of all child tiles (zones + class groups at this level)
3. **Model tile**: union AABB of all top-level zone tiles + _unzoned
4. **Root tile**: union AABB of all model tiles

---

## Metadata-Based Culling

### Static culling (tileset generation time)
- Exclude/include flags filter at generation time
- Only tiles with actual geometry are emitted (empty classes/zones omitted)

### Dynamic culling (API time — future phase)
- API endpoint generates tileset.json on the fly
- Query: `GET /tileset.json?models=arch,struct&zones=Building_A/Level_0&classes=IfcWall,IfcSlab`
- Server omits tiles not matching the query — client never sees them
- Works because directory structure is **predictable**: `{model}/{zone_path}/{class}.glb`
- The API only needs to filter the tileset.json tree, not reprocess geometry

### Predictable path convention

```
{output_dir}/{model_name}/{zone_path}/{IfcClassName}.glb
```

An API layer can reconstruct which GLBs exist from:
- The reference.json (zone tree structure and names)
- A manifest of processed models + their classes per zone (written alongside tileset.json)

---

## Processing Pipeline

### Stage 1: `extract-ref`

```
IFC file
  -> Walk IfcRelAggregates from IfcProject down to --leaf-type
  -> For each spatial element in the decomposition tree:
     a. Determine zone polygon (contained elements convex hull / geometry / bbox)
     b. Determine Z range (elevation + height, or contained elements min/max Z)
  -> Build zone tree
  -> Extract coordinate system (units, RTC offset)
  -> Write reference.json
```

### Stage 2: `tileset`

```
Load reference.json

For each --model:
  1. Parse IFC (reuse existing process_ifc pipeline)
  2. For each element (GuidMesh):
     a. Compute AABB centroid
     b. Walk zone tree, find deepest containing zone
     c. Bucket into (zone_path, ifc_class) group
  3. For each (zone_path, ifc_class) group:
     a. Write GLB to {model}/{zone_path}/{class}.glb
        (all elements of that class in that zone, individual nodes preserved)
     b. Compute group AABB
  4. Build per-model tileset.json mirroring zone tree
  5. Write per-model tileset.json

After all models:
  6. Build root tileset.json referencing per-model tilesets
  7. Write root tileset.json
```

### GLB per group

Each group GLB contains all elements of one IFC class in one zone. Elements retain their individual node names (GlobalId by default) and colors. This preserves per-element identification/picking in the client while enabling zone+class level culling.

---

## Rust Architecture

### New modules

```
rust/cli/src/
  tileset/
    mod.rs          # subcommand entry points (extract_ref, tileset)
    reference.rs    # extract-ref: walk spatial tree, compute volumes, write reference.json
    tiler.rs        # tileset: load ref, group elements, write GLBs + tileset.json
    types.rs        # ReferenceFile, Zone, Slice, TileNode, etc. (serde)
    spatial.rs      # centroid calc, point-in-polygon, zone assignment
```

### Key types

```rust
/// A vertical slice of a zone volume
#[derive(Serialize, Deserialize)]
struct Slice {
    polygon: Vec<[f64; 2]>,  // 2D polygon vertices (XY)
    z_min: f64,
    z_max: f64,
}

/// A zone volume: union of vertical slices
#[derive(Serialize, Deserialize)]
struct ZoneVolume {
    slices: Vec<Slice>,
}

/// A node in the zone tree
#[derive(Serialize, Deserialize)]
struct Zone {
    name: String,
    global_id: Option<String>,
    volume: ZoneVolume,
    children: Vec<Zone>,
}

/// Parsed reference file
#[derive(Serialize, Deserialize)]
struct ReferenceFile {
    version: u32,
    source: String,
    zones: Vec<Zone>,           // root-level zones (forest)
    coordinate_system: CoordinateSystem,
}

/// A group of elements sharing (zone_path, ifc_class)
struct TileGroup {
    zone_path: String,        // e.g. "Building_A/Level_0" or "_unzoned"
    ifc_class: String,        // e.g. "IfcWall"
    meshes: Vec<GuidMesh>,
    aabb: [f64; 6],           // min_x, min_y, min_z, max_x, max_y, max_z
}

/// Tileset.json node builder
struct TileNode {
    bounding_volume: [f64; 12],
    geometric_error: f64,
    metadata: Option<TileMetadata>,
    content: Option<TileContent>,
    children: Vec<TileNode>,
}
```

### Reuse

- `process_ifc()` from `processor.rs` → reused as-is to get `Vec<GuidMesh>`
- `write_glb()` from `glb.rs` → reused to write each group's GLB (takes `&[GuidMesh]`)
- Core parser, geometry engine, GLB writer unchanged

### Spatial utilities (new)

```rust
/// 2D point-in-polygon via ray casting
fn point_in_polygon(point: [f64; 2], polygon: &[[f64; 2]]) -> bool;

/// Test if a 3D point is inside a zone volume (any slice)
fn point_in_zone_volume(point: [f64; 3], volume: &ZoneVolume) -> bool;

/// Walk zone tree, return path of deepest containing zone
fn assign_to_zone(centroid: [f64; 3], zones: &[Zone]) -> Option<String>;

/// Compute AABB centroid from mesh positions
fn mesh_centroid(positions: &[f32]) -> [f64; 3];
```

---

## Future Phases

### Phase 2: Property-based grouping
- Parse `IfcRelDefinesByProperties` -> `IfcPropertySet` -> `IfcPropertySingleValue`
- Group by arbitrary property values (e.g., "Phase", "System", "FireRating")
- Adds a `--group-by-property <pset>.<prop>` flag

### Phase 3: LOD generation
- AABB / OBB / convex hull substitution for distant tiles
- `geometricError` becomes meaningful (>0 on parent tiles)
- Parent tiles get simplified geometry, children have full detail

### Phase 4: Dynamic API server
- `ifc-lite-headless serve --data <output_dir/> --port 8080`
- `GET /tileset.json?models=...&zones=...&classes=...`
- Filters tileset.json tree based on query params
- Serves GLBs from the predictable directory structure

### Future: `--grid <meters>`
- Auto-generate a regular grid of zones along the longest model axis
- Useful for infrastructure without explicit spatial structure in the IFC

---

## Constraints & Decisions

| Decision | Rationale |
|----------|-----------|
| Separate subcommands | Preserve IfcConvert CLI compat for existing convert path |
| Generic zones (not storeys) | Works for buildings, infrastructure, arbitrary spatial partitioning |
| Polygonal extrusion volumes | Handles non-axis-aligned zones; 2D point-in-polygon is fast and well-understood |
| Multi-slice volumes | Handles vertical L-shapes, setbacks without full 3D mesh volumes |
| Deepest-match assignment | Natural for nested zones; parent catches elements that span children |
| One ref = one grouping dimension | Avoids overlap ambiguity; multiple dimensions = multiple runs |
| Reference.json as intermediate | Extract once, tile many times; hand-editable for custom zones |
| Centroid containment | Simple, deterministic; handles cross-model mapping without shared relationships |
| One GLB per (zone, class) | Granular enough for culling, coarse enough to avoid file explosion |
| Metadata in tileset.json tiles | 3D Tiles 1.1 native; no extra files; CesiumJS reads it natively |
| `_unzoned` for unassigned | Explicit handling of elements outside all zones; client gets complete data |
| AABB for tile bounding volumes | Phase 1 simplicity; OBB/convex hull is Phase 3 |
| `geometricError: 0` on leaves | No LOD in Phase 1; all tiles load at full detail |
| Predictable paths | Enables future API to filter tileset.json without reprocessing geometry |
| Directory mirrors zone tree | Intuitive; supports arbitrary nesting depth |
