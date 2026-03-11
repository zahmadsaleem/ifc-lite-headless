# LOD & Visibility Culling Plan

## Current State (post Phase 1+2)

- **3d-tiles-renderer**: integrated — frustum culling, SSE-based loading, LRU memory budget
- **geometricError**: computed from bounding sphere radii in Rust (test model regenerated)
- **gltfUpAxis: "z"**: set in all generated tilesets — prevents the 3d-tiles-renderer from applying a Y-up rotation to Z-up IFC content
- **UnloadTilesPlugin**: 600MB budget, 2s delay before disposal (prevents thrash)
- **Large tilesets** (`large_tilesets/`): still have old hardcoded gE values — need regeneration
- **03058 models**: B-290 (current), V-360 and T-760 also available, need multi-model tileset

---

## Bugs Fixed (not Phase 3)

### Z-up became Y-up
**Cause**: `TilesRenderer` defaults `gltfUpAxis` to `'y'` if not set in `tileset.json asset`. With `'y'`, it applies a 90° X-axis rotation to every loaded GLB, converting Y-up → Z-up. Since our GLBs are already Z-up (IFC convention), this rotation was wrong.
**Fix**: Added `"gltfUpAxis": "z"` to `TilesetAsset` in Rust. With `'z'`, the switch falls through to identity — no rotation applied.

### Chrome stuck (memory thrash)
**Cause**: `UnloadTilesPlugin` unloads 100% of excess at exactly the threshold with no delay. Camera nudge triggers full reload cycle.
**Fix**: `bytesTarget: 600MB`, `delay: 2000ms` — tiles stay alive for 2s after leaving frustum, preventing immediate reload.

---

## Phase 3: LOD Proxy Geometry

**Goal**: Generate simplified stand-in geometry for zone tiles so distant zones render *something* instead of nothing. This completes the LOD hierarchy: proxy at distance → full detail when close.

### Proxy type by element profile

Two proxy strategies based on geometry aspect ratio:

#### Rod-like proxy (lines)
**When**: element AABB has one axis significantly longer than the others — aspect ratio `max_dim / mid_dim > 10`
**Target classes**: `IfcReinforcingBar`, `IfcTendon`, `IfcCableCarrierFitting`, `IfcPipeSegment`, `IfcCableSegment`, thin members
**Proxy**: A single line segment from the AABB min-center to max-center along the dominant axis
**Size**: 2 vertices, 1 line = negligible GPU cost
**GLB encoding**: `GL_LINES` primitive (mode=1) in GLTF
**Example**: 6607 HAV0 rebars → 6607 line segments instead of 6607 full meshes

#### OBB proxy (solid box)
**When**: element is roughly equidimensional (not rod-like)
**Target classes**: `IfcWall`, `IfcSlab`, `IfcBeam`, `IfcColumn`, `IfcDoor`, `IfcWindow`, all other solid types
**Proxy**: Oriented Bounding Box — tighter fit than AABB, better silhouette at distance
**OBB computation**: PCA on vertex positions to find principal axes; or simplified: for single-class groups, use the convex hull of AABB corners rotated to the dominant axis direction
**Fallback**: If PCA is too expensive per-element, use AABB (always correct, just looser)

#### Performance budget for OBB
- AABB: O(n) scan, ~0.1µs per element → negligible
- PCA on raw vertices: O(n) in vertex count, covariance matrix computation → ~1-5µs per element for typical IFC meshes
- For 40k elements: PCA ≈ 40-200ms total → acceptable at tileset generation time
- **Check**: measure actual time on 03058 B model before committing to PCA. If >1s, use AABB-derived OBB (rotate AABB to longest axis of element).

### Zone proxy GLB structure

Each zone tile gets a proxy GLB containing:
- Rod-like elements → `GL_LINES` primitives, one per element
- Solid elements → `GL_TRIANGLES` box primitives (12 tris per OBB), one per element

The proxy GLB for a zone is much smaller than the full detail:
- 6607 HAV0 rebars: full GLB ~55MB → proxy GLB ~200KB (6607 line segments)
- 87 IfcWall Plan_U1: full GLB ~1.7MB → proxy GLB ~50KB (87 OBB boxes)

### Tileset changes (Rust)

Zone tiles change from **no content** to **content pointing at proxy GLB**:
```json
{
  "geometricError": 52.4,
  "refine": "REPLACE",
  "content": { "uri": "D0/HAV0/_proxy.glb" },
  "children": [
    { "geometricError": 0.0, "content": { "uri": "D0/HAV0/IfcReinforcingBar.glb" } }
  ]
}
```

With `refine: "REPLACE"` (changed from no refine / ADD default), the renderer either shows the proxy **or** the children — never both. When SSE drops below `geometricError` (zone is small on screen), proxy renders. When SSE exceeds it (camera close), children load.

### Implementation steps

**3a. Detect rod-like vs solid per-element** (in `tiler.rs` during group construction):
```rust
fn element_proxy_type(aabb: &Aabb) -> ProxyType {
    let dims = sorted_dims(aabb); // [min, mid, max]
    if dims[2] / dims[1] > 10.0 { ProxyType::Line } else { ProxyType::Obb }
}
```

**3b. Aggregate proxy geometry per zone** (one proxy GLB per zone tile):
- For each group in the zone, compute its proxy primitive (line or OBB box)
- Combine all into one proxy GLB with two mesh primitives:
  - One `GL_LINES` primitive for all rods (positions buffer: 2 verts × N rods)
  - One `GL_TRIANGLES` primitive for all solids (12 tris × M solids)

**3c. Write proxy GLB** (new function in `glb.rs` or proxy module):
- Lines: flat list of (start, end) pairs
- Boxes: 8 corners × M boxes → indexed triangles

**3d. Set zone tile `refine: "REPLACE"` and `content` pointing to proxy GLB**

**3e. Adjust `geometricError`** on zone tiles so proxies load at an appropriate distance. Current heuristic (bounding sphere radius) is correct — no change needed.

---

## Phase 3b: OBB quality check

After Phase 3a is working, benchmark AABB vs PCA-OBB visually and by generation time:
- Generate test tileset with `--proxy-obb=aabb` vs `--proxy-obb=pca`
- Compare proxy GLB sizes and visual quality at distance
- If PCA generates tighter fits with acceptable time (<500ms on 03058), keep it
- Otherwise fall back to axis-aligned AABB boxes

---

## 03058 Multi-Model Tileset

The 03058 project has 3 IFC models:
| File | Code | Content |
|------|------|---------|
| `03058-D0-00-00-B-290-IF-001.ifc` | B-290 | Reinforcement (current) |
| `03058-D0-00-00-V-360-IF-002.ifc` | V-360 | TBD |
| `03058-00-00-00-T-760-IF-003.ifc` | T-760 | TBD |

Steps to include all three:
1. Run `extract-ref` on the model that contains the spatial structure (zones/storeys) — likely T-760 or B-290
2. Run `tileset` with all three `--model` flags:
   ```
   ifc-lite-headless tileset --ref 03058_ref.json \
     --model B-290.ifc --model-name B \
     --model V-360.ifc --model-name V \
     --model T-760.ifc --model-name T \
     -o large_tilesets/03058_tileset
   ```
3. Add a `task 03058` target in Taskfile.yml

---

## Phase 4: Memory Budget & Polish (deferred)

- Auto-detect device memory for `bytesTarget`
- Memory stats in UI: `GPU: 127MB / 600MB | 12/48 tiles | 6 downloading`
- `TilesFadePlugin` for smooth tile transitions
- `V.quality("high"/"medium"/"low")` presets — already implemented

---

## Implementation Order

| Step | What | Key files |
|------|------|-----------|
| ✅ Phase 1 | 3d-tiles-renderer integration | `viewer/index.html` |
| ✅ Phase 2 | geometricError from bounding sphere | `crates/cli/src/tileset/tiler.rs`, `types.rs` |
| ✅ Fix | gltfUpAxis: "z" in tileset asset | `types.rs`, `tiler.rs` |
| ✅ Fix | Chrome memory thrash (600MB, 2s delay) | `viewer/index.html` |
| Next | Phase 3a: rod/OBB proxy GLBs | `tiler.rs`, `glb.rs` (new proxy module) |
| After | 03058 multi-model Taskfile target | `Taskfile.yml` |
| After | Phase 3b: OBB quality benchmark | `tiler.rs` |
| Deferred | Phase 4: polish | `viewer/index.html` |

---

## Key Design Decisions

1. **Lines for rods, OBB for solids**: avoids inflating rebar proxies into boxes that would be meaningless at distance and waste GPU budget
2. **One proxy GLB per zone** (not per class): reduces file count and HTTP requests; mixed primitive types in one GLB is valid GLTF
3. **REPLACE refinement on zone tiles**: critical for correct LOD — proxy shows at distance, full detail replaces when close; ADD would show both simultaneously
4. **AABB-first OBB**: start with AABB-derived boxes, benchmark PCA before committing
5. **geometricError = bounding sphere radius** stays unchanged for zone tiles — the proxy distance threshold is already correct
