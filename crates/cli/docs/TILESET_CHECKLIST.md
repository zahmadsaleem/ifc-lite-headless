# 3D Tiles Pipeline - Implementation Checklist

Tracks progress against `TILESET_SPEC.md`.

## Phase 1: Core Pipeline

### CLI
- [x] `extract-ref` subcommand
- [x] `tileset` subcommand
- [x] Backward-compatible positional args (IfcConvert mode)
- [x] `--ref`, `--model`, `--model-name`, `-o` flags
- [x] `--exclude`, `--include`, `--y-up`, `-j`, `--deflection` on tileset
- [ ] `--root-type` flag on extract-ref (currently auto-detects)
- [ ] `--leaf-type` flag on extract-ref (parsed but unused)
- [ ] `--depth` flag on extract-ref
- [x] `--volume-mode` flag on extract-ref (elevation, bbox, contained)

### Reference Extraction
- [x] Parse IfcBuildingStorey (name, elevation)
- [x] Parse IfcBuilding (wraps storeys in zone tree)
- [x] Parse IfcRelAggregates (spatial decomposition)
- [x] Parse IfcRelContainedInSpatialStructure
- [x] Sort storeys by elevation, infer heights
- [x] Extract unit scale from IfcProject
- [x] Write reference.json
- [x] `elevation` volume mode (large fallback polygon)
- [x] `bbox` volume mode (AABB of contained elements)
- [x] `contained` volume mode (convex hull of contained elements)
- [ ] Extract polygon from IfcSpace geometry (`geometry` volume mode)
- [ ] Handle IfcZone grouping
- [ ] Handle multiple buildings (currently wraps single building)

### Zone Volumes
- [x] Single-slice polygonal extrusion (polygon + z_min/z_max)
- [x] Multi-slice support in types (serde)
- [x] Shorthand deserialization (flat polygon/z_min/z_max)
- [ ] Multi-slice extraction from IFC (vertical L-shapes)

### Zone Assignment
- [x] Centroid calculation from mesh positions
- [x] 2D point-in-polygon (ray casting)
- [x] Point-in-zone-volume (Z range + polygon)
- [x] Deepest-match tree walk
- [x] `_unzoned` for unassigned elements
- [x] Sanitize zone names for filesystem
- [x] Unit tests (5 passing)

### Tileset Generation
- [x] Group elements by (zone_path, ifc_class)
- [x] Write one GLB per group (reuses existing glb::write_glb)
- [x] Directory structure mirrors zone tree
- [x] Per-model tileset.json with zone hierarchy
- [x] Root federation tileset.json
- [x] 3D Tiles 1.1 schema (Model, Zone, IfcClassGroup classes)
- [x] Tile metadata on Zone and IfcClassGroup nodes
- [x] AABB bounding volumes (bottom-up union)
- [x] `geometricError: 0` on leaf tiles
- [x] `refine: ADD` on root
- [x] Multi-model support (--model repeatable)
- [x] Predictable path convention: `{model}/{zone_path}/{class}.glb`

### Viewer
- [x] Bun dev server (serves tileset + static assets)
- [x] Three.js + GLTFLoader (no ECEF/globe issues)
- [x] OrbitControls, Z-up, auto-center on model
- [x] Zone tree sidebar from tileset.json metadata
- [x] URL param filters: `?classes=`, `?zones=`, `?model=`
- [ ] Interactive toggle (checkbox show/hide per class/zone)
- [ ] Element picking (click to inspect GlobalId/properties)

### Build & Dev
- [x] Taskfile.yml with all commands
- [x] `task build` / `task test`
- [x] `task extract-ref` / `task tileset` / `task pipeline`
- [x] `task viewer:dev` / `task dev`
- [x] `task clean` / `task clean:tileset`

## Phase 2: Property-Based Grouping
- [ ] Parse IfcRelDefinesByProperties
- [ ] Parse IfcPropertySet -> IfcPropertySingleValue
- [ ] `--group-by-property <pset>.<prop>` flag
- [ ] Property values in tile metadata

## Phase 3: LOD Generation
- [ ] AABB substitution geometry for parent tiles
- [ ] OBB bounding volumes
- [ ] Convex hull simplification
- [ ] Meaningful `geometricError` values

## Phase 4: Dynamic API Server
- [ ] `ifc-lite-headless serve` subcommand
- [ ] `GET /tileset.json?models=&zones=&classes=` endpoint
- [ ] Server-side tileset.json filtering
- [ ] Manifest of processed models/classes/zones

## Future
- [ ] `--grid <meters>` for auto-generated zone grid
- [ ] Georeferencing (IfcMapConversion -> tileset transform)
- [ ] Multiple buildings in single reference
- [ ] IfcSpace geometry extraction for zone polygons
