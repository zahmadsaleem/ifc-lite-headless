# IfcConvert Compatibility Checklist

Drop-in replacement CLI for IfcConvert (v0.8.0), focused on GLB output.

## CLI Pattern

```
ifc-lite-headless input.ifc output.glb [options]
```

Output format inferred from file extension (currently only .glb supported).

## Phase 1 - Core flags

- [x] Positional input/output args (drop subcommand)
- [x] `--include <types...>` - include only matching IFC types
- [x] `--include+ <types...>` - include + decomposition/containment (flag parsed; recursive logic TBD)
- [x] `--exclude <types...>` - exclude matching IFC types (default: IfcOpeningElement IfcSpace)
- [x] `--exclude+ <types...>` - exclude + decomposition/containment (flag parsed; recursive logic TBD)
- [x] `--use-element-guids` - name nodes by GlobalId (default)
- [x] `--use-element-names` - name nodes by IfcRoot.Name
- [x] `--use-element-step-ids` - name nodes by STEP #id
- [x] `--use-element-types` - name nodes by IFC type
- [x] `--y-up` - set Y as up axis (default: Z-up, IfcConvert-compatible)
- [x] `-j, --threads <N>` - thread count (default: all cores via rayon)
- [x] `-v, --verbose` - increase verbosity
- [x] `-q, --quiet` - suppress output
- [x] `--no-progress` - suppress progress bar

## Phase 2 - Geometry transforms

- [ ] `--center-model` - center by placement offset
- [ ] `--center-model-geometry` - center by mesh vertex center
- [ ] `--model-offset <x;y;z>` - arbitrary translation
- [ ] `--weld-vertices` - merge duplicate vertices
- [ ] `--use-world-coords` - bake transforms into vertices
- [ ] `--no-normals` - skip normal computation
- [ ] `--generate-uvs` - box-projected texture coords
- [ ] `--disable-opening-subtractions` - skip boolean void cuts
- [ ] `--convert-back-units` - output in original IFC units
- [ ] `--site-local-placement` - place in IfcSite coords
- [ ] `--building-local-placement` - place in IfcBuilding coords

## Phase 3 - Polish

- [ ] `--element-hierarchy` - nest nodes under storeys
- [ ] `--force-space-transparency <val>` - override IfcSpace alpha
- [ ] `--digits <N>` - floating-point precision
- [ ] `--log-file <path>` - redirect log output
- [ ] `-y, --yes` - auto-confirm overwrite
- [ ] `--version` - display version

## Won't Support

These are not applicable to our geometry engine or GLB output:

- `--kernel` - we have our own engine
- `--plan` / `--model` - representation type selection
- All `--svg-*` flags - SVG only
- `--bounds`, `--scale`, `--center` - SVG only
- `--section-*`, `--elevation-*` - SVG only
- `--door-arcs`, `--print-space-*` - SVG only
- `--cache`, `--cache-file` - no cache system
- `--mesher-*-deflection` - OpenCASCADE specific
- `--reorient-shells`, `--unify-shapes` - OCC specific
- `--enable-layerset-slicing`, `--layerset-first` - advanced material slicing
- `--disable-boolean-result` - OCC specific
- `--validate` - quantity validation
- `--edge-arrows` - edge visualization
- `--filter-file` - file-based filters (maybe later)
- `--default-material-file` - external materials
- `--context-ids/types/identifiers` - representation context
- `--use-material-names` - DAE specific
- `--exterior-only` - complex geometric analysis
- `--base-uri`, `--wkt-*` - RDF/TTL only
- `--mmap` - memory-mapped input
- `--cgal-*`, `--circle-segments` - CGAL specific
- `--model-rotation` - quaternion rotation (maybe later)
- `--precision-factor`, `--no-wire-intersection-*` - OCC specific
- `--ecef` - Earth-centered coords
- `--separate-z-up-node` - glTF hierarchy tweak
- `--debug` - OCC boolean debug files
- `--calculate-quantities` - IFC quantities
- `--dimensionality`, `--triangulation-type` - OCC specific
