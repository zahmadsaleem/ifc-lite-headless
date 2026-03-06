# ifc-lite-headless

Headless IFC processing toolkit. First feature: IFC to GLB conversion with GUID node names.

## Project Structure
- `rust/core` - IFC/STEP parser (ifc-lite-core)
- `rust/geometry` - Geometry processing, mesh generation (ifc-lite-geometry)
- `rust/cli` - CLI binary with subcommands (ifc-lite-headless)
- `.private/` - Test IFC files (gitignored)
- `docker/` - Docker benchmark setup (vs IfcConvert 0.8.4)

## Build & Run
```
cargo build --release
./target/release/ifc-lite-headless glb input.ifc -o output.glb
```

## Benchmark
```
python3 docker/benchmark.py
```

## Key Design Decisions
- Subcommand architecture: `glb` is the first command, more to come
- Node names in GLB = IFC GlobalId (attribute 0 of IfcProduct entities)
- Colors extracted from IFC styled items, with type-based defaults
- Parallel geometry processing via rayon
- Nightly toolchain required (csgrs dependency uses edition2024)
