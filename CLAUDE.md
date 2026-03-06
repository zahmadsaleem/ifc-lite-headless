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
./target/release/ifc-lite-headless input.ifc output.glb [options]
```

IfcConvert-compatible CLI. See `--help` for all flags or `IFCCONVERT_COMPAT.md` for the compatibility checklist.

## Benchmark
```
python3 docker/benchmark.py
```

## Key Design Decisions
- IfcConvert-compatible CLI (positional args, same flags)
- Usable as both CLI and library (`config::ConvertConfig` + `processor::process_ifc`)
- Default excludes: IfcOpeningElement, IfcSpace (matches IfcConvert)
- Node names configurable: GlobalId (default), Name, StepId, Type
- Z-up default (IfcConvert-compatible); `--y-up` for glTF-spec compliance
- Colors extracted from IFC styled items, with type-based defaults
- Parallel geometry processing via rayon
- Nightly toolchain required (csgrs dependency uses edition2024)
