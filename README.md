# ifc-lite-headless

Headless IFC processing toolkit built on [ifc-lite](https://github.com/louistrue/ifc-lite) Rust crates.

## Features

### GLB Export
Converts IFC to GLB 2.0 with each node named by its IFC GlobalId.

```bash
cargo build --release
./target/release/ifc-lite-headless input.ifc output.glb [options]
```

- IfcConvert-compatible CLI (see `--help`)
- `node.name` = IFC GlobalId (default), Name, StepId, or Type
- PBR materials with IFC style colors
- Parallel geometry processing via rayon

### 3D Tiles Pipeline
Generates 3D Tiles 1.1 tilesets from IFC models, spatially partitioned by zone.

```bash
# Extract spatial reference from a zones IFC
./target/release/ifc-lite-headless extract-ref zones.ifc -o reference.json

# Generate tileset from model against reference
./target/release/ifc-lite-headless tileset --ref reference.json --model model.ifc -o output/
```

- Generic zone-based spatial partitioning (not limited to storeys)
- One GLB per zone per IFC class
- 3D Tiles 1.1 native tile metadata
- See [`crates/cli/docs/TILESET_SPEC.md`](crates/cli/docs/TILESET_SPEC.md) for full spec

### Viewer
Built-in 3D Tiles viewer for inspecting tileset output.

```bash
task dev  # build, generate tileset, start viewer
```

## Task Runner

Uses [Task](https://taskfile.dev). Key commands:

| Command | Description |
|---------|-------------|
| `task build` | Build release binary |
| `task test` | Run all tests |
| `task convert -- input.ifc output.glb` | Convert single IFC to GLB |
| `task pipeline` | Full 3D Tiles pipeline (extract-ref + tileset) |
| `task dev` | Build, generate tileset, start viewer |
| `task clean` | Clean build artifacts and test outputs |

## Benchmark vs IfcConvert 0.8.4

```bash
python3 bench/benchmark.py > bench/benchmark_results.csv
```

**30 files tested (0.3 MB - 652 MB), 43x average speedup:**

| Size range | Files | Avg speedup |
|------------|-------|-------------|
| < 1 MB     | 8     | 13x         |
| 1 - 20 MB  | 11    | 63x         |
| 20 - 70 MB | 7     | 39x         |
| 300+ MB    | 4     | 35x         |

See [`bench/benchmark_results.csv`](bench/benchmark_results.csv) for full results with entity counts.

## License

MPL-2.0
