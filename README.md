# ifc-lite-headless

Headless IFC processing toolkit built on [ifc-lite](https://github.com/louistrue/ifc-lite) Rust crates.

## Features

### GLB Export (`glb`)
Converts IFC to GLB 2.0 with each node named by its IFC GlobalId.

```bash
cargo build --release
./target/release/ifc-lite-headless glb model.ifc -o model.glb
```

- `node.name` = IFC GlobalId (e.g., `0kENwEoa5AkQagyavDrmx3`)
- `node.extras.ifcType` = IFC type (e.g., `IfcWall`)
- PBR materials with IFC style colors
- Parallel geometry processing

## Benchmark vs IfcConvert 0.8.4

```bash
python3 docker/benchmark.py
```

| File | ifc-lite-headless | IfcConvert 0.8.4 | Speedup |
|------|-------------------|------------------|---------|
| ARK hus A (12.6MB) | 0.49s | 20.48s | ~42x |
| File (1) (265KB) | 0.22s | 0.80s | ~4x |

*IfcConvert running under Rosetta (amd64 on ARM Mac). Native amd64 gap would be smaller.*

## License

MPL-2.0
