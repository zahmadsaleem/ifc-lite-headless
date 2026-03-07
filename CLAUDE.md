# ifc-lite-headless

Headless IFC processing toolkit. First feature: IFC to GLB conversion with GUID node names.

## Folder Philosophy
- Code lives in `crates/`, docs live next to the code they describe (`crates/cli/docs/`)
- Top-level only has project-wide files (README, Cargo.toml, CLAUDE.md, CI)
- No sprawling root docs — scope docs to the crate they belong to

## Project Structure
- `crates/core` - IFC/STEP parser (ifc-lite-core)
- `crates/geometry` - Geometry processing, mesh generation (ifc-lite-geometry)
- `crates/cli` - CLI binary with subcommands (ifc-lite-headless)
- `crates/cli/docs/` - CLI-scoped docs (IFCCONVERT_COMPAT, TILESET_SPEC, TILESET_CHECKLIST)
- `bench/` - Benchmark infra and results (vs IfcConvert 0.8.4)
- `viewer/` - 3D Tiles viewer (bun + three.js)
- `.private/` - Test IFC files (gitignored)

## Build & Run
```
cargo build --release
./target/release/ifc-lite-headless input.ifc output.glb [options]
```

IfcConvert-compatible CLI. See `--help` for all flags or `crates/cli/docs/IFCCONVERT_COMPAT.md` for the compatibility checklist.

## Taskfile
Uses [Task](https://taskfile.dev) for common workflows:
- `task build` - Build release binary
- `task test` - Run all tests
- `task convert -- input.ifc output.glb` - Convert single IFC to GLB
- `task pipeline` - Full 3D Tiles pipeline (extract-ref + tileset)
- `task dev` - Build, generate tileset, and start viewer
- `task viewer:dev` - Start the 3D Tiles viewer dev server
- `task clean` - Clean build artifacts and test outputs

## Key Design Decisions
- IfcConvert-compatible CLI (positional args, same flags)
- Usable as both CLI and library (`config::ConvertConfig` + `processor::process_ifc`)
- Default excludes: IfcOpeningElement, IfcSpace (matches IfcConvert)
- Node names configurable: GlobalId (default), Name, StepId, Type
- Z-up default (IfcConvert-compatible); `--y-up` for glTF-spec compliance
- Colors extracted from IFC styled items, with type-based defaults
- Parallel geometry processing via rayon
- Nightly toolchain required (csgrs dependency uses edition2024)

## Visual Debugging Workflow
When working on 3D geometry, use this feedback loop:

1. Edit code → `cargo build --release && task pipeline`
2. Reload viewer in Chrome via MCP tools
3. Inspect with `window.V` API — isolate by GUID, set camera, take screenshots
4. Compare with reference screenshots from user
5. Iterate

The viewer runs at localhost:3000 (`task viewer:dev`). `window.V` has methods like `find()`, `isolate()`, `showAll()`, `zoomToFit()`, `setCamera()`, `stats()`, `inspect()`, `highlight()`, `wireframe()`, `xray()`. Use Chrome MCP tools (javascript_tool, computer screenshot) — no mouse needed.
