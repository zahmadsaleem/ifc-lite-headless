// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use clap::Parser;
use std::path::PathBuf;

pub mod config;
mod glb;
mod processor;

use config::{ConvertConfig, ElementNaming, EntityFilter, FilterMode, UpAxis, Verbosity};

#[derive(Parser)]
#[command(
    name = "ifc-lite-headless",
    about = "IfcConvert-compatible IFC geometry converter",
    version
)]
struct Cli {
    /// Input IFC file
    input: PathBuf,

    /// Output file (format inferred from extension; default: <input>.glb)
    output: Option<PathBuf>,

    /// Include only entities of these IFC types
    #[arg(long, num_args = 1.., value_delimiter = ' ')]
    include: Option<Vec<String>>,

    /// Include entities and their decomposition/containment hierarchy
    #[arg(long = "include+", num_args = 1.., value_delimiter = ' ', id = "include_plus")]
    include_plus: Option<Vec<String>>,

    /// Exclude entities of these IFC types [default: IfcOpeningElement IfcSpace]
    #[arg(long, num_args = 1.., value_delimiter = ' ')]
    exclude: Option<Vec<String>>,

    /// Exclude entities and their decomposition/containment hierarchy
    #[arg(long = "exclude+", num_args = 1.., value_delimiter = ' ', id = "exclude_plus")]
    exclude_plus: Option<Vec<String>>,

    /// Name elements by IfcRoot.GlobalId
    #[arg(long)]
    use_element_guids: bool,

    /// Name elements by IfcRoot.Name
    #[arg(long)]
    use_element_names: bool,

    /// Name elements by STEP numeric id
    #[arg(long)]
    use_element_step_ids: bool,

    /// Name elements by IFC type
    #[arg(long)]
    use_element_types: bool,

    /// Set Y as up axis (default: Z-up)
    #[arg(long)]
    y_up: bool,

    /// Number of processing threads (default: all cores)
    #[arg(short = 'j', long = "threads")]
    threads: Option<usize>,

    /// Increase log verbosity (use -vv for debug)
    #[arg(short, long, action = clap::ArgAction::Count)]
    verbose: u8,

    /// Suppress status and progress output
    #[arg(short, long)]
    quiet: bool,

    /// Suppress progress bar
    #[arg(long)]
    no_progress: bool,

    /// Deflection tolerance for curved geometry in meters [default: 0.001]
    #[arg(long)]
    deflection: Option<f64>,
}

impl Cli {
    fn into_config(&self) -> ConvertConfig {
        let mut config = ConvertConfig::default();

        // Include filters
        if let Some(ref types) = self.include {
            config.include.push(EntityFilter {
                mode: FilterMode::Direct,
                types: normalize_types(types),
            });
        }
        if let Some(ref types) = self.include_plus {
            config.include.push(EntityFilter {
                mode: FilterMode::Recursive,
                types: normalize_types(types),
            });
        }

        // Exclude filters - replace defaults if user provides any
        if self.exclude.is_some() || self.exclude_plus.is_some() {
            config.exclude.clear();
        }
        if let Some(ref types) = self.exclude {
            config.exclude.push(EntityFilter {
                mode: FilterMode::Direct,
                types: normalize_types(types),
            });
        }
        if let Some(ref types) = self.exclude_plus {
            config.exclude.push(EntityFilter {
                mode: FilterMode::Recursive,
                types: normalize_types(types),
            });
        }

        // Element naming (last one wins, matching IfcConvert behavior)
        if self.use_element_names {
            config.naming = ElementNaming::Names;
        }
        if self.use_element_step_ids {
            config.naming = ElementNaming::StepIds;
        }
        if self.use_element_types {
            config.naming = ElementNaming::Types;
        }
        if self.use_element_guids {
            config.naming = ElementNaming::Guids;
        }

        if self.y_up {
            config.up_axis = UpAxis::Y;
        }

        if let Some(t) = self.threads {
            config.threads = t;
        }

        if self.quiet {
            config.verbosity = Verbosity::Quiet;
        } else if self.verbose > 0 {
            config.verbosity = Verbosity::Verbose;
        }

        config.no_progress = self.no_progress;
        if let Some(d) = self.deflection {
            config.deflection = d;
        }

        config
    }
}

fn normalize_types(types: &[String]) -> Vec<String> {
    types
        .iter()
        .map(|t| {
            let upper = t.to_uppercase();
            if upper.starts_with("IFC") {
                upper
            } else {
                format!("IFC{}", upper)
            }
        })
        .collect()
}

fn main() {
    let cli = Cli::parse();
    let config = cli.into_config();

    // Configure thread pool
    if config.threads > 0 {
        rayon::ThreadPoolBuilder::new()
            .num_threads(config.threads)
            .build_global()
            .ok();
    }

    let input = &cli.input;
    let output = cli.output.clone().unwrap_or_else(|| {
        let stem = input.file_stem().unwrap_or_default();
        PathBuf::from(format!("{}.glb", stem.to_string_lossy()))
    });

    // Validate output extension
    let ext = output
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase();

    if ext != "glb" {
        eprintln!(
            "Error: unsupported output format '.{}'. Currently only .glb is supported.",
            ext
        );
        std::process::exit(1);
    }

    // Ensure output directory exists
    if let Some(parent) = output.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).ok();
        }
    }

    run_glb(input, &output, &config);
}

fn run_glb(input: &PathBuf, output: &PathBuf, config: &ConvertConfig) {
    let content = match std::fs::read_to_string(input) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Error reading {}: {}", input.display(), e);
            std::process::exit(1);
        }
    };

    let quiet = config.verbosity == Verbosity::Quiet;

    if !quiet {
        eprintln!("Processing {}...", input.display());
    }
    let start = std::time::Instant::now();

    let mesh_data = processor::process_ifc(&content, config);

    if !quiet {
        let process_time = start.elapsed();
        eprintln!(
            "Extracted {} meshes in {:.2}s",
            mesh_data.len(),
            process_time.as_secs_f64()
        );
    }

    if mesh_data.is_empty() {
        if !quiet {
            eprintln!("Warning: No geometry found in IFC file");
        }
        std::process::exit(0);
    }

    let glb_start = std::time::Instant::now();
    match glb::write_glb(&mesh_data, output, config) {
        Ok(size) => {
            if !quiet {
                let glb_time = glb_start.elapsed();
                eprintln!(
                    "Wrote {} ({} bytes) in {:.2}s",
                    output.display(),
                    size,
                    glb_time.as_secs_f64()
                );
                eprintln!("Total: {:.2}s", start.elapsed().as_secs_f64());
            }
        }
        Err(e) => {
            eprintln!("Error writing GLB: {}", e);
            std::process::exit(1);
        }
    }
}
