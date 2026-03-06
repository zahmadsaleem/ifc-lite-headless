// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use clap::{Parser, Subcommand};
use std::path::PathBuf;

mod glb;
mod processor;

#[derive(Parser)]
#[command(name = "ifc-lite-headless", about = "Headless IFC processing toolkit")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Convert IFC to GLB with GUID node names
    Glb {
        /// Input IFC file path
        input: PathBuf,

        /// Output GLB file path (defaults to .out/<input>.glb)
        #[arg(short, long)]
        output: Option<PathBuf>,
    },
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Commands::Glb { input, output } => {
            let output = output.unwrap_or_else(|| {
                let out_dir = PathBuf::from(".out");
                std::fs::create_dir_all(&out_dir).ok();
                let stem = input.file_stem().unwrap_or_default();
                out_dir.join(format!("{}.glb", stem.to_string_lossy()))
            });
            if let Some(parent) = output.parent() {
                std::fs::create_dir_all(parent).ok();
            }
            run_glb(&input, &output);
        }
    }
}

fn run_glb(input: &PathBuf, output: &PathBuf) {
    let content = match std::fs::read_to_string(input) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Error reading {}: {}", input.display(), e);
            std::process::exit(1);
        }
    };

    eprintln!("Processing {}...", input.display());
    let start = std::time::Instant::now();

    let mesh_data = processor::process_ifc(&content);

    let process_time = start.elapsed();
    eprintln!(
        "Extracted {} meshes in {:.2}s",
        mesh_data.len(),
        process_time.as_secs_f64()
    );

    if mesh_data.is_empty() {
        eprintln!("Warning: No geometry found in IFC file");
        std::process::exit(0);
    }

    let glb_start = std::time::Instant::now();
    match glb::write_glb(&mesh_data, output) {
        Ok(size) => {
            let glb_time = glb_start.elapsed();
            eprintln!(
                "Wrote {} ({} bytes) in {:.2}s",
                output.display(),
                size,
                glb_time.as_secs_f64()
            );
            eprintln!("Total: {:.2}s", start.elapsed().as_secs_f64());
        }
        Err(e) => {
            eprintln!("Error writing GLB: {}", e);
            std::process::exit(1);
        }
    }
}
