mod yaml_output;

use kuka_krl_parser::parser;

use anyhow::{Context, Result};
use clap::Parser as ClapParser;
use std::path::PathBuf;

#[derive(ClapParser, Debug)]
#[command(name = "kuka-krl-parser")]
#[command(about = "Parse KUKA KRL projects and generate movement YAML files")]
struct Args {
    /// Path to the KRL project directory (containing .src and .dat files)
    #[arg(short, long)]
    input: PathBuf,

    /// Output directory for generated YAML files
    #[arg(short, long)]
    output: PathBuf,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if !args.input.exists() {
        anyhow::bail!("Input directory does not exist: {}", args.input.display());
    }

    std::fs::create_dir_all(&args.output)
        .with_context(|| format!("Creating output directory: {}", args.output.display()))?;

    println!("Parsing KRL project: {}", args.input.display());

    let programs =
        parser::parse_krl_project(&args.input).with_context(|| "Failed to parse KRL project")?;

    if programs.is_empty() {
        println!("No movement programs found.");
        return Ok(());
    }

    println!("Found {} movement program(s)", programs.len());

    for program in &programs {
        let filename = sanitize_filename(&program.program_name);
        let output_path = args.output.join(format!("{}.yaml", filename));

        yaml_output::write_movement_yaml(program, &output_path)
            .with_context(|| format!("Writing {}", output_path.display()))?;

        println!(
            "  -> {} ({} points)",
            output_path.display(),
            program.points.len()
        );
    }

    println!("Done.");
    Ok(())
}

fn sanitize_filename(name: &str) -> String {
    name.chars()
        .map(|c| {
            if c.is_alphanumeric() || c == '_' || c == '-' {
                c
            } else {
                '_'
            }
        })
        .collect::<String>()
        .to_lowercase()
}
