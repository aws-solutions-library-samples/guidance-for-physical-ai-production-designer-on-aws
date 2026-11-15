use anyhow::Result;
use std::io::Write;
use std::path::Path;

use kuka_krl_parser::types::*;

pub fn write_movement_yaml(program: &MovementProgram, output_path: &Path) -> Result<()> {
    if let Some(parent) = output_path.parent() {
        std::fs::create_dir_all(parent)?;
    }

    let mut f = std::fs::File::create(output_path)?;

    writeln!(f, "program_name: '{}'", program.program_name)?;
    writeln!(f, "frames:")?;

    // Bases
    writeln!(f, "  bases:")?;
    for (idx, frame) in &program.frames.bases {
        let name_part = if frame.name.is_empty() {
            if *idx == 0 {
                "WORLD".to_string()
            } else {
                format!("Base {}", idx)
            }
        } else {
            frame.name.clone()
        };
        writeln!(
            f,
            "    {}: {{x: {}, y: {}, z: {}, a: {}, b: {}, c: {}, name: {}}}",
            idx,
            format_f64(frame.x),
            format_f64(frame.y),
            format_f64(frame.z),
            format_f64(frame.a),
            format_f64(frame.b),
            format_f64(frame.c),
            name_part
        )?;
    }

    // Tools
    writeln!(f, "  tools:")?;
    for (idx, frame) in &program.frames.tools {
        let name_part = if frame.name.is_empty() {
            format!("Tool {}", idx)
        } else {
            frame.name.clone()
        };
        writeln!(
            f,
            "    {}: {{x: {}, y: {}, z: {}, a: {}, b: {}, c: {}, name: {}}}",
            idx,
            format_f64(frame.x),
            format_f64(frame.y),
            format_f64(frame.z),
            format_f64(frame.a),
            format_f64(frame.b),
            format_f64(frame.c),
            name_part
        )?;
    }

    // Points
    writeln!(f, "points:")?;
    for (name, point) in &program.points {
        writeln!(f, "  {}:", name)?;
        writeln!(
            f,
            "    coords: [{}, {}, {}]",
            format_f64(point.coords[0]),
            format_f64(point.coords[1]),
            format_f64(point.coords[2])
        )?;
        writeln!(
            f,
            "    ori: [{}, {}, {}]",
            format_f64(point.ori[0]),
            format_f64(point.ori[1]),
            format_f64(point.ori[2])
        )?;
        writeln!(
            f,
            "    params: {{tool: {}, base: {}, vb: {}, ve: {}, motion: {}, s: {}, t: {}}}",
            point.params.tool,
            point.params.base,
            format_f64(point.params.vb),
            format_f64(point.params.ve),
            point.params.motion,
            point.params.s,
            point.params.t
        )?;
        writeln!(f, "    desc: {}", point.desc)?;
    }

    // Sequence
    write!(f, "sequence: [")?;
    let seq_str: Vec<String> = program.sequence.iter().map(|s| s.to_string()).collect();
    write!(f, "{}", seq_str.join(", "))?;
    writeln!(f, "]")?;

    // I/O operations
    if !program.ios.is_empty() {
        writeln!(f, "ios:")?;
        for (i, io) in program.ios.iter().enumerate() {
            writeln!(f, "  - index: {}", i)?;
            writeln!(f, "    type: {}", io.io_type)?;
            writeln!(f, "    channel: {}", io.channel)?;
            writeln!(f, "    action: {}", io.action)?;
            if let Some(ref name) = io.signal_name {
                writeln!(f, "    signal_name: {}", name)?;
            }
            writeln!(f, "    context: '{}'", io.context.replace('\'', "''"))?;
        }
    }

    Ok(())
}

fn format_f64(v: f64) -> String {
    if v == v.trunc() && v.abs() < 1e10 {
        format!("{:.1}", v)
    } else {
        let s = format!("{}", v);
        if !s.contains('.') {
            format!("{}.0", s)
        } else {
            s
        }
    }
}
