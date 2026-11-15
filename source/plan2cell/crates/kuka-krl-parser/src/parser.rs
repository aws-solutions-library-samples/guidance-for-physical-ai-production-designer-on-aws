use anyhow::{Context, Result};
use std::collections::BTreeMap;
use std::path::Path;

use crate::types::*;

fn read_file_lossy(path: &Path) -> Result<String> {
    let bytes = std::fs::read(path).with_context(|| format!("Reading {}", path.display()))?;
    match String::from_utf8(bytes.clone()) {
        Ok(s) => Ok(s),
        Err(_) => {
            let (decoded, _, _) = encoding_rs::WINDOWS_1252.decode(&bytes);
            Ok(decoded.into_owned())
        }
    }
}

pub fn parse_system_frames(config_dat_content: &str) -> Frames {
    let (bases_raw, tools_raw) = parse_dat_frames(config_dat_content);
    Frames {
        bases: build_frames_map(bases_raw, true),
        tools: build_frames_map(tools_raw, false),
    }
}

pub fn parse_krl_project(project_dir: &Path) -> Result<Vec<MovementProgram>> {
    let mut programs = Vec::new();

    let dat_files = find_files_with_extension(project_dir, "dat");
    let src_files = find_files_with_extension(project_dir, "src");

    for src_path in &src_files {
        let stem = src_path
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_lowercase();
        let dat_path = dat_files
            .iter()
            .find(|d| d.file_stem().unwrap().to_string_lossy().to_lowercase() == stem);

        let src_content = read_file_lossy(src_path)?;

        let dat_content = if let Some(dp) = dat_path {
            read_file_lossy(dp).ok()
        } else {
            None
        };

        if let Some(program) = parse_program(&src_content, dat_content.as_deref(), &stem)? {
            programs.push(program);
        }
    }

    Ok(programs)
}

fn find_files_with_extension(dir: &Path, ext: &str) -> Vec<std::path::PathBuf> {
    let mut files = Vec::new();
    for entry in walkdir::WalkDir::new(dir)
        .into_iter()
        .filter_map(|e| e.ok())
    {
        if entry
            .path()
            .extension()
            .map(|e| e.to_string_lossy().to_lowercase())
            == Some(ext.to_lowercase())
        {
            files.push(entry.path().to_path_buf());
        }
    }
    files
}

fn parse_program(src: &str, dat: Option<&str>, name: &str) -> Result<Option<MovementProgram>> {
    let positions = if let Some(dat_content) = dat {
        parse_dat_positions(dat_content)
    } else {
        BTreeMap::new()
    };

    let pdat_map = if let Some(dat_content) = dat {
        parse_pdat(dat_content)
    } else {
        BTreeMap::new()
    };

    let fdat_map = if let Some(dat_content) = dat {
        parse_fdat(dat_content)
    } else {
        BTreeMap::new()
    };

    let motion_commands = parse_src_motions_with_context(src);

    if motion_commands.is_empty() {
        return Ok(None);
    }

    let point_descriptions = build_point_descriptions(src, motion_commands.len());

    let frames_data = if let Some(dat_content) = dat {
        parse_dat_frames(dat_content)
    } else {
        (BTreeMap::new(), BTreeMap::new())
    };

    let program_name = extract_program_name(src, name);

    let mut points = BTreeMap::new();
    let mut sequence = Vec::new();

    for (i, cmd) in motion_commands.iter().enumerate() {
        let point_name = format!("P{}", i);
        let pos = positions
            .get(&cmd.target.to_uppercase())
            .cloned()
            .unwrap_or_default();

        // Get velocity from PDAT if available
        let pdat_vel = cmd
            .pdat_name
            .as_ref()
            .and_then(|name| pdat_map.get(&name.to_uppercase()));
        let vel = pdat_vel.map(|p| p.vel).unwrap_or(cmd.context.vel_ptp);
        let apo = pdat_vel.map(|p| p.apo_dist).unwrap_or(0.0);

        // Get tool/base from FDAT if available
        let fdat = cmd
            .fdat_name
            .as_ref()
            .and_then(|name| fdat_map.get(&name.to_uppercase()));
        let tool_no = fdat.map(|f| f.tool_no).unwrap_or(cmd.context.tool_no);
        let base_no = fdat.map(|f| f.base_no).unwrap_or(cmd.context.base_no);

        let ve = if cmd.approximate || apo > 0.0 {
            vel
        } else {
            0.0
        };

        let point = Point {
            coords: [pos.x, pos.y, pos.z],
            ori: [pos.a, pos.b, pos.c],
            params: MotionParams {
                tool: tool_no,
                base: base_no,
                vb: vel,
                ve,
                motion: cmd.motion_type.to_string(),
                s: pos.s,
                t: pos.t,
            },
            desc: if point_descriptions[i].is_empty() {
                format_point_description(&cmd.target, i, &motion_commands)
            } else {
                point_descriptions[i].clone()
            },
        };

        points.insert(point_name.clone(), point);
        sequence.push(point_name);
    }

    let frames = Frames {
        bases: build_frames_map(frames_data.0, true),
        tools: build_frames_map(frames_data.1, false),
    };

    let ios = parse_io_operations(src, dat);

    Ok(Some(MovementProgram {
        program_name,
        frames,
        points,
        sequence,
        ios,
    }))
}

fn parse_dat_positions(dat: &str) -> BTreeMap<String, KrlPosition> {
    let mut positions = BTreeMap::new();

    for line in dat.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with(';') || trimmed.starts_with('&') || trimmed.is_empty() {
            continue;
        }

        // Find the assignment: everything before '=' is the left side
        if let Some(eq_pos) = trimmed.find('=') {
            let lhs = trimmed[..eq_pos].trim();
            let value = trimmed[eq_pos + 1..].trim();

            if value.starts_with('{') && value.contains('X') {
                // Extract the variable name from LHS
                // Patterns: "DECL E6POS XP1", "DECL FRAME BASE_DATA[1]", or just "XP1"
                let var_name = extract_var_name_from_decl(lhs);
                if let Some(pos) = parse_position_struct(value) {
                    positions.insert(var_name, pos);
                }
            }
        }
    }

    positions
}

fn extract_var_name_from_decl(lhs: &str) -> String {
    let tokens: Vec<&str> = lhs.split_whitespace().collect();
    // Take the last token as the variable name (handles DECL TYPE NAME and just NAME)
    let name = tokens.last().unwrap_or(&lhs);
    // Strip array suffix for the lookup but keep it for frame parsing
    name.to_uppercase()
}

fn parse_position_struct(s: &str) -> Option<KrlPosition> {
    let inner = s.trim().trim_start_matches('{').trim_end_matches('}');
    // Skip optional type prefix like "E6POS:" or "FRAME:"
    let data = if let Some(colon_pos) = inner.find(':') {
        &inner[colon_pos + 1..]
    } else {
        inner
    };

    let mut pos = KrlPosition::default();

    for part in data.split(',') {
        let part = part.trim();
        let (key, val) = split_key_value(part)?;
        match key.to_uppercase().as_str() {
            "X" => pos.x = val.parse().ok()?,
            "Y" => pos.y = val.parse().ok()?,
            "Z" => pos.z = val.parse().ok()?,
            "A" => pos.a = val.parse().ok()?,
            "B" => pos.b = val.parse().ok()?,
            "C" => pos.c = val.parse().ok()?,
            "S" => pos.s = val.parse().ok()?,
            "T" => pos.t = val.parse().ok()?,
            _ => {}
        }
    }

    Some(pos)
}

fn split_key_value(s: &str) -> Option<(&str, &str)> {
    let s = s.trim();
    let space_pos = s.find([' ', '\t'])?;
    Some((s[..space_pos].trim(), s[space_pos..].trim()))
}

fn parse_dat_frames(
    dat: &str,
) -> (
    BTreeMap<String, (u32, KrlPosition)>,
    BTreeMap<String, (u32, KrlPosition)>,
) {
    let mut bases = BTreeMap::new();
    let mut tools = BTreeMap::new();

    for line in dat.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with(';') || trimmed.starts_with('&') || trimmed.is_empty() {
            continue;
        }

        if let Some(eq_pos) = trimmed.find('=') {
            let lhs = trimmed[..eq_pos].trim();
            let value = trimmed[eq_pos + 1..].trim();
            let var_name = extract_var_name_from_decl(lhs);

            if var_name.starts_with("BASE_DATA") {
                if let Some(idx) = extract_array_index(&var_name)
                    && let Some(pos) = parse_position_struct(value)
                {
                    bases.insert(var_name.clone(), (idx, pos));
                }
            } else if var_name.starts_with("TOOL_DATA")
                && let Some(idx) = extract_array_index(&var_name)
                && let Some(pos) = parse_position_struct(value)
            {
                tools.insert(var_name.clone(), (idx, pos));
            }
        }
    }

    (bases, tools)
}

fn extract_array_index(name: &str) -> Option<u32> {
    let start = name.find('[')?;
    let end = name.find(']')?;
    name[start + 1..end].parse().ok()
}

fn build_frames_map(
    raw: BTreeMap<String, (u32, KrlPosition)>,
    is_base: bool,
) -> BTreeMap<u32, Frame> {
    let mut map = BTreeMap::new();
    for (_name, (idx, pos)) in raw {
        map.insert(
            idx,
            Frame {
                x: pos.x,
                y: pos.y,
                z: pos.z,
                a: pos.a,
                b: pos.b,
                c: pos.c,
                name: String::new(),
            },
        );
    }
    // Always include base 0 = WORLD for base frames
    if is_base && !map.contains_key(&0) {
        map.insert(
            0,
            Frame {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                a: 0.0,
                b: 0.0,
                c: 0.0,
                name: "WORLD".to_string(),
            },
        );
    }
    map
}

fn parse_src_motions_with_context(src: &str) -> Vec<MotionCommand> {
    let mut commands = Vec::new();
    let mut ctx = MotionContext::default();
    ctx.vel_ptp = 100.0;
    ctx.tool_no = 1;

    let mut current_pdat: Option<String> = None;
    let mut current_fdat: Option<String> = None;

    for line in src.lines() {
        let trimmed = line.trim();

        // Extract Base= and RobWzg= from FOLD comments before skipping them
        if trimmed.starts_with(';') {
            let upper = trimmed.to_uppercase();
            if upper.contains(";FOLD")
                && (upper.contains("PTP") || upper.contains("LIN") || upper.contains("CIRC"))
            {
                if let Some(base_val) = extract_fold_param(&upper, "BASE=") {
                    ctx.base_no = base_val;
                }
                if let Some(tool_val) = extract_fold_param(&upper, "ROBWZG=") {
                    ctx.tool_no = tool_val;
                }
            }
            continue;
        }
        if trimmed.starts_with('&') {
            continue;
        }

        let upper = trimmed.to_uppercase();
        let tokens: Vec<&str> = trimmed.split_whitespace().collect();

        if tokens.is_empty() {
            continue;
        }

        // Track context changes
        if upper.contains("$TOOL")
            && upper.contains("TOOL_DATA")
            && let Some(idx) = extract_tool_base_index(trimmed)
        {
            ctx.tool_no = idx;
        }
        if upper.contains("$BASE")
            && upper.contains("BASE_DATA")
            && let Some(idx) = extract_tool_base_index(trimmed)
        {
            ctx.base_no = idx;
        }
        if upper.contains("PDAT_ACT")
            && upper.contains('=')
            && let Some(val) = extract_assignment_value(trimmed)
        {
            current_pdat = Some(val.to_string());
        }
        if upper.contains("FDAT_ACT")
            && upper.contains('=')
            && let Some(val) = extract_assignment_value(trimmed)
        {
            current_fdat = Some(val.to_string());
        }
        // BAS(#PTP_PARAMS, vel) or BAS(#CP_PARAMS, vel)
        if upper.contains("BAS")
            && upper.contains("#PTP_PARAMS")
            && let Some(vel) = extract_bas_velocity(trimmed)
        {
            ctx.vel_ptp = vel;
        }
        if upper.contains("BAS")
            && upper.contains("#CP_PARAMS")
            && let Some(vel) = extract_bas_velocity(trimmed)
        {
            ctx.vel_cp = vel;
        }

        let first = tokens[0].to_uppercase();

        let motion_type = match first.as_str() {
            "PTP" => Some(MotionType::Ptp),
            "PTP_REL" => Some(MotionType::PtpRel),
            "LIN" => Some(MotionType::Lin),
            "LIN_REL" => Some(MotionType::LinRel),
            "CIRC" => Some(MotionType::Circ),
            "CIRC_REL" => Some(MotionType::CircRel),
            _ => None,
        };

        if let Some(mt) = motion_type
            && tokens.len() >= 2
        {
            let target = extract_motion_target(tokens[1]);
            let approximate = upper.contains("C_PTP")
                || upper.contains("C_DIS")
                || upper.contains("C_ORI")
                || upper.contains("C_VEL");

            commands.push(MotionCommand {
                motion_type: mt,
                target,
                approximate,
                context: ctx.clone(),
                pdat_name: current_pdat.take(),
                fdat_name: current_fdat.take(),
            });
        }
    }

    commands
}

fn extract_bas_velocity(line: &str) -> Option<f64> {
    let comma_pos = line.find(',')?;
    let rest = &line[comma_pos + 1..];
    let val_str: String = rest
        .chars()
        .skip_while(|c| c.is_whitespace())
        .take_while(|c| c.is_ascii_digit() || *c == '.')
        .collect();
    val_str.parse().ok()
}

fn parse_pdat(dat: &str) -> BTreeMap<String, PdatEntry> {
    let mut map = BTreeMap::new();

    for line in dat.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with(';') || trimmed.starts_with('&') || trimmed.is_empty() {
            continue;
        }

        let upper = trimmed.to_uppercase();
        if !upper.contains("PDAT") || !trimmed.contains('=') || !trimmed.contains('{') {
            continue;
        }

        if let Some(eq_pos) = trimmed.find('=') {
            let lhs = trimmed[..eq_pos].trim();
            let value = trimmed[eq_pos + 1..].trim();
            let var_name = extract_var_name_from_decl(lhs);

            if value.starts_with('{') {
                let mut entry = PdatEntry::default();
                let inner = value.trim_start_matches('{').trim_end_matches('}');
                for part in inner.split(',') {
                    let part = part.trim();
                    if let Some((key, val)) = split_key_value(part) {
                        match key.to_uppercase().as_str() {
                            "VEL" => entry.vel = val.parse().unwrap_or(100.0),
                            "ACC" => entry.acc = val.parse().unwrap_or(100.0),
                            "APO_DIST" => entry.apo_dist = val.parse().unwrap_or(0.0),
                            _ => {}
                        }
                    }
                }
                map.insert(var_name, entry);
            }
        }
    }

    map
}

fn parse_fdat(dat: &str) -> BTreeMap<String, FdatEntry> {
    let mut map = BTreeMap::new();

    for line in dat.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with(';') || trimmed.starts_with('&') || trimmed.is_empty() {
            continue;
        }

        let upper = trimmed.to_uppercase();
        if !upper.contains("FDAT") || !trimmed.contains('=') || !trimmed.contains('{') {
            continue;
        }

        if let Some(eq_pos) = trimmed.find('=') {
            let lhs = trimmed[..eq_pos].trim();
            let value = trimmed[eq_pos + 1..].trim();
            let var_name = extract_var_name_from_decl(lhs);

            if value.starts_with('{') {
                let mut entry = FdatEntry::default();
                let inner = value.trim_start_matches('{').trim_end_matches('}');
                for part in inner.split(',') {
                    let part = part.trim();
                    if let Some((key, val)) = split_key_value(part) {
                        match key.to_uppercase().as_str() {
                            "TOOL_NO" => entry.tool_no = val.parse().unwrap_or(1),
                            "BASE_NO" => entry.base_no = val.parse().unwrap_or(0),
                            _ => {}
                        }
                    }
                }
                map.insert(var_name, entry);
            }
        }
    }

    map
}

fn extract_fold_param(upper_line: &str, param: &str) -> Option<u32> {
    let start = upper_line.find(param)? + param.len();
    let rest = &upper_line[start..];
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

fn extract_motion_target(token: &str) -> String {
    // Target might be like "XP1" or "XP1," - clean it
    token.trim_end_matches(',').to_string()
}

fn extract_tool_base_index(line: &str) -> Option<u32> {
    // Pattern: $TOOL = TOOL_DATA[3] or $BASE = BASE_DATA[1]
    let bracket_start = line.find('[')?;
    let bracket_end = line.find(']')?;
    line[bracket_start + 1..bracket_end].parse().ok()
}

fn extract_assignment_value(line: &str) -> Option<&str> {
    let eq_pos = line.find('=')?;
    Some(line[eq_pos + 1..].trim())
}

fn extract_program_name(src: &str, fallback: &str) -> String {
    // Look for DEF line to get program name
    for line in src.lines() {
        let trimmed = line.trim();
        let upper = trimmed.to_uppercase();
        if upper.starts_with("DEF") && !upper.starts_with("DEFDAT") && !upper.starts_with("DEFFCT")
        {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 2 {
                let name = parts[1].split('(').next().unwrap_or(parts[1]);
                return name.to_string();
            }
        }
    }
    fallback.to_string()
}

fn format_point_description(target: &str, index: usize, commands: &[MotionCommand]) -> String {
    if index == 0 {
        return "Home position".to_string();
    }
    if index == commands.len() - 1 {
        return "Return to home".to_string();
    }
    format!("Motion to {}", target)
}

pub fn extract_fold_comments(src: &str) -> Vec<(usize, String)> {
    let mut comments = Vec::new();
    let mut motion_index = 0;

    for line in src.lines() {
        let trimmed = line.trim();

        if trimmed.contains("%CMOVE")
            || (trimmed.contains(";FOLD PTP")
                || trimmed.contains(";FOLD LIN")
                || trimmed.contains(";FOLD CIRC"))
                && trimmed.contains("%P")
        {
            motion_index += 1;
        }

        if trimmed.contains("VCOMMENT") || trimmed.contains("VW_COMMENT") {
            if let Some(start) = trimmed.find("2:") {
                let rest = &trimmed[start + 2..];
                let end = rest.find(',').unwrap_or(rest.len());
                let comment = rest[..end].trim().trim_matches('-').trim();
                if !comment.is_empty()
                    && comment.len() > 2
                    && !comment.chars().all(|c| c == '=' || c == 'x' || c == '-')
                {
                    comments.push((motion_index, comment.to_string()));
                }
            }
        }
    }
    comments
}

pub fn build_point_descriptions(src: &str, num_points: usize) -> Vec<String> {
    let comments = extract_fold_comments(src);
    let mut descs = vec![String::new(); num_points];

    if num_points == 0 {
        return descs;
    }

    descs[0] = "Home position".to_string();
    if num_points > 1 {
        descs[num_points - 1] = "Return to home".to_string();
    }

    for (motion_idx, comment) in &comments {
        let idx = (*motion_idx).min(num_points.saturating_sub(1));
        if idx > 0 && idx < num_points - 1 && descs[idx].is_empty() {
            descs[idx] = comment.clone();
        }
    }

    descs
}

fn parse_io_operations(src: &str, dat: Option<&str>) -> Vec<IoOperation> {
    let mut ios = Vec::new();

    // Collect SIGNAL declarations from both src and dat for name resolution
    let mut signal_map: BTreeMap<String, (String, u32)> = BTreeMap::new();
    let all_content: Vec<&str> = if let Some(d) = dat {
        vec![src, d]
    } else {
        vec![src]
    };

    for content in &all_content {
        for line in content.lines() {
            let trimmed = line.trim();
            let upper = trimmed.to_uppercase();
            if upper.contains("SIGNAL") && !trimmed.starts_with(';') && !trimmed.starts_with('&') {
                if let Some(sig) = parse_signal_declaration(trimmed) {
                    signal_map.insert(sig.0.to_uppercase(), (sig.1, sig.2));
                }
            }
        }
    }

    for line in src.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with(';') || trimmed.starts_with('&') || trimmed.is_empty() {
            continue;
        }
        let upper = trimmed.to_uppercase();

        // $OUT[n] = TRUE/FALSE (but not TRIGGER lines, those are handled separately)
        if upper.contains("$OUT[") && upper.contains('=') && !upper.contains("TRIGGER") {
            if let Some(io) = parse_output_assignment(trimmed, &upper) {
                ios.push(io);
            }
        }

        // WAIT FOR $IN[n]==TRUE/FALSE
        if upper.contains("WAIT") && upper.contains("$IN[") {
            if let Some(io) = parse_wait_input(trimmed, &upper) {
                ios.push(io);
            }
        }
        // Standalone $IN[n] read in conditions (IF $IN[n]...)
        else if upper.contains("$IN[") && (upper.contains("IF") || upper.contains("WHEN")) {
            if let Some(io) = parse_input_condition(trimmed, &upper) {
                ios.push(io);
            }
        }

        // PULSE($OUT[n], TRUE/FALSE, duration)
        if upper.contains("PULSE") && upper.contains("$OUT[") {
            if let Some(io) = parse_pulse(trimmed, &upper) {
                ios.push(io);
            }
        }

        // Signal-based outputs: signal_name = TRUE/FALSE
        // Check if the left side of an assignment is a known signal name
        if upper.contains('=') && !upper.contains("$OUT") && !upper.contains("$IN") {
            if let Some(eq_pos) = trimmed.find('=') {
                let lhs = trimmed[..eq_pos].trim().to_uppercase();
                let rhs = trimmed[eq_pos + 1..].trim().to_uppercase();
                if let Some((io_type, channel)) = signal_map.get(&lhs) {
                    if rhs == "TRUE" || rhs == "FALSE" {
                        ios.push(IoOperation {
                            io_type: io_type.clone(),
                            channel: *channel,
                            action: format!("set {}", rhs),
                            signal_name: Some(lhs.clone()),
                            context: trimmed.to_string(),
                        });
                    }
                }
            }
        }

        // ANOUT/ANIN statements
        if upper.starts_with("ANOUT") {
            if let Some(io) = parse_anout(trimmed) {
                ios.push(io);
            }
        }
        if upper.starts_with("ANIN") {
            if let Some(io) = parse_anin(trimmed) {
                ios.push(io);
            }
        }

        // TRIGGER WHEN ... DO $OUT[n]=...
        if upper.contains("TRIGGER") && upper.contains("$OUT[") {
            if let Some(io) = parse_trigger_io(trimmed, &upper) {
                ios.push(io);
            }
        }
    }

    ios
}

/// Parse: SIGNAL name $OUT[n] or SIGNAL name $IN[n] (optionally with TO for bit ranges)
fn parse_signal_declaration(line: &str) -> Option<(String, String, u32)> {
    let upper = line.to_uppercase();
    let signal_pos = upper.find("SIGNAL")?;
    let rest = line[signal_pos + 6..].trim();
    let tokens: Vec<&str> = rest.split_whitespace().collect();
    if tokens.len() < 2 {
        return None;
    }
    let name = tokens[0];
    let io_ref = tokens[1].to_uppercase();

    let (io_type, channel) = parse_io_reference(&io_ref)?;
    Some((name.to_string(), io_type, channel))
}

/// Parse $OUT[n] or $IN[n] and return (type_string, channel_number)
fn parse_io_reference(s: &str) -> Option<(String, u32)> {
    let upper = s.to_uppercase();
    if upper.contains("$OUT[") {
        let start = upper.find("$OUT[")? + 5;
        let end = upper[start..].find(']')? + start;
        let channel: u32 = upper[start..end].parse().ok()?;
        Some(("OUT".to_string(), channel))
    } else if upper.contains("$IN[") {
        let start = upper.find("$IN[")? + 4;
        let end = upper[start..].find(']')? + start;
        let channel: u32 = upper[start..end].parse().ok()?;
        Some(("IN".to_string(), channel))
    } else {
        None
    }
}

/// Parse: $OUT[n] = TRUE/FALSE
fn parse_output_assignment(line: &str, upper: &str) -> Option<IoOperation> {
    let (_, channel) = parse_io_reference(upper)?;
    let eq_pos = line.find('=')?;
    let rhs = line[eq_pos + 1..].trim().to_uppercase();
    let value = if rhs.starts_with("TRUE") || rhs == "TRUE" {
        "set TRUE"
    } else if rhs.starts_with("FALSE") || rhs == "FALSE" {
        "set FALSE"
    } else {
        "set expression"
    };

    Some(IoOperation {
        io_type: "OUT".to_string(),
        channel,
        action: value.to_string(),
        signal_name: None,
        context: line.trim().to_string(),
    })
}

/// Parse: WAIT FOR $IN[n]==TRUE/FALSE
fn parse_wait_input(line: &str, upper: &str) -> Option<IoOperation> {
    let (_, channel) = parse_io_reference(upper)?;
    let value = if upper.contains("TRUE") {
        "wait TRUE"
    } else if upper.contains("FALSE") {
        "wait FALSE"
    } else {
        "wait condition"
    };

    Some(IoOperation {
        io_type: "IN".to_string(),
        channel,
        action: value.to_string(),
        signal_name: None,
        context: line.trim().to_string(),
    })
}

/// Parse: IF $IN[n]==... or WHEN $IN[n]==...
fn parse_input_condition(line: &str, upper: &str) -> Option<IoOperation> {
    let (_, channel) = parse_io_reference(upper)?;

    Some(IoOperation {
        io_type: "IN".to_string(),
        channel,
        action: "read condition".to_string(),
        signal_name: None,
        context: line.trim().to_string(),
    })
}

/// Parse: PULSE($OUT[n], TRUE/FALSE, duration)
fn parse_pulse(line: &str, upper: &str) -> Option<IoOperation> {
    let (_, channel) = parse_io_reference(upper)?;
    let value = if upper.contains("TRUE") {
        "pulse TRUE"
    } else {
        "pulse FALSE"
    };

    Some(IoOperation {
        io_type: "OUT".to_string(),
        channel,
        action: value.to_string(),
        signal_name: None,
        context: line.trim().to_string(),
    })
}

/// Parse: TRIGGER WHEN ... DO $OUT[n]=...
fn parse_trigger_io(line: &str, upper: &str) -> Option<IoOperation> {
    let do_pos = upper.find("DO")?;
    let action_part = &line[do_pos + 2..];
    let action_upper = action_part.to_uppercase();
    let (_, channel) = parse_io_reference(&action_upper)?;

    let value = if action_upper.contains("TRUE") {
        "trigger set TRUE"
    } else if action_upper.contains("FALSE") {
        "trigger set FALSE"
    } else {
        "trigger set expression"
    };

    Some(IoOperation {
        io_type: "OUT".to_string(),
        channel,
        action: value.to_string(),
        signal_name: None,
        context: line.trim().to_string(),
    })
}

/// Parse: ANOUT ON name=value ...
fn parse_anout(line: &str) -> Option<IoOperation> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    if tokens.len() < 2 {
        return None;
    }
    // ANOUT ON/OFF channel_name or ANOUT channel_name = expression
    let action = if tokens.len() > 1 && tokens[1].to_uppercase() == "OFF" {
        "analog off"
    } else {
        "analog set"
    };

    Some(IoOperation {
        io_type: "ANOUT".to_string(),
        channel: 0,
        action: action.to_string(),
        signal_name: tokens.get(1).map(|s| s.to_string()),
        context: line.trim().to_string(),
    })
}

/// Parse: ANIN ON name ...
fn parse_anin(line: &str) -> Option<IoOperation> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    if tokens.len() < 2 {
        return None;
    }
    let action = if tokens.len() > 1 && tokens[1].to_uppercase() == "OFF" {
        "analog off"
    } else {
        "analog read"
    };

    Some(IoOperation {
        io_type: "ANIN".to_string(),
        channel: 0,
        action: action.to_string(),
        signal_name: tokens.get(1).map(|s| s.to_string()),
        context: line.trim().to_string(),
    })
}
