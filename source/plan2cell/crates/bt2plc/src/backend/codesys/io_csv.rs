//! IO mapping CSV in the "CoDeSys Mapping Export V1.2" layout (fallback for MCP servers that
//! can import mappings but not set them per channel). The IEC address column is left empty so
//! CODESYS assigns addresses; this import path is unverified (see PROJECT.md).

use std::fmt::Write as _;

use crate::backend::model::{PlcProject, ProjectRole};
use crate::config::Config;
use crate::ir::CellPlan;

const HEADER: &str = "//CoDeSys Mapping Export V1.2\n//Mapped variable,//Parameter name @ counter in device,//Unit,//Description,//IEC address,//Device name\n//Important: change only first, third or fourth column in Excel or add variable name before first\n";

pub fn io_mapping_csv(plan: &CellPlan, _cfg: &Config, p: &PlcProject) -> String {
    let mut o = String::from(HEADER);
    let robots: Vec<_> = match p.role {
        ProjectRole::HeadPlc => plan
            .robots
            .iter()
            .map(|r| (Some(r.addr.tag.clone()), r))
            .collect(),
        ProjectRole::RobotSim => vec![(None, &plan.robots[p.robot.unwrap()])],
    };
    let head = p.role == ProjectRole::HeadPlc;
    for (tag, r) in robots {
        let sfx = tag.as_ref().map(|t| format!("_{t}")).unwrap_or_default();
        let _ = writeln!(
            o,
            "//\n// {} {} ({}, {})\n//",
            r.addr.tag, r.ns, r.addr.station, r.addr.ip
        );
        let (in_ch, out_ch, in_var, out_var, in_desc, out_desc) = if head {
            (
                "8 safe digital inputs",
                "8 safe digital outputs",
                "bySafeIn",
                "bySafeOut",
                "safe status from robot",
                "safe commands to robot",
            )
        } else {
            (
                "8 safe digital outputs",
                "8 safe digital inputs",
                "bySafeOut",
                "bySafeIn",
                "safe commands from head PLC",
                "safe status to head PLC",
            )
        };
        let fd = format!("SafeIO_FDIO64{sfx}");
        // safety_module = false (head only): slot 1 is empty, no FDIO64 rows.
        let safe = !head || r.addr.safety_module;
        for k in (0..8).filter(|_| safe) {
            let ch = if k == 0 {
                in_ch.to_string()
            } else {
                format!("{in_ch}@{k}")
            };
            let _ = writeln!(
                o,
                "Application.GVL_IO.{in_var}{k}{sfx},{ch},,{} byte {k},,{fd}",
                in_desc
            );
        }
        for k in (0..8).filter(|_| safe) {
            let ch = if k == 0 {
                out_ch.to_string()
            } else {
                format!("{out_ch}@{k}")
            };
            let _ = writeln!(
                o,
                "Application.GVL_IO.{out_var}{k}{sfx},{ch},,{} byte {k},,{fd}",
                out_desc
            );
        }
        let dio = format!("VASS_DIO256{sfx}");
        if head {
            let _ = writeln!(
                o,
                "Application.GVL_IO.aVassInputData{sfx},Inputs,,VASS status from robot,,{dio}"
            );
            let _ = writeln!(
                o,
                "Application.GVL_IO.aVassOutputData{sfx},Outputs,,VASS commands to robot,,{dio}"
            );
        } else {
            let _ = writeln!(
                o,
                "Application.GVL_IO.aVassOutputData,Output Data,,VASS commands from head PLC,,{dio}"
            );
            let _ = writeln!(
                o,
                "Application.GVL_IO.aVassInputData,Input Data,,VASS status to head PLC,,{dio}"
            );
        }
    }
    o
}
