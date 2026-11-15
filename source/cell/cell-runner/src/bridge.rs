use crate::cell::{Phase, RobotState, SignalValue};
use crate::krl::{RobotKrl, StepStatus};
use crate::tools::ToolSimulator;

pub fn plc_to_krl(robot: &RobotState, krl: &mut RobotKrl) {
    let interp = match &mut krl.interpreter {
        Some(i) => i,
        None => return,
    };

    if let Some(SignalValue::Bool(true)) = robot.outputs.get("Frei_RC") {
        interp.io.set_input(2, true); // $IN[2] = Antriebe Ein
    }
    if let Some(SignalValue::Bool(true)) = robot.outputs.get("MaSich") {
        interp.io.set_input(7, true); // $IN[7] = Sicherheitskreis
    }
    interp.io.set_input(1, true);  // $IN[1] = Externer Start
    interp.io.set_input(3, true);  // $IN[3] = Automatik Bestätigung
    interp.io.set_input(8, true);  // $IN[8] = Not-Halt rückgesetzt
}

pub fn krl_to_plc(krl: &RobotKrl, robot: &mut RobotState) {
    // Program complete → FM
    if krl.program_complete {
        robot.inputs.insert("FM".into(), SignalValue::Bool(true));
    }

    // If KRL is running, it manages RoboterBereit and PF0
    if let Some(ref interp) = krl.interpreter {
        // $OUT[1] = Grundstellung (Home) → PF0
        let pf0 = interp.io.get_output(1);
        if pf0 {
            robot.inputs.insert("PF0".into(), SignalValue::Bool(true));
        }

        // $OUT[2] = Betriebsbereit → RoboterBereit
        let ready = interp.io.get_output(2);
        if ready {
            robot.inputs.insert("RoboterBereit".into(), SignalValue::Bool(true));
        }
    }
}

pub fn handle_folge_selection(robot: &mut RobotState, krl: &mut RobotKrl) {
    let folge = match robot.outputs.get("AnwahlFolge") {
        Some(SignalValue::Byte(f)) => *f,
        _ => 0,
    };

    // Echo SpiegelungFolge immediately when PLC sets AnwahlFolge
    if folge > 0 {
        robot.inputs.insert("SpiegelungFolge".into(), SignalValue::Byte(folge));
    }

    // Start KRL program when PLC is in Running or WaitFM and KRL hasn't started yet
    if folge > 0
        && (robot.phase == Phase::Running || robot.phase == Phase::WaitFM)
        && krl.active_folge != folge
    {
        krl.start_program(folge);
    }
}

pub fn sync_step(robot: &mut RobotState, krl: &mut RobotKrl, krl_steps_per_cycle: usize, tool_sim: &mut ToolSimulator, robot_idx: usize) {
    handle_folge_selection(robot, krl);
    plc_to_krl(robot, krl);

    if krl.interpreter.is_some() && !krl.program_complete {
        for _ in 0..krl_steps_per_cycle {
            if krl.program_complete {
                break;
            }
            // Run tool simulator before each KRL step (processes timers)
            if let Some(ref mut interp) = krl.interpreter {
                tool_sim.step_robot(robot_idx, &mut interp.io);
            }
            match krl.step(false) {
                StepStatus::MotionInProgress => break, // motion consuming time, wait for next cycle
                StepStatus::Executed => continue,      // instant instruction, try next
                StepStatus::Blocked => break,          // waiting for input signal
                StepStatus::Done => break,
            }
        }
    }

    krl_to_plc(krl, robot);
}
