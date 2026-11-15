//! Telegram -> KRL `$IN`/`$OUT` mapping of the `sim` profile.
//!
//! Telegram byte b, bit i <-> `$IN[in_base + 8b + i + 1]` (PLC -> robot) and
//! `$OUT[out_base + 8b + i + 1]` (robot -> PLC), as the DIO256 module is mapped on the VW
//! controllers (`KRC_IO.xml`; contract.md section 1). Byte layout: bt2plc DESIGN section 4.1;
//! STEP, REQUEST, GRANT and PLAN_ID offsets come from program.json.

use crate::config::SimCfg;
use crate::program::Program;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SimIo {
    pub in_base: u16,
    pub out_base: u16,
    /// First `$OUT` of the 16-bit op index.
    pub step: u16,
    /// `$OUT` of REQUEST bit 0 (bit b = `request + b`).
    pub request: u16,
    /// `$IN` of GRANT bit 0.
    pub grant: u16,
    /// First `$IN` of the 16-bit PLAN_ID.
    pub plan_id: u16,
    /// Job number in byte 2 only (8 bits) instead of bytes 2-3.
    pub job_byte: bool,
}

/// One single-bit signal of the status / command bytes 0-1.
#[derive(Clone, Copy, Debug)]
pub struct Bit {
    pub name: &'static str,
    pub byte: u16,
    pub bit: u16,
    pub what: &'static str,
}

/// PLC -> robot commands (bytes 0-1) that bt_sps reads.
pub const COMMANDS: &[Bit] = &[
    Bit {
        name: "BT_C_DRIVES",
        byte: 0,
        bit: 0,
        what: "DrivesOn (level)",
    },
    Bit {
        name: "BT_C_START",
        byte: 0,
        bit: 1,
        what: "ProgStart (500 ms pulse)",
    },
    Bit {
        name: "BT_C_STOP",
        byte: 0,
        bit: 2,
        what: "Stop (level)",
    },
    Bit {
        name: "BT_C_CANCEL",
        byte: 0,
        bit: 3,
        what: "Cancel (level)",
    },
    Bit {
        name: "BT_C_EXT",
        byte: 0,
        bit: 4,
        what: "EXT mode request",
    },
    Bit {
        name: "BT_C_RESET",
        byte: 0,
        bit: 5,
        what: "Reset (held 1 s)",
    },
    Bit {
        name: "BT_C_JOBREQ",
        byte: 1,
        bit: 0,
        what: "JobRequest",
    },
];

/// Robot -> PLC status bits (bytes 0-1) that bt_sps writes.
pub const STATUS: &[Bit] = &[
    Bit {
        name: "BT_O_DRIVES",
        byte: 0,
        bit: 0,
        what: "DrivesPowered",
    },
    Bit {
        name: "BT_O_RUN",
        byte: 0,
        bit: 1,
        what: "ProgRunning",
    },
    Bit {
        name: "BT_O_HOME",
        byte: 0,
        bit: 2,
        what: "AtHome",
    },
    Bit {
        name: "BT_O_INPOS",
        byte: 0,
        bit: 3,
        what: "InPosition",
    },
    Bit {
        name: "BT_O_EXT",
        byte: 0,
        bit: 4,
        what: "ModeExt",
    },
    Bit {
        name: "BT_O_READY",
        byte: 0,
        bit: 5,
        what: "Ready",
    },
    Bit {
        name: "BT_O_ERROR",
        byte: 0,
        bit: 6,
        what: "Error",
    },
    Bit {
        name: "BT_O_WARN",
        byte: 0,
        bit: 7,
        what: "Warning",
    },
    Bit {
        name: "BT_O_GATES",
        byte: 1,
        bit: 0,
        what: "Gates closed",
    },
    Bit {
        name: "BT_O_ESTOP",
        byte: 1,
        bit: 1,
        what: "EStopOK",
    },
    Bit {
        name: "BT_O_FIN",
        byte: 1,
        bit: 2,
        what: "ProgFinished",
    },
    Bit {
        name: "BT_O_JOBACK",
        byte: 1,
        bit: 3,
        what: "JobAck",
    },
    Bit {
        name: "BT_O_PERIPH",
        byte: 1,
        bit: 6,
        what: "Periphery ready",
    },
];

/// Robot -> PLC: job echo (bytes 2-3), error code (10-11), warning code (12-13), joints (16-27).
pub const ECHO_BYTE: u16 = 2;
pub const ERROR_BYTE: u16 = 10;
pub const WARNING_BYTE: u16 = 12;
pub const JOINT_BYTE: u16 = 16;
/// PLC -> robot: job number (bytes 2-3, or byte 2 only).
pub const JOB_BYTE: u16 = 2;

impl SimIo {
    pub fn new(p: &Program, c: &SimCfg) -> SimIo {
        let pr = &p.protocol;
        SimIo {
            in_base: c.in_base,
            out_base: c.out_base,
            step: c.out_base + 8 * u16::from(pr.step.byte) + 1,
            request: c.out_base + 8 * u16::from(pr.request.byte) + 1,
            grant: c.in_base + 8 * u16::from(pr.grant.byte) + 1,
            plan_id: c.in_base + 8 * u16::from(pr.plan_id.byte) + 1,
            job_byte: pr.job_number_byte(),
        }
    }

    pub fn input(&self, byte: u16, bit: u16) -> u16 {
        self.in_base + 8 * byte + bit + 1
    }

    pub fn output(&self, byte: u16, bit: u16) -> u16 {
        self.out_base + 8 * byte + bit + 1
    }

    pub fn req_bit(&self, b: u32) -> u16 {
        self.request + b as u16
    }

    pub fn grant_bit(&self, b: u32) -> u16 {
        self.grant + b as u16
    }

    pub fn job_bits(&self) -> u16 {
        if self.job_byte { 8 } else { 16 }
    }

    /// `$IN` range `[lo, hi]` of the telegram.
    pub fn in_range(&self) -> (u16, u16) {
        (self.in_base + 1, self.in_base + 256)
    }

    pub fn out_range(&self) -> (u16, u16) {
        (self.out_base + 1, self.out_base + 256)
    }
}

/// Bits of a mask, ascending (zone bits first, the permit bit 31 last).
pub fn bits(mask: u32) -> Vec<u32> {
    (0..32).filter(|b| mask & (1 << b) != 0).collect()
}
