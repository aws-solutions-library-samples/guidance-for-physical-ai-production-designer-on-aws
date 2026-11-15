//! Hardware addressing rules of the PROFINET segment: station names (NameOfStation) and the IPv4
//! addresses of the head PLC and the robots.
//!
//! One implementation for every caller: the bt2plc plan ([`crate::ir`] rejects a cell that breaks
//! a rule), `plan2cell hardware` (lists every problem at once) and front ends that check a form
//! before they save it.

use std::collections::BTreeMap;
use std::net::Ipv4Addr;

use serde::Serialize;

use crate::config::NetworkCfg;

/// Longest NameOfStation (IEC 61158-6-10).
pub const STATION_MAX_LEN: usize = 240;
/// Longest label of a NameOfStation.
pub const STATION_LABEL_MAX_LEN: usize = 63;

/// Why `name` is not a valid PROFINET NameOfStation (IEC 61158-6-10), or `None`.
///
/// - 1..=240 characters in total, labels of 1..=63 characters separated by `.`;
/// - only `a-z`, `0-9` and `-`; no label starts or ends with `-`;
/// - not shaped like an IPv4 address (`n.n.n.n`, n = 0..999);
/// - does not start with `port-xyz` or `port-xyz-abcde` (digits; reserved for port names).
pub fn station_name_problem(name: &str) -> Option<String> {
    if name.is_empty() {
        return Some("is empty".into());
    }
    if name.len() > STATION_MAX_LEN {
        return Some(format!(
            "is {} characters long (at most {STATION_MAX_LEN})",
            name.len()
        ));
    }
    if let Some(c) = name
        .chars()
        .find(|c| !(c.is_ascii_lowercase() || c.is_ascii_digit() || *c == '-' || *c == '.'))
    {
        return Some(format!(
            "contains '{c}' (only lowercase letters a-z, digits, '-' and '.' are allowed)"
        ));
    }
    let labels: Vec<&str> = name.split('.').collect();
    for l in &labels {
        if l.is_empty() {
            return Some("has an empty label (a leading, trailing or double '.')".into());
        }
        if l.len() > STATION_LABEL_MAX_LEN {
            return Some(format!(
                "has a label of {} characters ('{l}'; at most {STATION_LABEL_MAX_LEN})",
                l.len()
            ));
        }
        if l.starts_with('-') || l.ends_with('-') {
            return Some(format!("has a label that starts or ends with '-' ('{l}')"));
        }
    }
    if labels.len() == 4
        && labels
            .iter()
            .all(|l| l.len() <= 3 && l.bytes().all(|b| b.is_ascii_digit()))
    {
        return Some("has the form of an IPv4 address (n.n.n.n)".into());
    }
    if is_port_name(labels[0]) {
        return Some(format!(
            "starts with '{}' (port-xyz / port-xyz-abcde are reserved for port names)",
            labels[0]
        ));
    }
    None
}

fn is_port_name(label: &str) -> bool {
    let digits = |s: &str, n: usize| s.len() == n && s.bytes().all(|b| b.is_ascii_digit());
    let Some(rest) = label.strip_prefix("port-") else {
        return false;
    };
    match rest.split_once('-') {
        None => digits(rest, 3),
        Some((a, b)) => digits(a, 3) && digits(b, 5),
    }
}

/// Longest robot device name.
pub const DEVICE_NAME_MAX_LEN: usize = 64;

/// Names a robot's device name must not have (compared case-insensitively). The device name
/// becomes a device object under `PN_Controller` in the head project, under `Ethernet` in the
/// robot's simulator project, and a global IEC identifier (`<device>.xRunning`), so it must not
/// clash with the projects' own objects or with IEC words. Kept in sync with
/// `factoryos_protocol::hardware::RESERVED_DEVICE_NAMES` (a test there compares the lists).
pub const RESERVED_DEVICE_NAMES: &[&str] = &[
    // the device tree, tasks, libraries and generated GVLs/modules of the generated projects
    "Device",
    "Application",
    "Ethernet",
    "PN_Controller",
    "Library_Manager",
    "Task_Configuration",
    "MainTask",
    "Profinet_CommunicationTask",
    "Profinet_IOTask",
    "GVL_Cell",
    "GVL_Program",
    "SafeIO_FDIO64",
    "VASS_DIO256",
    "Standard",
    "IoStandard",
    "IoDrvEthernet",
    "IoDrvProfinet",
    "IoDrvProfinetDevice",
    "ProfinetCommon",
    "CommFB",
    // the ST templates' objects (templates/codesys/{head,device})
    "E_PlayerState",
    "E_ProgramState",
    "E_RobotMode",
    "ST_OpDef",
    "ST_PROFIsafe_Trailer",
    "ST_SafeInput",
    "ST_SafeOutput",
    "ST_VASS_ControlWord",
    "ST_VASS_InputTelegram",
    "ST_VASS_OutputTelegram",
    "ST_VASS_StatusWord",
    "ST_Waypoint",
    "GVL_DeviceParams",
    "GVL_IO",
    "FB_JointAngleEncoder",
    "FB_PROFIsafe_Simulator",
    "FB_ProgramPlayer",
    "FB_VASS_DeviceSimulator",
    "PLC_PRG",
    "E_CellError",
    "E_CellState",
    "E_GrantPolicy",
    "E_RobotState",
    "ST_OpInfo",
    "ST_RobotIn",
    "ST_RobotOut",
    "ST_Ticket",
    "FB_Cell",
    "FB_VassRobot",
    "FB_ZoneArbiter",
    // IEC 61131-3 / CODESYS keywords
    "ABSTRACT",
    "ACTION",
    "END_ACTION",
    "AND",
    "AND_THEN",
    "ARRAY",
    "AT",
    "BY",
    "CASE",
    "END_CASE",
    "CONFIGURATION",
    "END_CONFIGURATION",
    "CONSTANT",
    "CONTINUE",
    "DO",
    "ELSE",
    "ELSIF",
    "EN",
    "ENO",
    "EXIT",
    "EXTENDS",
    "FALSE",
    "FINAL",
    "FOR",
    "END_FOR",
    "FUNCTION",
    "END_FUNCTION",
    "FUNCTION_BLOCK",
    "END_FUNCTION_BLOCK",
    "IF",
    "END_IF",
    "IMPLEMENTS",
    "INTERFACE",
    "END_INTERFACE",
    "INTERNAL",
    "JMP",
    "METHOD",
    "END_METHOD",
    "MOD",
    "NOT",
    "NULL",
    "OF",
    "OR",
    "OR_ELSE",
    "OVERLAP",
    "PERSISTENT",
    "POINTER",
    "PRIVATE",
    "PROGRAM",
    "END_PROGRAM",
    "PROPERTY",
    "END_PROPERTY",
    "PROTECTED",
    "PUBLIC",
    "READ_ONLY",
    "READ_WRITE",
    "REF",
    "REFERENCE",
    "REF_TO",
    "REPEAT",
    "END_REPEAT",
    "RESOURCE",
    "END_RESOURCE",
    "RETAIN",
    "RETURN",
    "STEP",
    "END_STEP",
    "STRUCT",
    "END_STRUCT",
    "SUPER",
    "TASK",
    "THEN",
    "THIS",
    "TO",
    "TRANSITION",
    "END_TRANSITION",
    "TRUE",
    "TYPE",
    "END_TYPE",
    "UNION",
    "END_UNION",
    "UNTIL",
    "VAR",
    "VAR_ACCESS",
    "VAR_CONFIG",
    "VAR_EXTERNAL",
    "VAR_GLOBAL",
    "VAR_IN_OUT",
    "VAR_INPUT",
    "VAR_INST",
    "VAR_OUTPUT",
    "VAR_STAT",
    "VAR_TEMP",
    "END_VAR",
    "WHILE",
    "END_WHILE",
    "WITH",
    "XOR",
    // elementary and generic data types
    "ANY",
    "ANY_BIT",
    "ANY_DATE",
    "ANY_DERIVED",
    "ANY_ELEMENTARY",
    "ANY_INT",
    "ANY_MAGNITUDE",
    "ANY_NUM",
    "ANY_REAL",
    "ANY_STRING",
    "BIT",
    "BOOL",
    "BYTE",
    "CHAR",
    "DATE",
    "DATE_AND_TIME",
    "DINT",
    "DT",
    "DWORD",
    "INT",
    "LDATE",
    "LDATE_AND_TIME",
    "LDT",
    "LINT",
    "LREAL",
    "LTIME",
    "LTIME_OF_DAY",
    "LTOD",
    "LWORD",
    "REAL",
    "SINT",
    "STRING",
    "TIME",
    "TIME_OF_DAY",
    "TOD",
    "UDINT",
    "UINT",
    "ULINT",
    "USINT",
    "WCHAR",
    "WORD",
    "WSTRING",
    // standard function blocks and operators
    "TON",
    "TOF",
    "TP",
    "RTC",
    "R_TRIG",
    "F_TRIG",
    "CTU",
    "CTD",
    "CTUD",
    "RS",
    "SR",
    "SEMA",
    "ABS",
    "SQRT",
    "LN",
    "LOG",
    "EXP",
    "SIN",
    "COS",
    "TAN",
    "ASIN",
    "ACOS",
    "ATAN",
    "EXPT",
    "ADD",
    "SUB",
    "MUL",
    "DIV",
    "MOVE",
    "SEL",
    "MAX",
    "MIN",
    "LIMIT",
    "MUX",
    "SHL",
    "SHR",
    "ROL",
    "ROR",
    "GT",
    "GE",
    "EQ",
    "LE",
    "LT",
    "NE",
    "SIZEOF",
    "ADR",
    "BITADR",
    "INDEXOF",
    "TRUNC",
    "LEN",
    "LEFT",
    "RIGHT",
    "MID",
    "CONCAT",
    "INSERT",
    "DELETE",
    "REPLACE",
    "FIND",
];

/// Prefixes of the per-robot module names (`SafeIO_FDIO64_<tag>`, `VASS_DIO256_<tag>`).
pub const RESERVED_DEVICE_NAME_PREFIXES: &[&str] = &["SafeIO_FDIO64_", "VASS_DIO256_"];

/// Why `name` is not a valid robot device name, or `None`: an IEC identifier (a letter, then
/// letters, digits and `_`, at most 64 characters) without `__`, not ending in `_`, and none of
/// [`RESERVED_DEVICE_NAMES`] / [`RESERVED_DEVICE_NAME_PREFIXES`] (case-insensitive).
pub fn device_name_problem(name: &str) -> Option<String> {
    if name.is_empty() {
        return Some("is empty".into());
    }
    if name.len() > DEVICE_NAME_MAX_LEN {
        return Some(format!(
            "is {} characters long (at most {DEVICE_NAME_MAX_LEN})",
            name.len()
        ));
    }
    if !name.as_bytes()[0].is_ascii_alphabetic()
        || !name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
    {
        return Some("must be an IEC identifier (a letter, then letters, digits and '_')".into());
    }
    if name.contains("__") || name.ends_with('_') {
        return Some("must not contain '__' or end with '_' (reserved by CODESYS)".into());
    }
    if let Some(r) = RESERVED_DEVICE_NAMES
        .iter()
        .find(|r| r.eq_ignore_ascii_case(name))
    {
        return Some(format!(
            "is reserved ('{r}' is a name of the generated project or an IEC word)"
        ));
    }
    let upper = name.to_ascii_uppercase();
    if let Some(p) = RESERVED_DEVICE_NAME_PREFIXES
        .iter()
        .find(|p| upper.starts_with(&p.to_ascii_uppercase()))
    {
        return Some(format!(
            "starts with '{p}' (reserved for the robots' module names)"
        ));
    }
    None
}

/// One rule broken by the hardware configuration.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Issue {
    /// What is wrong, in the JSON names of `plan2cell hardware`: `network.headIp`,
    /// `network.controllerStation`, `robots.<ns>.station`, `robots.<ns>.ip`, ...
    pub field: String,
    pub message: String,
}

/// A robot's PROFINET identity, as the plan resolved it.
#[derive(Clone, Copy, Debug)]
pub struct DeviceAddr<'a> {
    pub ns: &'a str,
    pub station: &'a str,
    pub ip: Ipv4Addr,
}

/// Every hardware rule, every problem (empty = valid):
///
/// - the netmask is contiguous; the head IP is a host address of its subnet; the gateway
///   (`0.0.0.0` = none) is a host address of that subnet and not the head IP; the PN
///   controller's IP pool (`ip_pool_first` ..= `ip_pool_last`) lies in that subnet;
/// - every station name (the head's `controller_station` and each robot's) is a valid
///   NameOfStation ([`station_name_problem`]) and they are unique;
/// - every robot IP is inside the head subnet, is neither its network nor its broadcast address,
///   nor the gateway nor the head IP, and the IPs are unique.
pub fn check(net: &NetworkCfg, robots: &[DeviceAddr<'_>]) -> Vec<Issue> {
    let mut out = Vec::new();
    let mut issue = |field: String, message: String| out.push(Issue { field, message });
    let mask = u32::from(net.netmask);
    let contiguous = mask.leading_ones() + mask.trailing_zeros() == 32;
    if !contiguous || mask.leading_ones() > 30 || mask == 0 {
        issue(
            "network.netmask".into(),
            format!(
                "netmask {} must be a contiguous mask from /1 to /30",
                net.netmask
            ),
        );
        // Without a usable subnet the address rules below say nothing useful.
        return out;
    }
    let net_addr = u32::from(net.head_ip) & mask;
    let bcast = net_addr | !mask;
    let subnet = format!("{}/{}", Ipv4Addr::from(net_addr), mask.leading_ones());
    let host_problem = |ip: Ipv4Addr| -> Option<String> {
        let v = u32::from(ip);
        if v & mask != net_addr {
            Some(format!("{ip} is outside the head subnet {subnet}"))
        } else if v == net_addr {
            Some(format!("{ip} is the network address of {subnet}"))
        } else if v == bcast {
            Some(format!("{ip} is the broadcast address of {subnet}"))
        } else {
            None
        }
    };
    if let Some(p) = host_problem(net.head_ip) {
        issue("network.headIp".into(), format!("head IP {p}"));
    }
    let has_gw = !net.gateway.is_unspecified();
    if has_gw {
        if let Some(p) = host_problem(net.gateway) {
            issue("network.gateway".into(), format!("gateway {p}"));
        } else if net.gateway == net.head_ip {
            issue(
                "network.gateway".into(),
                format!("gateway {} is the head IP", net.gateway),
            );
        }
    }
    // The PN controller's DCP pool (FirstIPAddress / LastIPAddress): host addresses of the subnet.
    let mut pool_ok = true;
    for (which, ip) in [("first", net.ip_pool_first), ("last", net.ip_pool_last)] {
        if let Some(p) = host_problem(ip) {
            pool_ok = false;
            issue(
                "network.ipPool".into(),
                format!(
                    "PN controller IP pool: {which} address {p} (unset, it follows the head subnet)"
                ),
            );
        }
    }
    if pool_ok && u32::from(net.ip_pool_first) > u32::from(net.ip_pool_last) {
        issue(
            "network.ipPool".into(),
            format!(
                "PN controller IP pool: first address {} is after the last {}",
                net.ip_pool_first, net.ip_pool_last
            ),
        );
    }
    if let Some(p) = station_name_problem(&net.controller_station) {
        issue(
            "network.controllerStation".into(),
            format!("station name '{}' {p}", net.controller_station),
        );
    }
    let mut stations: BTreeMap<&str, String> = BTreeMap::new();
    stations.insert(&net.controller_station, "the head PLC".into());
    let mut ips: BTreeMap<Ipv4Addr, String> = BTreeMap::new();
    for r in robots {
        let f = |k: &str| format!("robots.{}.{k}", r.ns);
        if let Some(p) = station_name_problem(r.station) {
            issue(f("station"), format!("station name '{}' {p}", r.station));
        }
        if let Some(other) = stations.insert(r.station, format!("robot {}", r.ns)) {
            issue(
                f("station"),
                format!("station name '{}' is also used by {other}", r.station),
            );
        }
        if let Some(p) = host_problem(r.ip) {
            issue(f("ip"), format!("IP {p}"));
        } else if r.ip == net.head_ip {
            issue(f("ip"), format!("IP {} is the head PLC IP", r.ip));
        } else if has_gw && r.ip == net.gateway {
            issue(f("ip"), format!("IP {} is the gateway", r.ip));
        }
        if let Some(other) = ips.insert(r.ip, format!("robot {}", r.ns)) {
            issue(f("ip"), format!("IP {} is also used by {other}", r.ip));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn station_names() {
        for ok in [
            "kuka-robi1",
            "controller",
            "a",
            "plc.cell-1.line2",
            "port-01",
            "port-0011",
            "port-001x",
            "x.port-001",
            "1.2.3",
            "1.2.3.4.5",
            "1.2.3.a",
            "1234.2.3.4",
        ] {
            assert_eq!(station_name_problem(ok), None, "{ok}");
        }
        let long_label = "a".repeat(64);
        let long_name = [
            "a".repeat(60),
            "b".repeat(60),
            "c".repeat(60),
            "d".repeat(60),
        ]
        .join(".");
        assert_eq!(long_name.len(), 243);
        for bad in [
            "",
            "Kuka",
            "kuka_robi1",
            "kuka robi",
            "-kuka",
            "kuka-",
            "a.-b",
            "a-.b",
            "a..b",
            ".a",
            "a.",
            "192.168.100.11",
            "0.0.0.0",
            "port-001",
            "port-001-00001",
            "port-001.kuka",
            "ä",
            long_label.as_str(),
            long_name.as_str(),
        ] {
            assert!(station_name_problem(bad).is_some(), "{bad}");
        }
        let max_label = "a".repeat(63);
        assert_eq!(station_name_problem(&max_label), None);
    }

    #[test]
    fn device_names() {
        for ok in ["kuka_robi1", "weld_r1", "R1_dev", "PLC_PRG2", "Devices"] {
            assert_eq!(device_name_problem(ok), None, "{ok}");
        }
        for bad in [
            "",
            "1kuka",
            "kuka-robi",
            "a b",
            "PLC_PRG",
            "pn_controller",
            "IF",
            "end_if",
            "int",
            "TON",
            "GVL_IO",
            "gvl_cell",
            "MainTask",
            "Device",
            "ethernet",
            "VASS_DIO256_R1",
            "safeio_fdio64_x",
            "FB_Cell",
            "a__b",
            "robot_",
        ] {
            assert!(device_name_problem(bad).is_some(), "{bad}");
        }
        assert!(device_name_problem(&"a".repeat(65)).is_some());
    }

    #[test]
    fn every_template_object_is_a_reserved_device_name() {
        use crate::backend::codesys::templates::{DEVICE, HEAD};
        for (_, name, _, _) in HEAD.iter().chain(DEVICE.iter()) {
            assert!(
                RESERVED_DEVICE_NAMES
                    .iter()
                    .any(|r| r.eq_ignore_ascii_case(name)),
                "template object {name} is missing from RESERVED_DEVICE_NAMES"
            );
        }
    }

    fn net() -> NetworkCfg {
        NetworkCfg {
            gateway: Ipv4Addr::new(192, 168, 100, 1),
            ..NetworkCfg::default()
        }
    }

    fn d<'a>(ns: &'a str, station: &'a str, ip: [u8; 4]) -> DeviceAddr<'a> {
        DeviceAddr {
            ns,
            station,
            ip: Ipv4Addr::from(ip),
        }
    }

    #[test]
    fn valid_cell_has_no_issues() {
        let r = [
            d("robot1", "kuka-robi1", [192, 168, 100, 11]),
            d("robot2", "kuka-robi2", [192, 168, 100, 12]),
        ];
        assert_eq!(check(&net(), &r), vec![]);
        assert_eq!(check(&NetworkCfg::default(), &r), vec![]);
    }

    #[test]
    fn every_problem_is_listed() {
        let r = [
            d("robot1", "kuka-robi1", [192, 168, 100, 0]),
            d("robot2", "kuka-robi1", [192, 168, 100, 255]),
            d("robot3", "controller", [192, 168, 100, 1]),
            d("robot4", "Bad", [192, 168, 101, 4]),
            d("robot5", "r5", [192, 168, 100, 10]),
            d("robot6", "r6", [192, 168, 100, 10]),
        ];
        let issues = check(&net(), &r);
        let fields: Vec<&str> = issues.iter().map(|i| i.field.as_str()).collect();
        assert_eq!(
            fields,
            [
                "robots.robot1.ip",
                "robots.robot2.station",
                "robots.robot2.ip",
                "robots.robot3.station",
                "robots.robot3.ip",
                "robots.robot4.station",
                "robots.robot4.ip",
                "robots.robot5.ip",
                "robots.robot6.ip",
                "robots.robot6.ip",
            ],
            "{issues:#?}"
        );
        let m = |i: usize| issues[i].message.as_str();
        assert!(m(0).contains("network address"), "{}", m(0));
        assert!(m(1).contains("also used by robot robot1"), "{}", m(1));
        assert!(m(2).contains("broadcast"), "{}", m(2));
        assert!(m(3).contains("the head PLC"), "{}", m(3));
        assert!(m(4).contains("gateway"), "{}", m(4));
        assert!(
            m(6).contains("outside the head subnet 192.168.100.0/24"),
            "{}",
            m(6)
        );
        assert!(m(7).contains("head PLC IP"), "{}", m(7));
        assert!(m(9).contains("also used by robot robot5"), "{}", m(9));
    }

    #[test]
    fn the_ip_pool_follows_the_head_subnet() {
        // Unset in the config: derived from head_ip / netmask.
        let n: NetworkCfg =
            toml::from_str("head_ip = \"10.1.0.20\"\ngateway = \"10.1.0.1\"\n").unwrap();
        assert_eq!(n.ip_pool_first, Ipv4Addr::new(10, 1, 0, 2));
        assert_eq!(n.ip_pool_last, Ipv4Addr::new(10, 1, 0, 254));
        assert_eq!(check(&n, &[]), vec![]);
        let n: NetworkCfg =
            toml::from_str("head_ip = \"10.1.0.20\"\nnetmask = \"255.255.0.0\"\n").unwrap();
        assert_eq!(
            (n.ip_pool_first, n.ip_pool_last),
            (Ipv4Addr::new(10, 1, 0, 2), Ipv4Addr::new(10, 1, 255, 254))
        );
        // The default is the old fixed pool.
        let d: NetworkCfg = toml::from_str("").unwrap();
        assert_eq!(d, NetworkCfg::default());
        // Set: kept, and checked.
        let n: NetworkCfg = toml::from_str(
            "head_ip = \"10.1.0.20\"\nip_pool_first = \"192.168.100.2\"\nip_pool_last = \"192.168.100.254\"\n",
        )
        .unwrap();
        let issues = check(&n, &[]);
        assert_eq!(issues.len(), 2, "{issues:#?}");
        assert!(issues.iter().all(|i| i.field == "network.ipPool"));
        assert!(
            issues[0]
                .message
                .contains("outside the head subnet 10.1.0.0/24"),
            "{}",
            issues[0].message
        );
        let mut n = net();
        n.ip_pool_first = Ipv4Addr::new(192, 168, 100, 200);
        n.ip_pool_last = Ipv4Addr::new(192, 168, 100, 100);
        assert_eq!(check(&n, &[])[0].field, "network.ipPool");
    }

    #[test]
    fn network_rules() {
        let mut n = net();
        n.netmask = Ipv4Addr::new(255, 0, 255, 0);
        assert_eq!(check(&n, &[])[0].field, "network.netmask");
        let mut n = net();
        n.gateway = Ipv4Addr::new(10, 0, 0, 1);
        n.controller_station = "port-123".into();
        let f: Vec<String> = check(&n, &[]).into_iter().map(|i| i.field).collect();
        assert_eq!(f, ["network.gateway", "network.controllerStation"]);
        let mut n = net();
        n.head_ip = Ipv4Addr::new(192, 168, 100, 255);
        assert_eq!(check(&n, &[])[0].field, "network.headIp");
        let mut n = net();
        n.gateway = n.head_ip;
        assert_eq!(check(&n, &[])[0].field, "network.gateway");
    }
}
