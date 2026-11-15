//! trust-platform integration: type-check the generated ST (`check`) and co-simulate the
//! head PLC with the robot simulators (`cosim`, feature `sim`).

#[cfg(feature = "check")]
pub mod check;
#[cfg(feature = "sim")]
pub mod cosim;
