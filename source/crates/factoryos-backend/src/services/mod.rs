//! The subsystems behind the backend. Each one contributes HTTP routes (`router()`) and, where it
//! has live data, Zenoh publishers/queryables. Add new subsystems (robo-ros-runner, plc-runner,
//! trajectory, NICE DCV, …) as a module here and merge its router in `main`.

pub mod agent_login;
pub mod agentic;
pub mod asset_register;
pub mod assets;
pub mod cell;
pub mod cell_migrate;
pub mod cell_resolve;
pub mod cell_wiring;
pub mod cells;
pub mod chat;
pub mod codegen;
pub mod dcv;
pub mod dresser;
pub mod files;
pub mod fitter;
pub mod hardware;
pub mod infra;
pub mod mesh_convert;
pub mod process_def;
pub mod product;
pub mod product_tasks;
pub mod product_welds;
pub mod reach;
pub mod orient_sweep;
pub mod planner_model;
pub mod pose3;
pub mod rtc;
pub mod scene_pub;
pub mod scene_edit;
pub mod scene_migrate;
pub mod scene_paths;
pub mod scene_store;
pub mod smartpad;
pub mod tcp;
pub mod terminal;
pub mod training;
pub mod urdf_model;
pub mod view_cmd;
