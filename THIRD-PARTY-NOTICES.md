# Third-party notices

This Guidance builds on the open-source software below, each under its own licence. The table says
where each one comes in and how its licence was confirmed: from the component's own licence file or package metadata
(**checked**), or only from the project's published licence (**not checked locally**: to be confirmed against the
exact version before the release).

Paths are relative to this repository's root; the runnable tree is under `source/`.

Not listed one by one: the Rust crates and npm packages the builds resolve (`source/Cargo.lock`, `source/cell/Cargo.lock`,
`source/plan2cell/Cargo.lock`, `source/web/package-lock.json`, `deployment/package-lock.json`) and the Python packages of `source/deploy/requirements-*.txt`. Their full
licence inventory is generated from those lock files for a release, not kept here by hand.

## In this repository

| Component | Where | Licence | Confirmed |
|---|---|---|---|
| JetBrains Mono (font) | `source/plan2cell/crates/bt-view/assets/fonts/JetBrainsMono-Regular.ttf` (the viewer's and btgui's) | SIL Open Font License 1.1 | checked: the font's name table, `JetBrainsMono-OFL.txt` next to it |
| [Noto Emoji](https://github.com/googlefonts/noto-emoji) 1.05 (font) | `source/plan2cell/crates/bt-view/assets/fonts/NotoEmoji-Regular.ttf` (the viewer's and btgui's symbols) | SIL Open Font License 1.1 | checked: the font's name table (copyright Google Inc., OFL 1.1), `NotoEmoji-OFL.txt` next to it; byte-identical to `fonts/NotoEmoji-Regular.ttf` of the crate epaint_default_fonts 0.36.2 (sha256 `415dc6290378574135b64c808dc640c1df7531973290c4970c51fdeb849cb0c5`) |
| [emoji-icon-font](https://github.com/jslegers/emoji-icon-font) 1.1 (font) | `source/plan2cell/crates/bt-view/assets/fonts/emoji-icon-font.ttf` (the viewer's and btgui's symbols) | MIT | checked: `LICENSE-emoji-icon-font.txt` next to it, the licence file epaint_default_fonts 0.36.2 ships with the font (its name table names none); byte-identical to that crate's `fonts/emoji-icon-font.ttf` (sha256 `f426bec371f484646d002717e2c555a95cd5145e635d83a48bbef83f3abf18ca`) |
| Space Grotesk (font) | `source/crates/factoryos-viewer/assets/fonts/SpaceGrotesk-Medium.ttf` | SIL Open Font License 1.1 | checked: the font's name table, `SpaceGrotesk-OFL.txt` next to it |
| urdf-robo-dresser, robo-fitting-room | `source/crates/urdf-robo-dresser/`, `source/crates/robo-fitting-room/` | MIT-0 | checked: their `LICENSE` files (from aws-samples/sample-robotic-cellsim-tools, see `source/docs/UPSTREAM.md`) |
| The Rerun patches | `source/third_party/rerun/patches/` | MIT OR Apache-2.0, as Rerun | checked: Rerun 0.38.1's `Cargo.toml` and `LICENSE-MIT` / `LICENSE-APACHE` |
| [three.js](https://github.com/mrdoob/three.js) r0.169.0 (`three.module.js` and the `examples/jsm` controls, loaders and helpers) | `source/crates/urdf-robo-dresser/web/vendor/three/` | MIT | checked: `three.module.js`'s SPDX header (revision 169), byte-identical to three-0.169.0.tgz (sha512 verified), `LICENSE` in `vendor/three/`, sha256 per file in `source/crates/urdf-robo-dresser/web/vendor/VENDOR.md` |
| [urdf-loader](https://github.com/gkjohnson/urdf-loaders) 0.12.6 (`URDFLoader.js`, `URDFClasses.js`, `URDFDragControls.js`) | `source/crates/urdf-robo-dresser/web/vendor/urdf-loader/` | Apache-2.0 | from `source/crates/urdf-robo-dresser/web/vendor/VENDOR.md` (the files carry no licence header); checked: byte-identical to urdf-loader-0.12.6.tgz (sha512 verified); the package ships no LICENSE file, its package.json says Apache-2.0 |
| [js-yaml](https://github.com/nodeca/js-yaml) 4.1.1 (`js-yaml.min.js`) | `source/crates/robo-fitting-room/web/vendor/js-yaml/` (the Robo Fitter's page, served locally instead of from a CDN) | MIT | checked: npm tarball js-yaml-4.1.1.tgz against the registry's sha512, the file's header `/*! js-yaml 4.1.1 ... @license MIT */` and its `LICENSE` (`vendor/VENDOR.md`) |

## Fetched or linked by the build

| Component | Used for | Licence | Confirmed |
|---|---|---|---|
| [Rerun](https://github.com/rerun-io/rerun) 0.38.1 | the viewer (`source/scripts/fetch-rerun.sh` clones `source/third_party/rerun/PIN` and applies the patches) | MIT OR Apache-2.0 | checked: its `Cargo.toml`, `LICENSE-MIT`, `LICENSE-APACHE` |
| [Inter](https://github.com/rsms/inter) (font) | the viewer's text font, compiled in from Rerun's `re_ui` | SIL Open Font License 1.1 | checked: `OFL.txt` next to `Inter-Medium.otf` in Rerun 0.38.1 |
| [egui / eframe](https://github.com/emilk/egui) 0.36 | the viewer's and plan2cell's user interface, without egui's `default_fonts` (the crate epaint_default_fonts and its Ubuntu-Light font are not used: the fonts above instead) | MIT OR Apache-2.0 | checked: the crates' `Cargo.toml` |
| [wgpu](https://github.com/gfx-rs/wgpu) 30 | rendering | MIT OR Apache-2.0 | checked: the crate's `Cargo.toml` |
| [Zenoh](https://github.com/eclipse-zenoh/zenoh) 1.10.1 (backend, viewer), 1.7.2 (plan2cell) | the data bus | EPL-2.0 OR Apache-2.0 | checked: the crates' `Cargo.toml` |
| [zenoh-ts](https://github.com/eclipse-zenoh/zenoh-ts) 1.10.1 | the web viewer's bus client (`source/web/`) | Apache-2.0 OR EPL-2.0 | checked: its `package.json` |
| [trust-platform](https://github.com/johannesPettersson80/trust-platform) @ `ecbcb08` (v0.24.70) | plan2cell: parsing and type-checking the generated Structured Text, the PLC runtime of the co-simulation | MIT OR Apache-2.0 | checked: its workspace `Cargo.toml`, `LICENSE-MIT`, `LICENSE-APACHE` |
| [open-ot-experiments](https://github.com/johannesPettersson80/open-ot-experiments) @ `137f0e7` | plan2cell, through trust-platform | Apache-2.0 | checked: its `LICENSE` and README |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | the cell runtime's kinematics (C++), the planners' (Python `pin`) | BSD-2-Clause | checked: `LICENSE` of Pinocchio 3.9.0 |
| [Coal](https://github.com/coal-library/coal) (formerly hpp-fcl) | collision checks | BSD-3-Clause | checked: `LICENSE` of Coal 3.0.3 |
| [Boost](https://www.boost.org) | Pinocchio's and Coal's dependency, bundled with the cell runtime | BSL-1.0 | checked: the headers' licence notice, the `cmeel-boost` wheel's metadata |
| [Eigen](https://eigen.tuxfamily.org) | linear algebra (Pinocchio, Coal) | MPL-2.0 | checked: `COPYING.README` of Eigen 5.0.1 |
| [urdfdom](https://github.com/ros/urdfdom), console_bridge | URDF parsing (Pinocchio) | BSD-3-Clause | checked: their `LICENSE` files |
| [TinyXML-2](https://github.com/leethomason/tinyxml2) | XML parsing (urdfdom) | Zlib | checked: its `LICENSE.txt` |
| [rsruckig](https://crates.io/crates/rsruckig) | the cell runtime's online trajectory generation (a Rust port of Ruckig) | MIT | checked: the crate's `Cargo.toml` |
| [behaviortree-rs](https://crates.io/crates/behaviortree-rs) | the cell runtime's behaviour-tree executor (BehaviorTree.CPP v4 XML) | MIT | checked: the crate's `Cargo.toml` |
| [nalgebra](https://nalgebra.org) | the cell runtime's linear algebra | Apache-2.0 | checked: the crate's `Cargo.toml` |
| [urdf-rs](https://github.com/openrr/urdf-rs) | URDF parsing (Rust) | Apache-2.0 | checked: the crate's `Cargo.toml` |
| [PyTorch](https://pytorch.org) | running the learned policies | BSD-3-Clause | not checked locally |
| [PyTorch Geometric](https://github.com/pyg-team/pytorch_geometric) | the policies' graph networks | MIT | not checked locally |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | the policies' simulation environment | Apache-2.0 | not checked locally |
| [Gymnasium](https://github.com/Farama-Foundation/Gymnasium) | the policies' environment interface | MIT | not checked locally |
| [NumPy](https://numpy.org), [SciPy](https://scipy.org) | numerics | BSD-3-Clause | checked: the packages' `LICENSE.txt` |
| [trimesh](https://github.com/mikedh/trimesh) | mesh handling | MIT | checked: the package's metadata |
| [PyYAML](https://pyyaml.org) | configuration files | MIT | checked: the package's metadata |
| [OMPL](https://ompl.kavrakilab.org) | motion planning (umarell) | BSD-3-Clause | checked: the package's metadata |
| [eclipse-zenoh](https://github.com/eclipse-zenoh/zenoh-python) (Python) | the planners' bus client | Apache-2.0 OR EPL-2.0 | checked: the package's metadata |
| [AWS CDK](https://github.com/aws/aws-cdk) (`aws-cdk-lib` 2.275.0, the `aws-cdk` CLI 2.1148.0), [constructs](https://github.com/aws/constructs) 10.8.1 | the deployment's AWS CDK app (`deployment/`) | Apache-2.0 | checked: the packages' `package.json` |
| [cdk-nag](https://github.com/cdklabs/cdk-nag) 3.0.2 | the deployment's rule checks at synth | Apache-2.0 | checked: the package's `package.json` |

## Installed at deploy time, not part of this repository

| Component | Used for | Licence | Confirmed |
|---|---|---|---|
| [umarell](https://github.com/aws-samples/sample-umarell) @ `08d5a39` (`source/deploy/umarell.pin`), with the upstream fixes of `source/cell-planner/docs/upstream/patches_08d5a39/` | the agentic planner and its planning tools (U3) | as published in that repository | not checked locally: to be confirmed when it is public |
| The umarell patches of `source/deploy/umarell-public/` | applied to umarell's chat tree at deploy time (the trajectory MCP server on loopback) | as umarell | to be confirmed with umarell |
| Kiro CLI 2.26.1 (`KIRO_VERSION` in `source/deploy/provision.sh`) | the agentic planner's agent, installed on the instance (checked by its sha256) | Kiro's terms ([kiro.dev](https://kiro.dev)); not redistributed | — |
| Ubuntu 24.04 LTS (arm64), from Canonical's public AMI | the instance's operating system and its apt packages | the licences of its packages (Ubuntu's terms); not redistributed | — |
| Rust (rustup and the toolchains `source/rust-toolchain.toml` names), wasm-bindgen-cli, binaryen (`wasm-opt`), Node.js | the release build on the instance (`deployment/bootstrap/install-build-deps.sh`, each checked by its sha256 or pinned version) | MIT OR Apache-2.0 (Rust, wasm-bindgen); Apache-2.0 (binaryen); MIT (Node.js); not redistributed | not checked locally |
| AWS CLI v2, Amazon CloudWatch agent | the instance's access to its stack (release state, logs) | Apache-2.0 (AWS CLI); MIT (CloudWatch agent); not redistributed | not checked locally |

## Models and data

The robot and tool meshes in `source/assets/` and the policy checkpoints in `source/cell-planner/runs/` are part of this sample;
their provenance is being confirmed before the publication.
