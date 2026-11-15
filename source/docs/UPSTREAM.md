# Where the imported code came from

Factory OS builds everything the server runs from this repository. Code that started elsewhere was
copied in once; from the import on, it is developed here.

| Here | Came from | Notes |
|---|---|---|
| `crates/urdf-robo-dresser/` | [aws-samples/sample-robotic-cellsim-tools](https://github.com/aws-samples/sample-robotic-cellsim-tools) @ `8db4945`, `urdf-robo-dresser` | License MIT-0 (`LICENSE`). `tests/requirements.rs` is a spec with `todo!()`s, excluded from the default `cargo test`. |
| `crates/robo-fitting-room/` | the same, `robo-fitting-room` | As above. |
| `crates/factoryos-urdf-materials/src/materials/` | an earlier, separate crate of this project | Merged into the crate it was an optional dependency of. Its rerun patch is `third_party/rerun/patches/rerun-override-builtin-importer.patch`. |
| `cell/cell-runner/`, `cell/krl-parser/` | an internal snapshot of the robotic cell code runner (its cell runner and its lenient KRL parser) | The parts the cell runtime uses. |
| `cell/cell-rerun-server/` | the same snapshot | Assembled by `cell/scripts/build-cell-rerun-server.sh`. |
| `plan2cell/crates/kuka-krl-parser/` | the same KRL parser | So that bt2krl's parser smoke check builds from this repository alone. |

Not imported, and why:

- **rerun** ([rerun-io/rerun](https://github.com/rerun-io/rerun)): `scripts/fetch-rerun.sh` clones the release in
  `third_party/rerun/PIN` and applies `third_party/rerun/patches/`.
- **umarell** (the agentic planner, [aws-samples/sample-umarell](https://github.com/aws-samples/sample-umarell)): an
  external Python/ROS project used at runtime only, installed on the server at the commit in `deploy/umarell.pin`.

`THIRD-PARTY-NOTICES.md` lists the third-party components and their licences.
