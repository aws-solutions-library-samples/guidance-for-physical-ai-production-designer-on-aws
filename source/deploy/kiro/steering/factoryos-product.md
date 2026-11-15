# Physical AI Production Designer: products and spot welds (product_*)

* Frame: the product's own, metres, z up; a box stands on its frame's origin (the centre of its bottom face). A weld is
  a point operation with the OUTWARD surface normal (the gun works against it), an id (W01, …), a group (welds of one
  group are done together, e.g. `structural` before `secondary`) and optional parameter overrides (weldTime s,
  force kN, current kA, approachOffset m, priority).
* Read the product first (product_get): its geometry (box size, mesh, URDF) and tasks. A new product: product_save with
  geometry and one spotWeld task (tool: the spot welding gun of the dresser catalogue).
* Adding welds where they make sense:
  * Box parts: product_generate_welds on a face (rows × columns, or a pitch, with a margin from the edges).
  * Any part: product_snap_point first to see where points land (surface point, normal), then product_add_welds
    (snap true, the default). Put welds on flanges and overlaps, at least ~20 mm apart (min pitch), ≥ 10 mm from free
    edges, never on the bottom face the product stands on.
  * Moving or re-orienting: product_move_weld (position or delta; snap keeps it on the surface).
  * Grouping and order: product_regroup_welds (group, priority).
  * Removing: product_remove_welds (ids; ["*"] all of a task).
* Every edit saves the product, answers the issues at the welds it touched, and the cells using the product get their
  task programs derived again (planned plans become stale). Finish with product_check_welds; fix what it flags
  (inside_part, off_surface, normal_mismatch, too_close, near_edge, duplicate_id).
* product_preview shows unsaved edits in the Product & Task 3D view; view_camera page product to point at a weld.
