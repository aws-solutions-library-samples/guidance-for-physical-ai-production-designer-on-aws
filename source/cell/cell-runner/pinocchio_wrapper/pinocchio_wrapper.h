#ifndef PINOCCHIO_WRAPPER_H
#define PINOCCHIO_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Opaque handle to a loaded Pinocchio model + data.
typedef struct PinocchioContext PinocchioContext;

/// Load a URDF model. Returns NULL on failure, sets error_msg (caller must free with pinocchio_free_string).
PinocchioContext* pinocchio_load_urdf(const char* urdf_path, char** error_msg);

/// Compute forward kinematics. Writes 6 values to `pose_out` [x,y,z,a,b,c] (mm, degrees).
/// Returns 0 on success, -1 on failure.
int pinocchio_fk(PinocchioContext* ctx, const double* joint_positions, double* pose_out);

/// Solve inverse kinematics. `target_pose` is [x,y,z,a,b,c] (mm, degrees).
/// Writes result to `joints_out`. Returns 0 on success, -1 on failure (sets error_msg).
int pinocchio_ik_solve(
    PinocchioContext* ctx,
    const double* current_joints,
    const double* target_pose,
    uint32_t max_iterations,
    double tolerance,
    double* joints_out,
    char** error_msg
);

/// Free a loaded model context.
void pinocchio_free(PinocchioContext* ctx);

/// Free an error message string.
void pinocchio_free_string(char* s);

/* ------------------------------------------------------------------------- */
/* Collision detection (Pinocchio GeometryModel + Coal backend)              */
/* ------------------------------------------------------------------------- */

/// Opaque handle to a loaded collision model (Model + GeometryModel + Data + GeometryData).
typedef struct CollisionContext CollisionContext;

/// One reported colliding (or nearest) object pair. Names are borrowed pointers
/// into the context and remain valid until collision_free(ctx).
typedef struct {
    int   geom_a;            // GeometryObject index (first)
    int   geom_b;            // GeometryObject index (second)
    const char* name_a;      // GeometryObject name (first)
    const char* name_b;      // GeometryObject name (second)
    double distance;         // <0 or 0 => penetrating; >0 => separation (metres)
} CollisionPairOut;

/// Load a collision model from URDF <collision> geometry. Optional SRDF path
/// disables known-safe self-collision pairs (adjacent links, permanent contacts).
/// Returns NULL on failure and sets error_msg (free with pinocchio_free_string).
CollisionContext* collision_load_urdf(const char* urdf_path,
                                      const char* srdf_path /* nullable */,
                                      char** error_msg);

/// Number of collision pairs registered on the model (after SRDF filtering).
int collision_num_pairs(const CollisionContext* ctx);

/// Check collisions for a joint configuration q (radians, length model.nq).
/// Writes up to max_out colliding pairs into out; sets *num_out to the count.
/// If stop_at_first != 0, returns after the first detected collision.
/// Returns 1 if any collision found, 0 if clear, -1 on error (sets error_msg).
int collision_check(CollisionContext* ctx,
                    const double* q,
                    int stop_at_first,
                    CollisionPairOut* out,
                    int max_out,
                    int* num_out,
                    char** error_msg);

/// Minimum separation distance across all pairs for configuration q (metres).
/// Writes the closest pair into *out when out != NULL. Returns 0 on success.
int collision_min_distance(CollisionContext* ctx,
                           const double* q,
                           CollisionPairOut* out,
                           double* min_distance_out,
                           char** error_msg);

/// Forward kinematics of every collision geometry object placement in the
/// model's base frame, as 7-tuples [x,y,z (m), qx,qy,qz,qw] written to
/// placements_out (length 7 * num objects). Used by the scene layer to compose
/// world-frame cross-model checks. Returns 0 on success.
int collision_geometry_placements(CollisionContext* ctx,
                                  const double* q,
                                  double* placements_out,
                                  int* num_objects_out,
                                  char** error_msg);

/// Local-frame axis-aligned half-extents of every collision geometry object,
/// as [hx, hy, hz] (m) triples written to half_extents_out (length 3 * num
/// objects), in the same registration order as collision_geometry_placements.
/// This is a static property of the loaded geometry (independent of joint
/// configuration) and is therefore computed once, not per-configuration.
/// Every component is guaranteed strictly positive: a degenerate or
/// unavailable axis is substituted with a small positive epsilon (1e-4 m)
/// rather than reported as zero. Returns 0 on success.
int collision_geometry_half_extents(CollisionContext* ctx,
                                    double* half_extents_out,
                                    int* num_objects_out,
                                    char** error_msg);

/// Centre of every collision geometry object's local-frame axis-aligned
/// bounding box, as [cx, cy, cz] (m) triples in the object's own frame
/// (length 3 * num objects), in the same registration order as
/// collision_geometry_placements. A mesh is rarely centred on its link frame,
/// so the box of collision_geometry_half_extents sits at
/// placement * translation(centre). A missing geometry reports [0, 0, 0].
/// Returns 0 on success.
int collision_geometry_local_centers(CollisionContext* ctx,
                                     double* centers_out,
                                     int* num_objects_out,
                                     char** error_msg);

/// Free a collision context.
void collision_free(CollisionContext* ctx);

#ifdef __cplusplus
}
#endif

#endif
