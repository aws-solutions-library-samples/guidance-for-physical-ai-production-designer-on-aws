#include "pinocchio_wrapper.h"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

// Collision support is compiled only when the `collision` Cargo feature is
// enabled (build.rs -> CMake -DENABLE_COLLISION=ON -> PINOCCHIO_WRAPPER_COLLISION).
// The default build never sees any of the collision headers or symbols.
#ifdef PINOCCHIO_WRAPPER_COLLISION
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <cstddef>
#include <fstream>
#include <limits>
#endif

/// Internal context holding the Pinocchio model, data, and end-effector frame ID.
struct PinocchioContext {
    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex ee_frame_id;
};

/// Helper: duplicate a std::string as a C string (caller frees with pinocchio_free_string).
static char* dup_error(const std::string& msg) {
    return strdup(msg.c_str());
}

/// Convert radians to degrees.
static inline double rad2deg(double r) { return r * (180.0 / M_PI); }

/// Convert degrees to radians.
static inline double deg2rad(double d) { return d * (M_PI / 180.0); }

/// Convert metres to millimetres.
static inline double m2mm(double m) { return m * 1000.0; }

/// Convert millimetres to metres.
static inline double mm2m(double mm) { return mm / 1000.0; }

/// Extract ZYX Euler angles (in radians) from a rotation matrix.
/// Returns (rz, ry, rx) matching KUKA A, B, C convention.
static Eigen::Vector3d rotation_to_zyx_euler(const Eigen::Matrix3d& R) {
    // ZYX decomposition: R = Rz(a) * Ry(b) * Rx(c)
    double sy = -R(2, 0);
    double cy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));

    double a, b, c;
    if (cy > 1e-6) {
        a = std::atan2(R(1, 0), R(0, 0));
        b = std::atan2(sy, cy);
        c = std::atan2(R(2, 1), R(2, 2));
    } else {
        // Gimbal lock
        a = std::atan2(-R(0, 1), R(1, 1));
        b = std::atan2(sy, cy);
        c = 0.0;
    }
    return Eigen::Vector3d(a, b, c);
}

/// Build a rotation matrix from ZYX Euler angles (in radians).
/// a = Rz, b = Ry, c = Rx (KUKA A, B, C convention).
static Eigen::Matrix3d zyx_euler_to_rotation(double a, double b, double c) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(c, Eigen::Vector3d::UnitX());
    return R;
}

// ---------------------------------------------------------------------------
// Public C API
// ---------------------------------------------------------------------------

extern "C" {

PinocchioContext* pinocchio_load_urdf(const char* urdf_path, char** error_msg) {
    try {
        auto* ctx = new PinocchioContext();
        pinocchio::urdf::buildModel(std::string(urdf_path), ctx->model);
        ctx->data = pinocchio::Data(ctx->model);

        if (ctx->model.nframes < 2) {
            if (error_msg) {
                *error_msg = dup_error("URDF model has no operational frames");
            }
            delete ctx;
            return nullptr;
        }
        // Use tool0 frame — matches KUKA flange convention and reference project
        ctx->ee_frame_id = static_cast<pinocchio::FrameIndex>(ctx->model.nframes - 1);
        for (size_t i = 0; i < ctx->model.nframes; ++i) {
            if (ctx->model.frames[i].name == "tool0") {
                ctx->ee_frame_id = static_cast<pinocchio::FrameIndex>(i);
                break;
            }
        }

        return ctx;
    } catch (const std::exception& e) {
        if (error_msg) {
            *error_msg = dup_error(std::string("Failed to load URDF: ") + e.what());
        }
        return nullptr;
    }
}

int pinocchio_fk(PinocchioContext* ctx, const double* joint_positions, double* pose_out) {
    if (!ctx || !joint_positions || !pose_out) return -1;

    try {
        const int nq = ctx->model.nq;
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(joint_positions, nq);

        pinocchio::forwardKinematics(ctx->model, ctx->data, q);
        pinocchio::updateFramePlacements(ctx->model, ctx->data);

        const pinocchio::SE3& oMf = ctx->data.oMf[ctx->ee_frame_id];
        Eigen::Vector3d pos = oMf.translation();
        Eigen::Vector3d euler = rotation_to_zyx_euler(oMf.rotation());

        // Output: [x, y, z] in mm, [a, b, c] in degrees
        pose_out[0] = m2mm(pos(0));
        pose_out[1] = m2mm(pos(1));
        pose_out[2] = m2mm(pos(2));
        pose_out[3] = rad2deg(euler(0)); // A
        pose_out[4] = rad2deg(euler(1)); // B
        pose_out[5] = rad2deg(euler(2)); // C

        return 0;
    } catch (...) {
        return -1;
    }
}

int pinocchio_ik_solve(
    PinocchioContext* ctx,
    const double* current_joints,
    const double* target_pose,
    uint32_t max_iterations,
    double tolerance,
    double* joints_out,
    char** error_msg
) {
    if (!ctx || !current_joints || !target_pose || !joints_out) {
        if (error_msg) *error_msg = dup_error("null pointer argument");
        return -1;
    }

    try {
        const int nq = ctx->model.nq;
        const int nv = ctx->model.nv;

        // Convert target pose from mm/degrees to m/radians
        Eigen::Vector3d target_pos(
            mm2m(target_pose[0]),
            mm2m(target_pose[1]),
            mm2m(target_pose[2])
        );
        Eigen::Matrix3d target_rot = zyx_euler_to_rotation(
            deg2rad(target_pose[3]),
            deg2rad(target_pose[4]),
            deg2rad(target_pose[5])
        );

        pinocchio::SE3 oMdes(target_rot, target_pos);

        // Initial joint configuration (seed)
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(current_joints, nq);

        // Damped least-squares IK parameters
        const double damp = 1e-6;
        double residual = 0.0;

        for (uint32_t i = 0; i < max_iterations; ++i) {
            // Compute FK and frame placements
            pinocchio::forwardKinematics(ctx->model, ctx->data, q);
            pinocchio::updateFramePlacements(ctx->model, ctx->data);

            // Compute pose error in SE3 (6D: [linear; angular])
            const pinocchio::SE3& oMf = ctx->data.oMf[ctx->ee_frame_id];
            pinocchio::SE3 err_se3 = oMf.actInv(oMdes);
            Eigen::Matrix<double, 6, 1> err;
            err.head<3>() = err_se3.translation();
            err.tail<3>() = pinocchio::log3(err_se3.rotation());

            residual = err.norm();
            if (residual < tolerance) {
                // Converged — write result
                Eigen::Map<Eigen::VectorXd>(joints_out, nq) = q;
                return 0;
            }

            // Compute frame Jacobian in local frame
            Eigen::MatrixXd J(6, nv);
            J.setZero();
            pinocchio::computeFrameJacobian(
                ctx->model, ctx->data, q, ctx->ee_frame_id,
                pinocchio::LOCAL, J
            );

            // Damped least-squares: dq = J^T (J J^T + λ I)^{-1} err
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt.diagonal().array() += damp;
            Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);

            // Clamp max step to 1.0 rad per joint to prevent oscillation
            double max_dq = dq.lpNorm<Eigen::Infinity>();
            if (max_dq > 1.0) {
                dq *= 1.0 / max_dq;
            }
            q = pinocchio::integrate(ctx->model, q, dq);
            for (int j = 0; j < nq; ++j) {
                q[j] = std::max(ctx->model.lowerPositionLimit[j],
                        std::min(ctx->model.upperPositionLimit[j], q[j]));
            }
        }

        // Did not converge
        if (error_msg) {
            *error_msg = dup_error(
                "IK did not converge after " + std::to_string(max_iterations) +
                " iterations (residual: " + std::to_string(residual) + ")"
            );
        }
        return -1;
    } catch (const std::exception& e) {
        if (error_msg) {
            *error_msg = dup_error(std::string("IK exception: ") + e.what());
        }
        return -1;
    }
}

void pinocchio_free(PinocchioContext* ctx) {
    delete ctx;
}

void pinocchio_free_string(char* s) {
    free(s);
}

} // extern "C"

/* ========================================================================= */
/* Collision detection (Pinocchio GeometryModel + Coal backend)             */
/*                                                                           */
/* Everything below is compiled only for the `collision` Cargo feature       */
/* (PINOCCHIO_WRAPPER_COLLISION). When the feature is off the default build   */
/* sees no collision headers, types, or symbols at all.                      */
/* ========================================================================= */

#ifdef PINOCCHIO_WRAPPER_COLLISION

/// Internal context holding the Pinocchio kinematic model + data alongside the
/// collision GeometryModel + GeometryData. Names in CollisionPairOut borrow
/// into geom_model.geometryObjects and remain valid until collision_free(ctx).
struct CollisionContext {
    pinocchio::Model model;
    pinocchio::GeometryModel geom_model;
    pinocchio::Data data;
    pinocchio::GeometryData geom_data;
};

/// Helper: true when `path` names a file that exists and can be opened for read.
static bool file_is_readable(const std::string& path) {
    std::ifstream f(path.c_str());
    return f.good();
}

/// Degrees of freedom a collision query configuration must supply. The Rust
/// caller always passes a fixed 6-vector (CollisionModel::check_self takes
/// &[f64;6]); a model whose nq differs is a DOF mismatch and must be rejected
/// before any element of `q` is read (Req 9.5).
static constexpr int COLLISION_EXPECTED_NQ = 6;

extern "C" {

CollisionContext* collision_load_urdf(const char* urdf_path,
                                      const char* srdf_path /* nullable */,
                                      char** error_msg) {
    try {
        // --- URDF path validation ------------------------------------------
        if (!urdf_path || urdf_path[0] == '\0') {
            if (error_msg) {
                *error_msg = dup_error("URDF path is empty");
            }
            return nullptr;
        }
        const std::string urdf(urdf_path);
        if (!file_is_readable(urdf)) {
            if (error_msg) {
                *error_msg = dup_error(
                    "URDF file cannot be read: '" + urdf + "'");
            }
            return nullptr;
        }

        auto* ctx = new CollisionContext();

        // --- Build kinematic model + collision geometry --------------------
        try {
            pinocchio::urdf::buildModel(urdf, ctx->model);
            ctx->data = pinocchio::Data(ctx->model);
            // Mesh references in a dressed URDF are written relative to the URDF
            // itself (e.g. "./meshes/robot/link_1_hull_0.obj"). buildGeom's
            // no-package-dirs overload resolves those against the process CWD (and
            // ROS_PACKAGE_PATH), which fails for every self-contained cell. Pass
            // the URDF's own directory as a package dir so the meshes resolve
            // regardless of where the runner was started from.
            std::vector<std::string> package_dirs;
            const std::size_t slash = urdf.find_last_of("/\\");
            if (slash != std::string::npos) {
                package_dirs.push_back(urdf.substr(0, slash));
            }
            pinocchio::urdf::buildGeom(
                ctx->model, urdf, pinocchio::COLLISION, ctx->geom_model, package_dirs);
        } catch (const std::exception& e) {
            if (error_msg) {
                *error_msg = dup_error(
                    "Failed to parse URDF '" + urdf + "': " + e.what());
            }
            delete ctx;
            return nullptr;
        }

        // Register every candidate self-collision pair. When the URDF carries
        // no <collision> geometry this simply yields zero pairs (Req 1.6): the
        // context loads successfully and collision_num_pairs() returns 0.
        ctx->geom_model.addAllCollisionPairs();

        // --- Optional SRDF self-collision filtering ------------------------
        // srdf_path == nullptr  => no SRDF supplied, retain all pairs (Req 1.4).
        // srdf_path == ""       => supplied but empty, treat as an error (Req 1.10).
        if (srdf_path) {
            if (srdf_path[0] == '\0') {
                if (error_msg) {
                    *error_msg = dup_error("SRDF path is empty");
                }
                delete ctx;
                return nullptr;
            }
            const std::string srdf(srdf_path);
            if (!file_is_readable(srdf)) {
                if (error_msg) {
                    *error_msg = dup_error(
                        "SRDF file cannot be read: '" + srdf + "'");
                }
                delete ctx;
                return nullptr;
            }
            try {
                pinocchio::srdf::removeCollisionPairs(
                    ctx->model, ctx->geom_model, srdf, false);
            } catch (const std::exception& e) {
                if (error_msg) {
                    *error_msg = dup_error(
                        "Failed to parse SRDF '" + srdf + "': " + e.what());
                }
                delete ctx;
                return nullptr;
            }
        }

        // GeometryData reflects the (post-SRDF) collision pair set.
        ctx->geom_data = pinocchio::GeometryData(ctx->geom_model);

        return ctx;
    } catch (const std::exception& e) {
        if (error_msg) {
            *error_msg = dup_error(
                std::string("Failed to load collision model: ") + e.what());
        }
        return nullptr;
    } catch (...) {
        if (error_msg) {
            *error_msg = dup_error("Failed to load collision model: unknown error");
        }
        return nullptr;
    }
}

int collision_num_pairs(const CollisionContext* ctx) {
    if (!ctx) return 0;
    try {
        return static_cast<int>(ctx->geom_model.collisionPairs.size());
    } catch (...) {
        return 0;
    }
}

void collision_free(CollisionContext* ctx) {
    // Safe on nullptr; releases the context exactly once (Req 10.9).
    delete ctx;
}

/* ------------------------------------------------------------------------- */
/* Deferred query entry points (implemented in tasks 2.4 / 2.5).             */
/* Minimal stubs keep the collision-feature build fully linkable until then; */
/* they never throw across the ABI and report failure rather than acting.    */
/* ------------------------------------------------------------------------- */

int collision_check(CollisionContext* ctx,
                    const double* q,
                    int stop_at_first,
                    CollisionPairOut* out,
                    int max_out,
                    int* num_out,
                    char** error_msg) {
    // Establish the count output up front so every early return leaves it
    // well-defined (Req 10.4, 10.5, 10.7).
    if (num_out) *num_out = 0;

    // --- Argument validation (Req 10.7) --------------------------------
    // A null output buffer or a zero (or negative) capacity is a caller
    // error: return -1, report zero entries, and never touch `out`.
    if (!out || max_out <= 0) {
        if (error_msg) {
            *error_msg = dup_error(
                "collision_check: null output buffer or max_out <= 0");
        }
        return -1;
    }
    if (!ctx || !q) {
        if (error_msg) {
            *error_msg = dup_error(
                "collision_check: null context or configuration pointer");
        }
        return -1;
    }
    if (!num_out) {
        // No place to report the written count; treat as a caller error and
        // do not proceed (we cannot honor the Req 10.1 count contract).
        if (error_msg) {
            *error_msg = dup_error("collision_check: null num_out pointer");
        }
        return -1;
    }

    try {
        // --- DOF validation before indexing q (Req 9.5) ----------------
        // Reject a DOF mismatch *before* reading any element of `q`, both to
        // satisfy the requirement and to prevent an out-of-bounds read of the
        // caller's fixed-length (6) buffer for models with nq != 6.
        const int nq = ctx->model.nq;
        if (nq != COLLISION_EXPECTED_NQ) {
            if (error_msg) {
                *error_msg = dup_error(
                    "collision_check: DOF mismatch (model nq=" +
                    std::to_string(nq) + ", configuration provides " +
                    std::to_string(COLLISION_EXPECTED_NQ) + ")");
            }
            return -1;
        }

        // Now safe to read exactly nq (== 6) configuration values.
        Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q, nq);

        const bool stop = (stop_at_first != 0);

        // Forward kinematics -> geometry placements -> narrowphase collision
        // over the registered (post-SRDF) collision pairs. Input q_vec is a
        // local copy, so the caller's `q` is never mutated.
        pinocchio::forwardKinematics(ctx->model, ctx->data, q_vec);
        pinocchio::updateGeometryPlacements(
            ctx->model, ctx->data, ctx->geom_model, ctx->geom_data);
        pinocchio::computeCollisions(ctx->geom_model, ctx->geom_data, stop);

        // Translate colliding pairs into the flat output buffer, never writing
        // beyond `max_out` entries (Req 10.1, 10.2).
        int written = 0;
        const std::size_t num_pairs = ctx->geom_model.collisionPairs.size();
        for (std::size_t i = 0; i < num_pairs; ++i) {
            const auto& res = ctx->geom_data.collisionResults[i];
            if (!res.isCollision()) {
                continue;
            }

            if (written >= max_out) {
                // More collisions exist than the buffer can hold: report the
                // truncated count (== max_out) and signal a collision without
                // writing past the buffer bounds (Req 10.2).
                *num_out = written;
                return 1;
            }

            const auto& cp = ctx->geom_model.collisionPairs[i];
            CollisionPairOut& entry = out[written];
            entry.geom_a = static_cast<int>(cp.first);
            entry.geom_b = static_cast<int>(cp.second);
            // Borrowed pointers into geom_model.geometryObjects; valid until
            // collision_free(ctx). The Rust layer copies them before the next
            // FFI call (Req 10.8).
            entry.name_a = ctx->geom_model.geometryObjects[cp.first].name.c_str();
            entry.name_b = ctx->geom_model.geometryObjects[cp.second].name.c_str();
            // Signed separation: penetration_depth is <= 0 while penetrating.
            entry.distance = (res.numContacts() > 0)
                                 ? res.getContact(0).penetration_depth
                                 : 0.0;
            ++written;

            if (stop) {
                break;
            }
        }

        // Exact count written (Req 10.1); 1 on collision (Req 10.3),
        // 0 when clear (Req 10.4).
        *num_out = written;
        return (written > 0) ? 1 : 0;
    } catch (const std::exception& e) {
        // Catch everything at the ABI boundary; never unwind across it
        // (Req 10.6). Report failure with -1 and a message (Req 10.5).
        if (num_out) *num_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                std::string("collision_check exception: ") + e.what());
        }
        return -1;
    } catch (...) {
        if (num_out) *num_out = 0;
        if (error_msg) {
            *error_msg = dup_error("collision_check exception: unknown error");
        }
        return -1;
    }
}

int collision_min_distance(CollisionContext* ctx,
                           const double* q,
                           CollisionPairOut* out,
                           double* min_distance_out,
                           char** error_msg) {
    // Establish the distance output up front so every early return leaves it
    // well-defined for the caller.
    if (min_distance_out) {
        *min_distance_out = std::numeric_limits<double>::infinity();
    }

    // --- Argument validation -------------------------------------------
    if (!ctx || !q) {
        if (error_msg) {
            *error_msg = dup_error(
                "collision_min_distance: null context or configuration pointer");
        }
        return -1;
    }

    try {
        // --- DOF validation before indexing q (Req 9.5) ----------------
        // Reject a DOF mismatch *before* reading any element of `q`.
        const int nq = ctx->model.nq;
        if (nq != COLLISION_EXPECTED_NQ) {
            if (error_msg) {
                *error_msg = dup_error(
                    "collision_min_distance: DOF mismatch (model nq=" +
                    std::to_string(nq) + ", configuration provides " +
                    std::to_string(COLLISION_EXPECTED_NQ) + ")");
            }
            return -1;
        }

        // Now safe to read exactly nq (== 6) configuration values. q_vec is a
        // local copy, so the caller's `q` is never mutated.
        Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q, nq);

        // Forward kinematics -> geometry placements -> narrowphase distance
        // over the registered (post-SRDF) collision pairs.
        pinocchio::forwardKinematics(ctx->model, ctx->data, q_vec);
        pinocchio::updateGeometryPlacements(
            ctx->model, ctx->data, ctx->geom_model, ctx->geom_data);
        pinocchio::computeDistances(ctx->geom_model, ctx->geom_data);

        const std::size_t num_pairs = ctx->geom_model.collisionPairs.size();
        if (num_pairs == 0) {
            // No pairs to evaluate: no separation distance is available. Leave
            // *min_distance_out at +inf and do not write `out` (Req 5.5 is
            // enforced by the Rust scene layer). Success: nothing failed.
            return 0;
        }

        // Find the pair with the smallest signed separation. Coal's
        // DistanceResult::min_distance is negative while penetrating.
        double best = std::numeric_limits<double>::infinity();
        std::size_t best_idx = 0;
        for (std::size_t i = 0; i < num_pairs; ++i) {
            const double d = ctx->geom_data.distanceResults[i].min_distance;
            if (d < best) {
                best = d;
                best_idx = i;
            }
        }

        if (min_distance_out) {
            *min_distance_out = best;
        }

        // Write the closest pair when the caller requested it (Req: "Writes the
        // closest pair into *out when out != NULL").
        if (out) {
            const auto& cp = ctx->geom_model.collisionPairs[best_idx];
            out->geom_a = static_cast<int>(cp.first);
            out->geom_b = static_cast<int>(cp.second);
            // Borrowed pointers into geom_model.geometryObjects; valid until
            // collision_free(ctx). The Rust layer copies them before the next
            // FFI call (Req 10.8).
            out->name_a = ctx->geom_model.geometryObjects[cp.first].name.c_str();
            out->name_b = ctx->geom_model.geometryObjects[cp.second].name.c_str();
            out->distance = best;
        }

        return 0;
    } catch (const std::exception& e) {
        // Catch everything at the ABI boundary; never unwind across it
        // (Req 10.6). Report failure with -1 and a message (Req 10.5).
        if (min_distance_out) {
            *min_distance_out = std::numeric_limits<double>::infinity();
        }
        if (error_msg) {
            *error_msg = dup_error(
                std::string("collision_min_distance exception: ") + e.what());
        }
        return -1;
    } catch (...) {
        if (min_distance_out) {
            *min_distance_out = std::numeric_limits<double>::infinity();
        }
        if (error_msg) {
            *error_msg = dup_error("collision_min_distance exception: unknown error");
        }
        return -1;
    }
}

int collision_geometry_placements(CollisionContext* ctx,
                                  const double* q,
                                  double* placements_out,
                                  int* num_objects_out,
                                  char** error_msg) {
    // Establish the count output up front so every early return leaves it
    // well-defined.
    if (num_objects_out) *num_objects_out = 0;

    // --- Argument validation -------------------------------------------
    if (!ctx || !q || !placements_out) {
        if (error_msg) {
            *error_msg = dup_error(
                "collision_geometry_placements: null context, configuration, "
                "or output pointer");
        }
        return -1;
    }

    try {
        // --- DOF validation before indexing q (Req 9.5) ----------------
        // Reject a DOF mismatch *before* reading any element of `q`.
        const int nq = ctx->model.nq;
        if (nq != COLLISION_EXPECTED_NQ) {
            if (error_msg) {
                *error_msg = dup_error(
                    "collision_geometry_placements: DOF mismatch (model nq=" +
                    std::to_string(nq) + ", configuration provides " +
                    std::to_string(COLLISION_EXPECTED_NQ) + ")");
            }
            return -1;
        }

        // Now safe to read exactly nq (== 6) configuration values. q_vec is a
        // local copy, so the caller's `q` is never mutated.
        Eigen::VectorXd q_vec = Eigen::Map<const Eigen::VectorXd>(q, nq);

        // Forward kinematics -> geometry placements. Each geometry object's
        // base-frame placement is then available in geom_data.oMg[i].
        pinocchio::forwardKinematics(ctx->model, ctx->data, q_vec);
        pinocchio::updateGeometryPlacements(
            ctx->model, ctx->data, ctx->geom_model, ctx->geom_data);

        // Write exactly 7 * ngeoms values: [x, y, z, qx, qy, qz, qw] per object.
        // The Rust caller pre-sizes placements_out to 7 * MAX_GEOMETRY_OBJECTS
        // and clamps the reported count to that bound, so writing ngeoms
        // placements (on the order of ten for a 6-DOF URDF) stays in bounds.
        const std::size_t n = static_cast<std::size_t>(ctx->geom_model.ngeoms);
        for (std::size_t i = 0; i < n; ++i) {
            const pinocchio::SE3& oMg = ctx->geom_data.oMg[i];
            const Eigen::Vector3d t = oMg.translation();
            // Extract the rotation as a (normalized) quaternion. Eigen stores
            // coefficients internally as (x, y, z, w).
            Eigen::Quaterniond quat(oMg.rotation());
            quat.normalize();

            double* p = placements_out + i * 7;
            p[0] = t.x();
            p[1] = t.y();
            p[2] = t.z();
            p[3] = quat.x();
            p[4] = quat.y();
            p[5] = quat.z();
            p[6] = quat.w();
        }

        if (num_objects_out) *num_objects_out = static_cast<int>(n);
        return 0;
    } catch (const std::exception& e) {
        // Catch everything at the ABI boundary; never unwind across it
        // (Req 10.6). Report failure with -1 and a message (Req 10.5).
        if (num_objects_out) *num_objects_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                std::string("collision_geometry_placements exception: ") + e.what());
        }
        return -1;
    } catch (...) {
        if (num_objects_out) *num_objects_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                "collision_geometry_placements exception: unknown error");
        }
        return -1;
    }
}

int collision_geometry_half_extents(CollisionContext* ctx,
                                    double* half_extents_out,
                                    int* num_objects_out,
                                    char** error_msg) {
    // Establish the count output up front so every early return leaves it
    // well-defined.
    if (num_objects_out) *num_objects_out = 0;

    // --- Argument validation -------------------------------------------
    if (!ctx || !half_extents_out) {
        if (error_msg) {
            *error_msg = dup_error(
                "collision_geometry_half_extents: null context or output pointer");
        }
        return -1;
    }

    try {
        // Local-frame half-extents are a static property of the loaded
        // geometry (independent of joint configuration): no forward
        // kinematics or geometry-placement update is needed here.
        //
        // The Rust caller pre-sizes half_extents_out to 3 * MAX_GEOMETRY_OBJECTS
        // and clamps the reported count to that bound, so writing ngeoms
        // triples (on the order of ten for a 6-DOF URDF) stays in bounds.
        const std::size_t n = static_cast<std::size_t>(ctx->geom_model.ngeoms);
        constexpr double kMinHalfExtent = 1e-4; // metres; never report zero/degenerate.

        for (std::size_t i = 0; i < n; ++i) {
            double hx = kMinHalfExtent;
            double hy = kMinHalfExtent;
            double hz = kMinHalfExtent;

            const auto& geom_ptr = ctx->geom_model.geometryObjects[i].geometry;
            if (geom_ptr) {
                // computeLocalAABB() populates aabb_local (a local-frame AABB
                // with .min_ / .max_ corners) for every Coal/hpp-fcl
                // CollisionGeometry subclass. This is independent of the
                // object's world placement.
                geom_ptr->computeLocalAABB();
                const auto& aabb = geom_ptr->aabb_local;
                const double ex = (aabb.max_[0] - aabb.min_[0]) / 2.0;
                const double ey = (aabb.max_[1] - aabb.min_[1]) / 2.0;
                const double ez = (aabb.max_[2] - aabb.min_[2]) / 2.0;

                // Substitute a small positive epsilon for any degenerate
                // (<= 0) or non-finite axis rather than leaving it zero.
                hx = (std::isfinite(ex) && ex > 0.0) ? ex : kMinHalfExtent;
                hy = (std::isfinite(ey) && ey > 0.0) ? ey : kMinHalfExtent;
                hz = (std::isfinite(ez) && ez > 0.0) ? ez : kMinHalfExtent;
            }
            // else: no geometry attached to this object — fall back to the
            // epsilon half-extents already set above rather than skipping the
            // entry (registration order/count must match
            // collision_geometry_placements exactly).

            double* p = half_extents_out + i * 3;
            p[0] = hx;
            p[1] = hy;
            p[2] = hz;
        }

        if (num_objects_out) *num_objects_out = static_cast<int>(n);
        return 0;
    } catch (const std::exception& e) {
        // Catch everything at the ABI boundary; never unwind across it
        // (Req 10.6). Report failure with -1 and a message (Req 10.5).
        if (num_objects_out) *num_objects_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                std::string("collision_geometry_half_extents exception: ") + e.what());
        }
        return -1;
    } catch (...) {
        if (num_objects_out) *num_objects_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                "collision_geometry_half_extents exception: unknown error");
        }
        return -1;
    }
}

int collision_geometry_local_centers(CollisionContext* ctx,
                                     double* centers_out,
                                     int* num_objects_out,
                                     char** error_msg) {
    if (num_objects_out) *num_objects_out = 0;
    if (!ctx || !centers_out) {
        if (error_msg) {
            *error_msg = dup_error(
                "collision_geometry_local_centers: null context or output pointer");
        }
        return -1;
    }
    try {
        // Same registration order and count as collision_geometry_half_extents
        // (the Rust caller pre-sizes centers_out to 3 * MAX_GEOMETRY_OBJECTS).
        const std::size_t n = static_cast<std::size_t>(ctx->geom_model.ngeoms);
        for (std::size_t i = 0; i < n; ++i) {
            double c[3] = {0.0, 0.0, 0.0};
            const auto& geom_ptr = ctx->geom_model.geometryObjects[i].geometry;
            if (geom_ptr) {
                geom_ptr->computeLocalAABB();
                const auto& aabb = geom_ptr->aabb_local;
                for (int k = 0; k < 3; ++k) {
                    const double v = (aabb.max_[k] + aabb.min_[k]) / 2.0;
                    c[k] = std::isfinite(v) ? v : 0.0;
                }
            }
            double* p = centers_out + i * 3;
            p[0] = c[0];
            p[1] = c[1];
            p[2] = c[2];
        }
        if (num_objects_out) *num_objects_out = static_cast<int>(n);
        return 0;
    } catch (const std::exception& e) {
        if (num_objects_out) *num_objects_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                std::string("collision_geometry_local_centers exception: ") + e.what());
        }
        return -1;
    } catch (...) {
        if (num_objects_out) *num_objects_out = 0;
        if (error_msg) {
            *error_msg = dup_error(
                "collision_geometry_local_centers exception: unknown error");
        }
        return -1;
    }
}

} // extern "C"

#endif // PINOCCHIO_WRAPPER_COLLISION
