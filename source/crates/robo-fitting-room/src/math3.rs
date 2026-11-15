//! Small 4x4 homogeneous matrix utilities used by the FK pass in compose, and the 3-vector and
//! 3×3 helpers (with a symmetric eigensolver) of the mass properties and drives.
//! Conventions: row-major matrices, ZYX Euler for RPY (matching URDF semantics).

pub type Mat4 = [[f64; 4]; 4];

pub const IDENTITY: Mat4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

pub fn from_translation_rpy(translation: [f64; 3], rpy: [f64; 3]) -> Mat4 {
    let [roll, pitch, yaw] = rpy;
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            translation[0],
        ],
        [
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            translation[1],
        ],
        [-sp, cp * sr, cp * cr, translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub fn multiply(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut out = [[0.0f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] =
                a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];
        }
    }
    out
}

pub fn translation_of(m: &Mat4) -> [f64; 3] {
    [m[0][3], m[1][3], m[2][3]]
}

/// Applies the homogeneous transform `m` to the point `p`: `m · (p, 1)`.
pub fn transform_point(m: &Mat4, p: [f64; 3]) -> [f64; 3] {
    let row = |i: usize| m[i][0] * p[0] + m[i][1] * p[1] + m[i][2] * p[2] + m[i][3];
    [row(0), row(1), row(2)]
}

/// Rodrigues rotation about a unit axis by `angle` radians, embedded in a Mat4.
/// Used to bake home-pose joint angles into link bind transforms.
pub fn rotation_about_axis(axis: [f64; 3], angle: f64) -> Mat4 {
    let len_sq = axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2];
    if len_sq < 1e-12 {
        return IDENTITY;
    }
    let inv_len = 1.0 / len_sq.sqrt();
    let (x, y, z) = (axis[0] * inv_len, axis[1] * inv_len, axis[2] * inv_len);
    let (s, c) = angle.sin_cos();
    let one_c = 1.0 - c;
    [
        [
            c + x * x * one_c,
            x * y * one_c - z * s,
            x * z * one_c + y * s,
            0.0,
        ],
        [
            y * x * one_c + z * s,
            c + y * y * one_c,
            y * z * one_c - x * s,
            0.0,
        ],
        [
            z * x * one_c - y * s,
            z * y * one_c + x * s,
            c + z * z * one_c,
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// The inverse of the rigid transform `m` (a rotation R and a translation t): `(Rᵀ, −Rᵀ·t)`.
pub fn rigid_inverse(m: &Mat4) -> Mat4 {
    let r = transpose(&rotation_of(m));
    let t = apply3(&r, translation_of(m));
    [
        [r[0][0], r[0][1], r[0][2], -t[0]],
        [r[1][0], r[1][1], r[1][2], -t[1]],
        [r[2][0], r[2][1], r[2][2], -t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// The shortest rotation taking +X onto the direction `to` (normalised; a zero `to` gives the
/// identity): about `x̂ × to` by the angle between them, and for `to` along −X a half turn about
/// +Z (which also maps +Y to −Y and keeps +Z).
pub fn rotation_taking_x_to(to: [f64; 3]) -> Mat4 {
    let len = norm(to);
    if !(len.is_finite() && len > 1e-12) {
        return IDENTITY;
    }
    let to = scale(to, 1.0 / len);
    let x = [1.0, 0.0, 0.0];
    let k = cross(x, to);
    let sin = norm(k);
    let cos = dot(x, to);
    if sin < 1e-12 {
        return if cos > 0.0 {
            IDENTITY
        } else {
            rotation_about_axis([0.0, 0.0, 1.0], std::f64::consts::PI)
        };
    }
    rotation_about_axis(k, sin.atan2(cos))
}

/// Convert the 3x3 rotation sub-matrix of `m` to a unit quaternion `[x, y, z, w]`,
/// canonicalised so `w >= 0`.
pub fn quat_of(m: &Mat4) -> [f64; 4] {
    let trace = m[0][0] + m[1][1] + m[2][2];
    let (qx, qy, qz, qw) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            (m[2][1] - m[1][2]) / s,
            (m[0][2] - m[2][0]) / s,
            (m[1][0] - m[0][1]) / s,
            0.25 * s,
        )
    } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
        let s = (1.0 + m[0][0] - m[1][1] - m[2][2]).sqrt() * 2.0;
        (
            0.25 * s,
            (m[0][1] + m[1][0]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[2][1] - m[1][2]) / s,
        )
    } else if m[1][1] > m[2][2] {
        let s = (1.0 + m[1][1] - m[0][0] - m[2][2]).sqrt() * 2.0;
        (
            (m[0][1] + m[1][0]) / s,
            0.25 * s,
            (m[1][2] + m[2][1]) / s,
            (m[0][2] - m[2][0]) / s,
        )
    } else {
        let s = (1.0 + m[2][2] - m[0][0] - m[1][1]).sqrt() * 2.0;
        (
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
            (m[1][0] - m[0][1]) / s,
        )
    };
    let mag = (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
    let (qx, qy, qz, qw) = if mag < 1e-12 {
        (0.0, 0.0, 0.0, 1.0)
    } else {
        (qx / mag, qy / mag, qz / mag, qw / mag)
    };
    // Canonicalise: PhysX prefers w >= 0
    if qw < 0.0 {
        [-qx, -qy, -qz, -qw]
    } else {
        [qx, qy, qz, qw]
    }
}

// --- 3-vectors and 3×3 matrices: mass properties and drive tuning ---

pub type Vec3 = [f64; 3];
/// A 3×3 matrix, row-major like [`Mat4`].
pub type Mat3 = [[f64; 3]; 3];

pub const IDENTITY3: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

/// The rotation block of `m`.
pub fn rotation_of(m: &Mat4) -> Mat3 {
    [
        [m[0][0], m[0][1], m[0][2]],
        [m[1][0], m[1][1], m[1][2]],
        [m[2][0], m[2][1], m[2][2]],
    ]
}

/// The direction `v` rotated by `m`; unlike a point ([`transform_point`]) it isn't translated.
pub fn rotate(m: &Mat4, v: Vec3) -> Vec3 {
    apply3(&rotation_of(m), v)
}

/// `m · v`.
pub fn apply3(m: &Mat3, v: Vec3) -> Vec3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

pub fn mat3_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

pub fn transpose(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// `r · s · rᵀ`: the tensor `s`, given in the axes `r` maps from, in the axes `r` maps to (a body's
/// inertia in its parent's axes).
pub fn conjugate(r: &Mat3, s: &Mat3) -> Mat3 {
    mat3_mul(&mat3_mul(r, s), &transpose(r))
}

/// The diagonal matrix `diag(d)`.
pub fn diag(d: Vec3) -> Mat3 {
    [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]]
}

/// The outer product `a · bᵀ`.
pub fn outer(a: Vec3, b: Vec3) -> Mat3 {
    [scale(b, a[0]), scale(b, a[1]), scale(b, a[2])]
}

pub fn mat3_add(a: &Mat3, b: &Mat3) -> Mat3 {
    [add(a[0], b[0]), add(a[1], b[1]), add(a[2], b[2])]
}

pub fn mat3_scale(a: &Mat3, s: f64) -> Mat3 {
    [scale(a[0], s), scale(a[1], s), scale(a[2], s)]
}

/// The determinant of `m`.
pub fn det3(m: &Mat3) -> f64 {
    dot(m[0], cross(m[1], m[2]))
}

/// The unit quaternion `[x, y, z, w]` of the rotation `r`, canonicalised like [`quat_of`] (`w >= 0`).
pub fn quat_of_rotation(r: &Mat3) -> [f64; 4] {
    quat_of(&[
        [r[0][0], r[0][1], r[0][2], 0.0],
        [r[1][0], r[1][1], r[1][2], 0.0],
        [r[2][0], r[2][1], r[2][2], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

/// Eigen-decomposition of the symmetric 3×3 matrix `a` (its upper triangle is read) by cyclic Jacobi
/// rotations: the eigenvalues `d` and a rotation `v` (det +1) whose columns are their unit
/// eigenvectors, so `a = v · diag(d) · vᵀ`. Of the equivalent decompositions (the eigenpairs in any
/// order, each eigenvector of either sign) it returns the one whose `v` is closest to the identity
/// (the largest trace), so a diagonal `a` comes back as its own diagonal with `v = I`.
pub fn eig3(a: &Mat3) -> (Vec3, Mat3) {
    let mut m = [
        [a[0][0], a[0][1], a[0][2]],
        [a[0][1], a[1][1], a[1][2]],
        [a[0][2], a[1][2], a[2][2]],
    ];
    let mut v = IDENTITY3;
    // Jacobi converges quadratically; a 3×3 needs a handful of sweeps.
    for _ in 0..64 {
        let off = m[0][1] * m[0][1] + m[0][2] * m[0][2] + m[1][2] * m[1][2];
        let on = m[0][0] * m[0][0] + m[1][1] * m[1][1] + m[2][2] * m[2][2];
        if off == 0.0 || off <= 1e-32 * on {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            let apq = m[p][q];
            if apq == 0.0 {
                continue;
            }
            // The plane rotation that zeroes m[p][q] (Numerical Recipes §11.1): t = tan φ, the
            // smaller root of t² + 2θt − 1 = 0.
            let theta = (m[q][q] - m[p][p]) / (2.0 * apq);
            let t = if theta.abs() > 1e150 {
                0.5 / theta
            } else {
                let t = 1.0 / (theta.abs() + (theta * theta + 1.0).sqrt());
                if theta < 0.0 {
                    -t
                } else {
                    t
                }
            };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            let mut j = IDENTITY3;
            j[p][p] = c;
            j[q][q] = c;
            j[p][q] = s;
            j[q][p] = -s;
            m = mat3_mul(&mat3_mul(&transpose(&j), &m), &j);
            m[p][q] = 0.0;
            m[q][p] = 0.0;
            v = mat3_mul(&v, &j);
        }
    }
    canonical_eigenbasis([m[0][0], m[1][1], m[2][2]], &v)
}

/// Of the eigenbases equivalent to the eigenvalues `d` and eigenvectors `v` (columns) — the
/// eigenpairs in any order, each eigenvector of either sign — the proper rotation closest to the
/// identity (the largest trace; the first in a fixed order on a tie).
fn canonical_eigenbasis(d: Vec3, v: &Mat3) -> (Vec3, Mat3) {
    const ORDERS: [[usize; 3]; 6] = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    let mut best: Option<(f64, Vec3, Mat3)> = None;
    for order in ORDERS {
        // Column k is eigenvector order[k], signed so the diagonal entry r[k][k] is >= 0.
        let mut r = [[0.0f64; 3]; 3];
        for (k, &e) in order.iter().enumerate() {
            let sign = if v[k][e] < 0.0 { -1.0 } else { 1.0 };
            for (row, src) in r.iter_mut().zip(v) {
                row[k] = sign * src[e];
            }
        }
        if det3(&r) < 0.0 {
            // A reflection: flip the column that costs the least trace.
            let k = (0..3)
                .min_by(|&a, &b| r[a][a].total_cmp(&r[b][b]))
                .unwrap_or(2);
            for row in r.iter_mut() {
                row[k] = -row[k];
            }
        }
        let trace = r[0][0] + r[1][1] + r[2][2];
        if best.as_ref().is_none_or(|(t, _, _)| trace > *t) {
            best = Some((trace, order.map(|e| d[e]), r));
        }
    }
    let (_, d, r) = best.expect("six candidate orders");
    (d, r)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rigid_inverse_undoes_a_rigid_transform() {
        let m = from_translation_rpy([1.0, -2.0, 0.5], [0.3, -0.7, 1.2]);
        let back = multiply(&rigid_inverse(&m), &m);
        for (i, row) in back.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((v - want).abs() < 1e-12, "{back:?}");
            }
        }
    }

    #[test]
    fn rotation_taking_x_to_a_direction_maps_x_onto_it() {
        let s = 0.5f64.sqrt();
        for to in [
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, -3.0, 0.0],
            [s, 0.0, s],
            [0.2, -0.4, 0.9],
            [-1.0, 1e-13, 0.0],
        ] {
            let r = rotation_taking_x_to(to);
            let unit = scale(to, 1.0 / norm(to));
            assert_close(rotate(&r, [1.0, 0.0, 0.0]), unit);
            // A proper rotation.
            assert!((det3(&rotation_of(&r)) - 1.0).abs() < 1e-12);
            let rt = transpose(&rotation_of(&r));
            let i = mat3_mul(&rt, &rotation_of(&r));
            for (a, row) in i.iter().enumerate() {
                assert_close(*row, IDENTITY3[a]);
            }
        }
        // +X itself and a zero direction: the identity; −X: a half turn about +Z.
        assert_eq!(rotation_taking_x_to([2.0, 0.0, 0.0]), IDENTITY);
        assert_eq!(rotation_taking_x_to([0.0; 3]), IDENTITY);
        let half = rotation_taking_x_to([-1.0, 0.0, 0.0]);
        assert_close(rotate(&half, [0.0, 0.0, 1.0]), [0.0, 0.0, 1.0]);
        assert_close(rotate(&half, [0.0, 1.0, 0.0]), [0.0, -1.0, 0.0]);
    }

    #[test]
    fn identity_round_trip() {
        let q = quat_of(&IDENTITY);
        assert!((q[0]).abs() < 1e-9);
        assert!((q[1]).abs() < 1e-9);
        assert!((q[2]).abs() < 1e-9);
        assert!((q[3] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn translation_only() {
        let m = from_translation_rpy([1.0, 2.0, 3.0], [0.0, 0.0, 0.0]);
        assert_eq!(translation_of(&m), [1.0, 2.0, 3.0]);
        let q = quat_of(&m);
        assert!((q[3] - 1.0).abs() < 1e-9);
    }

    fn assert_close(got: [f64; 3], want: [f64; 3]) {
        for (g, w) in got.iter().zip(want) {
            assert!((g - w).abs() < 1e-12, "{:?} != {:?}", got, want);
        }
    }

    #[test]
    fn transform_point_rotates_then_translates() {
        // Yaw 90° maps +X to +Y; the translation is added afterwards.
        let m = from_translation_rpy([1.0, 2.0, 3.0], [0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        assert_close(transform_point(&m, [1.0, 0.0, 0.5]), [1.0, 3.0, 3.5]);
        // Composes like the matrices: (a · b) p = a (b p).
        let a = from_translation_rpy([0.5, -1.0, 2.0], [0.3, -0.2, 1.1]);
        let b = from_translation_rpy([-2.0, 0.25, 1.0], [-0.7, 0.4, 0.05]);
        let q = [0.3, -4.0, 2.5];
        assert_close(
            transform_point(&multiply(&a, &b), q),
            transform_point(&a, transform_point(&b, q)),
        );
        assert_eq!(transform_point(&IDENTITY, q), q);
    }

    #[test]
    fn yaw_90_deg() {
        let m = from_translation_rpy([0.0; 3], [0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        let q = quat_of(&m);
        // 90° around Z: w = cos(45°) = sqrt(2)/2, z = sin(45°) = sqrt(2)/2
        assert!((q[2] - 0.7071067811865475).abs() < 1e-9);
        assert!((q[3] - 0.7071067811865475).abs() < 1e-9);
    }

    #[test]
    fn vector_helpers() {
        let (a, b) = ([1.0, 2.0, 3.0], [-4.0, 0.5, 2.0]);
        assert_eq!(dot(a, b), -4.0 + 1.0 + 6.0);
        assert_eq!(cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]);
        // a × b is perpendicular to both.
        assert!(dot(cross(a, b), a).abs() < 1e-12 && dot(cross(a, b), b).abs() < 1e-12);
        assert_eq!(add(a, b), [-3.0, 2.5, 5.0]);
        assert_eq!(sub(a, b), [5.0, 1.5, 1.0]);
        assert_eq!(scale(a, 2.0), [2.0, 4.0, 6.0]);
        assert_eq!(norm([3.0, 4.0, 12.0]), 13.0);
        // `rotate` turns a direction but doesn't translate it.
        let m = from_translation_rpy([5.0, 6.0, 7.0], [0.0, 0.0, std::f64::consts::FRAC_PI_2]);
        assert_close(rotate(&m, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_close(transform_point(&m, [1.0, 0.0, 0.0]), [5.0, 7.0, 7.0]);
        assert_eq!(det3(&IDENTITY3), 1.0);
        assert_eq!(det3(&diag([2.0, 3.0, -1.0])), -6.0);
    }

    /// `r · diag(d) · rᵀ`.
    fn compose_tensor(d: Vec3, r: &Mat3) -> Mat3 {
        conjugate(r, &diag(d))
    }

    fn assert_mat3_close(got: &Mat3, want: &Mat3, tol: f64) {
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (got[i][j] - want[i][j]).abs() < tol,
                    "{got:?} != {want:?} at [{i}][{j}]"
                );
            }
        }
    }

    /// `(d, v)` decomposes `a`, with `v` a proper rotation.
    fn assert_decomposes(a: &Mat3, d: Vec3, v: &Mat3) {
        assert_mat3_close(&compose_tensor(d, v), a, 1e-12);
        assert_mat3_close(&mat3_mul(&transpose(v), v), &IDENTITY3, 1e-12);
        assert!((det3(v) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn eig3_keeps_a_diagonal_matrix_as_is() {
        let a = diag([0.5, 0.2, 0.9]);
        let (d, v) = eig3(&a);
        assert_eq!(d, [0.5, 0.2, 0.9]);
        assert_eq!(v, IDENTITY3);
        let (d, v) = eig3(&[[0.0; 3]; 3]);
        assert_eq!((d, v), ([0.0; 3], IDENTITY3));
    }

    #[test]
    fn eig3_recovers_a_rotated_diagonal() {
        // A small rotation: the decomposition closest to the identity is that rotation itself.
        let r = rotation_of(&from_translation_rpy([0.0; 3], [0.2, -0.1, 0.3]));
        let a = compose_tensor([1.0, 2.0, 3.0], &r);
        let (d, v) = eig3(&a);
        assert_decomposes(&a, d, &v);
        for i in 0..3 {
            assert!((d[i] - [1.0, 2.0, 3.0][i]).abs() < 1e-12, "{d:?}");
        }
        assert_mat3_close(&v, &r, 1e-9);
        // A half turn about Z: the same tensor, so the identity comes back.
        let flip = rotation_of(&from_translation_rpy([0.0; 3], [0.0, 0.0, std::f64::consts::PI]));
        let (d, v) = eig3(&compose_tensor([1.0, 2.0, 3.0], &flip));
        assert_mat3_close(&v, &IDENTITY3, 1e-12);
        assert!((d[0] - 1.0).abs() < 1e-12 && (d[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn eig3_decomposes_general_and_degenerate_tensors() {
        let general = [[4.0, -1.5, 0.3], [-1.5, 2.0, 0.7], [0.3, 0.7, 1.0]];
        let (d, v) = eig3(&general);
        assert_decomposes(&general, d, &v);
        // Two equal moments (a body symmetric about one axis), tilted.
        let r = rotation_of(&from_translation_rpy([0.0; 3], [1.1, 0.4, -2.0]));
        let degenerate = compose_tensor([2.0, 2.0, 0.5], &r);
        let (d, v) = eig3(&degenerate);
        assert_decomposes(&degenerate, d, &v);
        // Only the upper triangle is read.
        let mut lower_garbage = general;
        lower_garbage[1][0] = 99.0;
        assert_eq!(eig3(&lower_garbage), eig3(&general));
        // Deterministic.
        assert_eq!(eig3(&general), eig3(&general));
    }

    #[test]
    fn quat_of_rotation_matches_quat_of() {
        let m = from_translation_rpy([1.0, 2.0, 3.0], [0.3, -0.4, 2.0]);
        assert_eq!(quat_of_rotation(&rotation_of(&m)), quat_of(&m));
    }

    #[test]
    fn quat_w_negative_is_canonicalised() {
        // Build a rotation that yields w<0 then verify quat_of flips it.
        // 270° around Z: cos(135°) ≈ -0.707, sin(135°) ≈ 0.707
        let angle = 3.0 * std::f64::consts::FRAC_PI_2;
        let m = from_translation_rpy([0.0; 3], [0.0, 0.0, angle]);
        let q = quat_of(&m);
        assert!(q[3] >= 0.0, "quat_of must canonicalise w >= 0; got w = {}", q[3]);
    }
}
