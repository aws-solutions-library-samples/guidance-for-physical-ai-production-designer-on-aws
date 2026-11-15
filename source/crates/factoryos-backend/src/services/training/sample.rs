//! Cells drawn from a training job, as scenegen draws them (`scenegen/sample.py`), geometry only:
//! no IK, so the planner's feasibility filter still rejects some (unreachable welds, colliding
//! bases). Deterministic per (job, seed).

use std::f64::consts::TAU;

use factoryos_protocol::training::{
    Placement, SampledCell, SampledProduct, SampledRobot, SampledWeld, TrainingCell, TrainingJob, WeldFaces, YawMode,
};

/// Tries per product / robot / weld before it is given up on (and noted).
const PRODUCT_TRIES: usize = 200;
const ROBOT_TRIES: usize = 400;
const WELD_TRIES: usize = 50;

/// splitmix64: small, fast and the same everywhere.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed ^ 0x6a09_e667_f3bc_c908)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// U[0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn uniform(&mut self, [lo, hi]: [f64; 2]) -> f64 {
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        lo + (hi - lo) * self.unit()
    }

    /// U{lo..=hi}.
    pub fn int(&mut self, [lo, hi]: [u32; 2]) -> u32 {
        let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
        lo + (self.next_u64() % u64::from(hi - lo + 1)) as u32
    }

    fn index(&mut self, n: usize) -> usize {
        (self.next_u64() % n.max(1) as u64) as usize
    }
}

/// A product's footprint: centre, half sizes and yaw (radians).
#[derive(Clone, Copy, Debug)]
struct Rect {
    c: [f64; 2],
    h: [f64; 2],
    yaw: f64,
}

impl Rect {
    fn of(p: &SampledProduct) -> Self {
        Self { c: [p.center[0], p.center[1]], h: [p.half[0], p.half[1]], yaw: p.yaw_deg.to_radians() }
    }

    fn axes(&self) -> [[f64; 2]; 2] {
        let (s, c) = self.yaw.sin_cos();
        [[c, s], [-s, c]]
    }

    fn corners(&self) -> [[f64; 2]; 4] {
        let [u, v] = self.axes();
        let at = |a: f64, b: f64| [self.c[0] + u[0] * a + v[0] * b, self.c[1] + u[1] * a + v[1] * b];
        let [hx, hy] = self.h;
        [at(hx, hy), at(-hx, hy), at(-hx, -hy), at(hx, -hy)]
    }

    /// Distance of `p` from the rectangle (0 inside).
    fn distance(&self, p: [f64; 2]) -> f64 {
        let [u, v] = self.axes();
        let d = [p[0] - self.c[0], p[1] - self.c[1]];
        let lx = (d[0] * u[0] + d[1] * u[1]).abs() - self.h[0];
        let ly = (d[0] * v[0] + d[1] * v[1]).abs() - self.h[1];
        lx.max(0.0).hypot(ly.max(0.0))
    }

    /// How far the outline is from the centre along the unit direction `dir`.
    fn support(&self, dir: [f64; 2]) -> f64 {
        let [u, v] = self.axes();
        let (a, b) = ((dir[0] * u[0] + dir[1] * u[1]).abs(), (dir[0] * v[0] + dir[1] * v[1]).abs());
        // The ray leaves through the side it hits first.
        let tx = if a > 1e-9 { self.h[0] / a } else { f64::INFINITY };
        let ty = if b > 1e-9 { self.h[1] / b } else { f64::INFINITY };
        tx.min(ty)
    }

    /// Distance between two rectangles (0 if they overlap).
    fn gap(&self, other: &Rect) -> f64 {
        if self.overlaps(other) {
            return 0.0;
        }
        let a = self.corners().iter().map(|p| other.distance(*p)).fold(f64::INFINITY, f64::min);
        let b = other.corners().iter().map(|p| self.distance(*p)).fold(f64::INFINITY, f64::min);
        a.min(b)
    }

    /// Separating-axis test.
    fn overlaps(&self, other: &Rect) -> bool {
        let (ca, cb) = (self.corners(), other.corners());
        for axis in self.axes().into_iter().chain(other.axes()) {
            let project = |cs: &[[f64; 2]; 4]| {
                cs.iter().map(|p| p[0] * axis[0] + p[1] * axis[1]).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| (lo.min(x), hi.max(x)))
            };
            let ((alo, ahi), (blo, bhi)) = (project(&ca), project(&cb));
            if ahi < blo || bhi < alo {
                return false;
            }
        }
        true
    }
}

/// The loaded cell itself, with its own welds (as [`TrainingCell`] has them).
pub fn loaded_cell(cell: &TrainingCell, welds: &[SampledWeld]) -> SampledCell {
    SampledCell {
        seed: None,
        robots: cell
            .robots
            .iter()
            .map(|r| SampledRobot { model: r.model.clone(), position: r.position, yaw_deg: r.yaw_deg })
            .collect(),
        products: cell.products.clone(),
        welds: welds.to_vec(),
        notes: Vec::new(),
    }
}

/// Draw one cell from `job`; `cell`: the loaded cell (its bases and products, for the options
/// that start from them).
pub fn sample(job: &TrainingJob, seed: u64, cell: &TrainingCell) -> SampledCell {
    let mut rng = Rng::new(seed);
    let mut notes = Vec::new();
    let n_robots = draw_count(job, &mut rng);

    let products = if job.products.from_cell {
        if cell.products.is_empty() {
            notes.push("the loaded cell has no product boxes: products drawn instead".to_owned());
            draw_products(job, &mut rng, &mut notes)
        } else {
            cell.products.clone()
        }
    } else {
        draw_products(job, &mut rng, &mut notes)
    };
    let robots = place_robots(job, n_robots, &products, cell, &mut rng, &mut notes);
    let welds = draw_welds(job, robots.len(), &products, &mut rng, &mut notes);
    SampledCell { seed: Some(seed), robots, products, welds, notes }
}

fn draw_count(job: &TrainingJob, rng: &mut Rng) -> usize {
    let weights: Vec<(u32, f64)> = job.robots.count_weights.iter().filter(|(n, w)| **n > 0 && **w > 0.0).map(|(n, w)| (*n, *w)).collect();
    let total: f64 = weights.iter().map(|(_, w)| w).sum();
    if weights.is_empty() || total <= 0.0 {
        return 1;
    }
    let mut x = rng.unit() * total;
    for (n, w) in &weights {
        if x < *w {
            return *n as usize;
        }
        x -= w;
    }
    weights.last().map_or(1, |(n, _)| *n as usize)
}

fn draw_products(job: &TrainingJob, rng: &mut Rng, notes: &mut Vec<String>) -> Vec<SampledProduct> {
    let spec = &job.products;
    let n = rng.int(spec.count).max(1) as usize;
    let radius = spec.layout_radius_m[0] + spec.layout_radius_m[1] * n as f64;
    let mut products: Vec<SampledProduct> = Vec::new();
    for i in 0..n {
        let placed = (0..PRODUCT_TRIES).find_map(|_| {
            let sx = rng.uniform(spec.size_m);
            let sy = if spec.square { sx } else { rng.uniform(spec.size_m) };
            let h = rng.uniform(spec.height_m);
            let yaw = rng.uniform(spec.yaw_deg);
            let (r, a) = (radius * rng.unit().sqrt(), rng.unit() * TAU);
            let candidate = SampledProduct {
                name: format!("product {}", i + 1),
                center: [r * a.cos(), r * a.sin(), h / 2.0],
                half: [sx / 2.0, sy / 2.0, h / 2.0],
                yaw_deg: yaw,
            };
            let rect = Rect::of(&candidate);
            products.iter().all(|p| Rect::of(p).gap(&rect) >= spec.min_gap_m).then_some(candidate)
        });
        match placed {
            Some(p) => products.push(p),
            None => notes.push(format!("product {}: no room in the layout disk (r = {radius:.2} m)", i + 1)),
        }
    }
    products
}

fn place_robots(
    job: &TrainingJob,
    n: usize,
    products: &[SampledProduct],
    cell: &TrainingCell,
    rng: &mut Rng,
    notes: &mut Vec<String>,
) -> Vec<SampledRobot> {
    let spec = &job.robots;
    let rects: Vec<Rect> = products.iter().map(Rect::of).collect();
    let spacing = spec.min_spacing_m.max(2.0 * spec.footprint_r_m);
    let clearance = spec.ring_m[0].min(spec.ring_m[1]).max(spec.footprint_r_m);
    let models: Vec<String> = if spec.models.is_empty() {
        cell.robots.iter().map(|r| r.model.clone()).collect()
    } else {
        spec.models.clone()
    };
    let centre = centroid(products);
    let mut placement = spec.placement;
    if placement == Placement::CellJitter && cell.robots.is_empty() {
        notes.push("the loaded cell has no robots: random ring placement instead".to_owned());
        placement = Placement::RandomRing;
    }
    let circle = (rng.uniform(spec.ring_m) + outline_radius(&rects, centre), rng.unit() * TAU);

    let mut robots: Vec<SampledRobot> = Vec::new();
    for i in 0..n {
        let clear = |p: [f64; 2], robots: &[SampledRobot], strict: bool| {
            let off_products = rects.iter().all(|r| r.distance(p) >= if strict { clearance } else { spec.footprint_r_m });
            off_products && robots.iter().all(|r| (r.position[0] - p[0]).hypot(r.position[1] - p[1]) >= spacing)
        };
        let found = (0..ROBOT_TRIES).find_map(|_| {
            let (p, toward, cell_yaw) = match placement {
                Placement::RandomRing => {
                    if rects.is_empty() {
                        let a = rng.unit() * TAU;
                        let d = rng.uniform(spec.ring_m);
                        return Some(([d * a.cos(), d * a.sin()], [0.0, 0.0], None)).filter(|(p, ..)| clear(*p, &robots, false));
                    }
                    let anchor = rects[rng.index(rects.len())];
                    let a = rng.unit() * TAU;
                    let dir = [a.cos(), a.sin()];
                    let d = anchor.support(dir) + rng.uniform(spec.ring_m);
                    ([anchor.c[0] + dir[0] * d, anchor.c[1] + dir[1] * d], anchor.c, None)
                }
                Placement::Circle => {
                    let a = circle.1 + TAU * i as f64 / n as f64;
                    let base = [centre[0] + circle.0 * a.cos(), centre[1] + circle.0 * a.sin()];
                    (jitter(base, spec.jitter_m, rng), centre, None)
                }
                Placement::CellJitter => {
                    let r = &cell.robots[i % cell.robots.len()];
                    (jitter([r.position[0], r.position[1]], spec.jitter_m, rng), centre, Some(r.yaw_deg))
                }
            };
            // The ring keeps scenegen's clearance; the others only keep the base off the products.
            clear(p, &robots, placement == Placement::RandomRing).then_some((p, toward, cell_yaw))
        });
        let Some((p, toward, cell_yaw)) = found else {
            notes.push(format!("robot {}: no room for its base ({ROBOT_TRIES} tries)", i + 1));
            continue;
        };
        let jitter_deg = rng.uniform([-spec.yaw_jitter_deg, spec.yaw_jitter_deg]);
        let yaw_deg = match (cell_yaw, spec.yaw) {
            (Some(yaw), _) => yaw + jitter_deg,
            (None, YawMode::Uniform) => rng.uniform([0.0, 360.0]),
            (None, YawMode::Facing) => (toward[1] - p[1]).atan2(toward[0] - p[0]).to_degrees() + jitter_deg,
        };
        let model = models.get(robots.len() % models.len().max(1)).cloned().unwrap_or_default();
        robots.push(SampledRobot { model, position: [p[0], p[1], rng.uniform(spec.base_z_m)], yaw_deg: yaw_deg.rem_euclid(360.0) });
    }
    robots
}

/// Where the job puts the bases, as closed floor outlines for the 3D view: the ring's inner and
/// outer edge around each product, the circle's band, or the jitter disks around the cell's bases.
pub fn placement_guides(job: &TrainingJob, products: &[SampledProduct], cell: &TrainingCell) -> Vec<Vec<[f64; 2]>> {
    const STEPS: usize = 72;
    let spec = &job.robots;
    let rects: Vec<Rect> = products.iter().map(Rect::of).collect();
    let circle = |c: [f64; 2], r: f64| (0..=STEPS).map(|k| {
        let a = TAU * k as f64 / STEPS as f64;
        [c[0] + r * a.cos(), c[1] + r * a.sin()]
    }).collect::<Vec<_>>();
    match spec.placement {
        Placement::RandomRing => rects
            .iter()
            .flat_map(|rect| spec.ring_m.map(|d| (0..=STEPS).map(|k| {
                let a = TAU * k as f64 / STEPS as f64;
                let dir = [a.cos(), a.sin()];
                let r = rect.support(dir) + d;
                [rect.c[0] + dir[0] * r, rect.c[1] + dir[1] * r]
            }).collect()))
            .collect(),
        Placement::Circle => {
            let centre = centroid(products);
            let outline = outline_radius(&rects, centre);
            spec.ring_m.iter().map(|d| circle(centre, outline + d)).collect()
        }
        Placement::CellJitter => cell.robots.iter().map(|r| circle([r.position[0], r.position[1]], spec.jitter_m.max(0.02))).collect(),
    }
}

fn jitter(p: [f64; 2], radius: f64, rng: &mut Rng) -> [f64; 2] {
    let (r, a) = (radius.max(0.0) * rng.unit().sqrt(), rng.unit() * TAU);
    [p[0] + r * a.cos(), p[1] + r * a.sin()]
}

fn centroid(products: &[SampledProduct]) -> [f64; 2] {
    if products.is_empty() {
        return [0.0, 0.0];
    }
    let n = products.len() as f64;
    [products.iter().map(|p| p.center[0]).sum::<f64>() / n, products.iter().map(|p| p.center[1]).sum::<f64>() / n]
}

/// The farthest product corner from `centre`.
fn outline_radius(rects: &[Rect], centre: [f64; 2]) -> f64 {
    rects
        .iter()
        .flat_map(|r| r.corners())
        .map(|p| (p[0] - centre[0]).hypot(p[1] - centre[1]))
        .fold(0.0, f64::max)
}

/// A face of a product box: outward normal and the two in-face axes (local frame), half sizes
/// along them, and its area.
struct Face {
    n: [f64; 3],
    t: [[f64; 3]; 2],
    half: [f64; 2],
}

fn faces(half: [f64; 3], which: WeldFaces) -> Vec<Face> {
    let [hx, hy, hz] = half;
    let mut faces = vec![Face { n: [0.0, 0.0, 1.0], t: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], half: [hx, hy] }];
    if which == WeldFaces::TopAndSides {
        faces.push(Face { n: [1.0, 0.0, 0.0], t: [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], half: [hy, hz] });
        faces.push(Face { n: [-1.0, 0.0, 0.0], t: [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], half: [hy, hz] });
        faces.push(Face { n: [0.0, 1.0, 0.0], t: [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], half: [hx, hz] });
        faces.push(Face { n: [0.0, -1.0, 0.0], t: [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], half: [hx, hz] });
    }
    faces
}

fn draw_welds(job: &TrainingJob, n_robots: usize, products: &[SampledProduct], rng: &mut Rng, notes: &mut Vec<String>) -> Vec<SampledWeld> {
    let spec = &job.welds;
    if products.is_empty() || n_robots == 0 {
        return Vec::new();
    }
    let r = n_robots as u32;
    let hi = spec.total[1].min(spec.per_robot_max.max(1) * r);
    let lo = spec.total[0].max(r + 1).min(hi);
    let total = rng.int([lo, hi]) as usize;

    // Per product, then trimmed or padded to the total.
    let mut counts: Vec<usize> = products.iter().map(|_| rng.int(spec.per_product) as usize).collect();
    while counts.iter().sum::<usize>() > total {
        let nonzero: Vec<usize> = (0..counts.len()).filter(|i| counts[*i] > 0).collect();
        counts[nonzero[rng.index(nonzero.len())]] -= 1;
    }
    while counts.iter().sum::<usize>() < total {
        let i = rng.index(counts.len());
        counts[i] += 1;
    }

    let mut welds: Vec<SampledWeld> = Vec::new();
    let mut dropped = 0;
    for (k, (product, count)) in products.iter().zip(counts).enumerate() {
        let faces = faces(product.half, spec.faces);
        let areas: Vec<f64> = faces.iter().map(|f| f.half[0] * f.half[1]).collect();
        let (s, c) = product.yaw_deg.to_radians().sin_cos();
        let rotate = |v: [f64; 3]| [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
        for _ in 0..count {
            let placed = (0..WELD_TRIES).find_map(|_| {
                let face = &faces[weighted(&areas, rng)];
                let a = (face.half[0] - spec.edge_inset_m).max(0.0);
                let b = (face.half[1] - spec.edge_inset_m).max(0.0);
                let (u, v) = (rng.uniform([-a, a]), rng.uniform([-b, b]));
                let local: [f64; 3] = std::array::from_fn(|i| {
                    face.n[i] * product.half.iter().zip(face.n).map(|(h, n)| h * n.abs()).sum::<f64>() + face.t[0][i] * u + face.t[1][i] * v
                });
                let offset = rotate(local);
                let position = [product.center[0] + offset[0], product.center[1] + offset[1], product.center[2] + offset[2]];
                let spaced = welds.iter().all(|w| dist(w.position, position) >= spec.min_spacing_m);
                spaced.then(|| {
                    let tilt = rng.uniform(spec.tilt_deg).to_radians();
                    let az = rng.unit() * TAU;
                    let (st, ct) = tilt.sin_cos();
                    let approach: [f64; 3] = std::array::from_fn(|i| -face.n[i] * ct + st * (az.cos() * face.t[0][i] + az.sin() * face.t[1][i]));
                    SampledWeld { position, approach: rotate(approach), product: k }
                })
            });
            match placed {
                Some(w) => welds.push(w),
                None => dropped += 1,
            }
        }
    }
    if dropped > 0 {
        notes.push(format!("{dropped} weld(s) found no room at {:.0} mm spacing", spec.min_spacing_m * 1000.0));
    }
    welds
}

fn weighted(weights: &[f64], rng: &mut Rng) -> usize {
    let total: f64 = weights.iter().sum();
    let mut x = rng.unit() * total;
    for (i, w) in weights.iter().enumerate() {
        if x < *w {
            return i;
        }
        x -= w;
    }
    weights.len() - 1
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use factoryos_protocol::training::CellRobot;
    use std::collections::BTreeMap;
    use std::f64::consts::PI;

    fn job() -> TrainingJob {
        let mut job = TrainingJob::default();
        job.robots.models = vec!["a.urdf".into(), "b.urdf".into()];
        job
    }

    #[test]
    fn the_same_seed_draws_the_same_cell() {
        let cell = TrainingCell::default();
        assert_eq!(sample(&job(), 7, &cell), sample(&job(), 7, &cell));
        assert_ne!(sample(&job(), 7, &cell), sample(&job(), 8, &cell));
    }

    #[test]
    fn samples_keep_the_jobs_limits() {
        let cell = TrainingCell::default();
        let job = job();
        for seed in 0..200 {
            let s = sample(&job, seed, &cell);
            let r = s.robots.len();
            assert!((1..=4).contains(&r), "seed {seed}: {r} robots");
            assert!((1..=4).contains(&s.products.len()));
            assert!(s.welds.len() <= 6 * r && s.welds.len() <= 24, "seed {seed}: {} welds for {r} robots", s.welds.len());
            for p in &s.products {
                assert!((0.15..=0.5).contains(&p.half[0]) && (0.25..=0.65).contains(&p.half[2]));
            }
            let rects: Vec<Rect> = s.products.iter().map(Rect::of).collect();
            for robot in &s.robots {
                for rect in &rects {
                    assert!(rect.distance([robot.position[0], robot.position[1]]) >= 0.8 - 1e-9);
                }
            }
            for (i, a) in s.robots.iter().enumerate() {
                assert_eq!(a.model, job.robots.models[i % 2]);
                for b in &s.robots[i + 1..] {
                    assert!((a.position[0] - b.position[0]).hypot(a.position[1] - b.position[1]) >= 1.5 - 1e-9);
                }
            }
            for w in &s.welds {
                let p = &s.products[w.product];
                assert!((w.position[2] - 2.0 * p.half[2]).abs() < 1e-9, "top welds sit on the top");
                assert!(w.approach[2] <= -(20f64.to_radians().cos()) + 1e-9);
            }
        }
    }

    #[test]
    fn a_fixed_robot_count_and_cube_products() {
        let mut job = job();
        job.robots.count_weights = BTreeMap::from([(3, 1.0)]);
        job.products.square = true;
        let s = sample(&job, 3, &TrainingCell::default());
        assert_eq!(s.robots.len(), 3);
        assert!(s.products.iter().all(|p| p.half[0] == p.half[1]));
    }

    #[test]
    fn cell_jitter_starts_from_the_cells_bases() {
        let mut job = job();
        job.robots.placement = Placement::CellJitter;
        job.robots.jitter_m = 0.1;
        job.robots.count_weights = BTreeMap::from([(2, 1.0)]);
        job.products.from_cell = true;
        let cell = TrainingCell {
            name: "c".into(),
            robots: vec![
                CellRobot { id: "r1".into(), model: "a.urdf".into(), position: [2.0, 0.0, 0.0], yaw_deg: 180.0 },
                CellRobot { id: "r2".into(), model: "a.urdf".into(), position: [-2.0, 0.0, 0.0], yaw_deg: 0.0 },
            ],
            products: vec![SampledProduct { name: "p".into(), center: [0.0, 0.0, 0.4], half: [0.5, 0.3, 0.4], yaw_deg: 0.0 }],
            welds: 0,
        };
        let s = sample(&job, 1, &cell);
        assert_eq!(s.products, cell.products);
        assert_eq!(s.robots.len(), 2);
        for (robot, base) in s.robots.iter().zip(&cell.robots) {
            assert!((robot.position[0] - base.position[0]).hypot(robot.position[1] - base.position[1]) <= 0.1 + 1e-9);
        }
    }

    #[test]
    fn side_welds_point_into_their_face() {
        let mut job = job();
        job.welds.faces = WeldFaces::TopAndSides;
        job.welds.tilt_deg = [0.0, 0.0];
        for seed in 0..50 {
            let s = sample(&job, seed, &TrainingCell::default());
            for w in &s.welds {
                let p = &s.products[w.product];
                // Back along the approach from the weld is outside the box.
                let out = [w.position[0] - w.approach[0] * 0.05, w.position[1] - w.approach[1] * 0.05, w.position[2] - w.approach[2] * 0.05];
                assert!(Rect::of(p).distance([out[0], out[1]]) > 0.0 || out[2] > 2.0 * p.half[2]);
            }
        }
    }

    #[test]
    fn rectangles() {
        let a = Rect { c: [0.0, 0.0], h: [1.0, 0.5], yaw: 0.0 };
        let b = Rect { c: [3.0, 0.0], h: [1.0, 0.5], yaw: 0.0 };
        assert!((a.gap(&b) - 1.0).abs() < 1e-9);
        assert!((a.support([1.0, 0.0]) - 1.0).abs() < 1e-9);
        assert!((a.support([0.0, 1.0]) - 0.5).abs() < 1e-9);
        let turned = Rect { c: [1.5, 0.0], h: [1.0, 0.5], yaw: PI / 2.0 };
        assert!(a.overlaps(&turned));
    }
}
