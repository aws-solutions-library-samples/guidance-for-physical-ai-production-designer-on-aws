use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use crate::error::FittingRoomError;

/// The linear value of the sRGB-encoded colour component `c` (the sRGB decoding of IEC 61966-2-1):
/// `c / 12.92` up to 0.04045, `((c + 0.055) / 1.055)^2.4` above. An MTL's `Kd` is sRGB (CAD
/// colours are 8-bit sRGB), UsdPreviewSurface's `diffuseColor` linear (FR-111).
pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// FR-111: the linear diffuse colour of a material whose colour isn't known — a mesh without a
/// material, an MTL that can't be read or lacks the material, a material without a `Kd` line — and
/// of the default material (`factoryos_default`): mid grey (lighter reads as white under Isaac
/// Sim's default lighting).
pub const DEFAULT_DIFFUSE_LINEAR: [f64; 3] = [0.45, 0.45, 0.45];

#[derive(Debug, Clone)]
pub struct Material {
    /// Diffuse colour (`Kd`) as the MTL gives it (sRGB); `None` without a `Kd` line. Written as
    /// UsdPreviewSurface `diffuseColor`, linear: [`Material::diffuse_linear`].
    pub diffuse: Option<[f64; 3]>,
    /// Specular colour (Ks). Used to estimate metallic/roughness via heuristics.
    pub specular: [f64; 3],
    /// Specular exponent (Ns). Higher = sharper highlight.
    pub shininess: f64,
    /// Opacity (`d`). 1.0 = fully opaque.
    pub opacity: f64,
}

impl Default for Material {
    fn default() -> Self {
        Self::placeholder()
    }
}

impl Material {
    /// A material nothing is known of: no colour (so [`DEFAULT_DIFFUSE_LINEAR`]), a soft sheen,
    /// opaque. A `newmtl` starts from it.
    pub fn placeholder() -> Self {
        Self {
            diffuse: None,
            specular: [0.5, 0.5, 0.5],
            shininess: 10.0,
            opacity: 1.0,
        }
    }

    /// FR-111: the linear diffuse colour: the `Kd` decoded from sRGB ([`srgb_to_linear`]), else
    /// [`DEFAULT_DIFFUSE_LINEAR`] (already linear, not decoded).
    pub fn diffuse_linear(&self) -> [f64; 3] {
        self.diffuse
            .map_or(DEFAULT_DIFFUSE_LINEAR, |kd| kd.map(srgb_to_linear))
    }

    /// Not metallic: MTL's `Ks` is a specular colour, not metalness (a painted robot's Ks 0.5 read as
    /// half metal: in an RTX renderer, Isaac Sim's, it mirrors the sky and floor and its paint looks
    /// washed out). MTL has no metalness; higher Ns still means less rough.
    pub fn metallic(&self) -> f64 {
        0.0
    }

    pub fn roughness(&self) -> f64 {
        // Ns ranges 0..1000+; map inversely so high Ns → low roughness.
        (1.0 / (1.0 + self.shininess * 0.05)).clamp(0.05, 1.0)
    }
}

pub fn parse_mtl(path: &Path) -> Result<HashMap<String, Material>, FittingRoomError> {
    let file = File::open(path).map_err(|e| {
        FittingRoomError::UsdWrite(format!("open {} for MTL parse: {}", path.display(), e))
    })?;
    let reader = BufReader::new(file);

    let mut map: HashMap<String, Material> = HashMap::new();
    let mut current_name: Option<String> = None;

    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => continue,
        };
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let mut parts = line.split_whitespace();
        let tag = match parts.next() {
            Some(t) => t,
            None => continue,
        };
        match tag {
            "newmtl" => {
                if let Some(name) = parts.next() {
                    current_name = Some(name.to_string());
                    map.entry(name.to_string())
                        .or_insert_with(Material::placeholder);
                }
            }
            "Kd" | "Ka" | "Ks" => {
                let v: Vec<f64> = parts.filter_map(|p| p.parse::<f64>().ok()).collect();
                if v.len() >= 3 {
                    if let Some(name) = &current_name {
                        if let Some(m) = map.get_mut(name) {
                            let triple = [v[0], v[1], v[2]];
                            match tag {
                                "Kd" => m.diffuse = Some(triple),
                                "Ks" => m.specular = triple,
                                _ => {}
                            }
                        }
                    }
                }
            }
            "Ns" => {
                if let Some(val) = parts.next().and_then(|p| p.parse::<f64>().ok()) {
                    if let Some(name) = &current_name {
                        if let Some(m) = map.get_mut(name) {
                            m.shininess = val;
                        }
                    }
                }
            }
            "d" | "Tr" => {
                if let Some(val) = parts.next().and_then(|p| p.parse::<f64>().ok()) {
                    if let Some(name) = &current_name {
                        if let Some(m) = map.get_mut(name) {
                            // `d` = opacity, `Tr` = transparency (1 - d).
                            m.opacity = if tag == "d" { val } else { 1.0 - val };
                        }
                    }
                }
            }
            _ => {}
        }
    }

    Ok(map)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn parses_single_material() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("a.mtl");
        let mut f = File::create(&p).unwrap();
        writeln!(f, "newmtl Black").unwrap();
        writeln!(f, "Kd 0.1 0.1 0.12").unwrap();
        writeln!(f, "Ks 0.5 0.5 0.5").unwrap();
        writeln!(f, "Ns 25.0").unwrap();
        writeln!(f, "d 0.95").unwrap();

        let mats = parse_mtl(&p).unwrap();
        let m = mats.get("Black").unwrap();
        assert!((m.diffuse.unwrap()[0] - 0.1).abs() < 1e-9);
        assert!((m.shininess - 25.0).abs() < 1e-9);
        assert!((m.opacity - 0.95).abs() < 1e-9);
    }

    #[test]
    fn parses_multiple_materials() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("b.mtl");
        let mut f = File::create(&p).unwrap();
        writeln!(f, "newmtl A").unwrap();
        writeln!(f, "Kd 1 0 0").unwrap();
        writeln!(f, "newmtl B").unwrap();
        writeln!(f, "Kd 0 1 0").unwrap();

        let mats = parse_mtl(&p).unwrap();
        assert_eq!(mats.len(), 2);
        assert!((mats["A"].diffuse.unwrap()[0] - 1.0).abs() < 1e-9);
        assert!((mats["B"].diffuse.unwrap()[1] - 1.0).abs() < 1e-9);
    }

    // FR-111
    #[test]
    fn srgb_to_linear_decodes_srgb() {
        let close = |got: f64, want: f64| assert!((got - want).abs() < 1e-12, "{got} != {want}");
        close(srgb_to_linear(0.5), 0.21404114048223255);
        // A dark and a light channel of the power segment.
        close(srgb_to_linear(0.25), 0.05087608817155679);
        close(srgb_to_linear(0.75), 0.5225215539683918);
        // The linear segment, up to and including its end.
        close(srgb_to_linear(0.02), 0.02 / 12.92);
        close(srgb_to_linear(0.04045), 0.04045 / 12.92);
        assert_eq!(srgb_to_linear(0.0), 0.0);
        close(srgb_to_linear(1.0), 1.0);
        // Continuous at the knee (but for the standard's own 2.3e-9 step) and increasing.
        assert!((srgb_to_linear(0.04045) - srgb_to_linear(0.04045 + 1e-12)).abs() < 3e-9);
        assert!(srgb_to_linear(0.3) < srgb_to_linear(0.31));
        // The MTL parse keeps Kd as given; compose linearises it.
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("c.mtl");
        std::fs::write(&p, "newmtl G\nKd 0.5 0.5 0.5\n").unwrap();
        let g = &parse_mtl(&p).unwrap()["G"];
        assert_eq!(g.diffuse, Some([0.5; 3]));
        close(g.diffuse_linear()[0], 0.21404114048223255);
    }

    // FR-111
    #[test]
    fn a_material_without_kd_gets_the_linear_default() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("d.mtl");
        std::fs::write(
            &p,
            "newmtl Bare\nKs 0.2 0.2 0.2\nNs 40\nd 0.5\nnewmtl Grey\nKd 0.45 0.45 0.45\n",
        )
        .unwrap();
        let mats = parse_mtl(&p).unwrap();
        let bare = &mats["Bare"];
        // No Kd: no colour, so the linear default as is, not decoded and not a placeholder grey;
        // the rest of the material as given.
        assert_eq!(bare.diffuse, None);
        assert_eq!(bare.diffuse_linear(), DEFAULT_DIFFUSE_LINEAR);
        assert_eq!(DEFAULT_DIFFUSE_LINEAR, [0.45; 3]);
        assert_eq!(
            (bare.specular, bare.shininess, bare.opacity),
            ([0.2; 3], 40.0, 0.5)
        );
        // The same as a material nothing is known of.
        assert_eq!(
            Material::placeholder().diffuse_linear(),
            DEFAULT_DIFFUSE_LINEAR
        );
        assert_eq!(Material::default().diffuse_linear(), DEFAULT_DIFFUSE_LINEAR);
        assert_eq!(Material::default().opacity, 1.0);
        // An sRGB Kd of 0.45 is decoded, so it is darker than the default.
        assert!(mats["Grey"].diffuse_linear()[0] < 0.2);
    }
}
