fn main() {
    // Enable collision support in the CMake wrapper only for `collision`
    // feature builds so it finds/links the Coal backend; non-collision builds
    // configure with ENABLE_COLLISION=OFF and are unaffected.
    let collision_enabled = std::env::var_os("CARGO_FEATURE_COLLISION").is_some();
    let mut cmake_config = cmake::Config::new("pinocchio_wrapper");
    cmake_config.define(
        "ENABLE_COLLISION",
        if collision_enabled { "ON" } else { "OFF" },
    );
    let dst = cmake_config.build();
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=pinocchio_wrapper");

    if cfg!(target_os = "macos") {
        println!("cargo:rustc-link-lib=dylib=c++");
    } else {
        println!("cargo:rustc-link-lib=dylib=stdc++");
    }

    let pkg_found = pkg_config::probe_library("pinocchio")
        .or_else(|_| pkg_config::probe_library("pinocchio_default"));

    if let Ok(lib) = pkg_found {
        for path in &lib.link_paths {
            println!("cargo:rustc-link-search=native={}", path.display());
        }
        for lib_name in &lib.libs {
            println!("cargo:rustc-link-lib=dylib={}", lib_name);
        }
    } else {
        let prefix = std::env::var("PINOCCHIO_PREFIX").unwrap_or_default();
        if !prefix.is_empty() {
            let lib64 = std::path::Path::new(&prefix).join("lib64");
            let lib = std::path::Path::new(&prefix).join("lib");
            if lib64.join("libpinocchio_default.so").exists() {
                println!("cargo:rustc-link-search=native={}", lib64.display());
            } else {
                println!("cargo:rustc-link-search=native={}", lib.display());
            }
        }
        println!("cargo:rustc-link-lib=dylib=pinocchio_default");
        println!("cargo:rustc-link-lib=dylib=pinocchio_parsers");
        println!("cargo:rustc-link-lib=dylib=urdfdom_model");
        println!("cargo:rustc-link-lib=dylib=urdfdom_sensor");
        println!("cargo:rustc-link-lib=dylib=urdfdom_world");
        println!("cargo:rustc-link-lib=dylib=boost_filesystem");
    }

    // Coal (hpp-fcl) collision backend linkage, gated behind the `collision`
    // Cargo feature. Cargo exposes enabled features to build scripts via the
    // `CARGO_FEATURE_<NAME>` environment variable (feature-name uppercased with
    // `-` replaced by `_`); `cfg!(feature = ...)` does not reflect the package's
    // features inside a build script, so the env var is the correct guard.
    // When the feature is disabled, no Coal probe or link flags are emitted and
    // the default build is unaffected.
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_COLLISION");
    if collision_enabled {
        let coal = pkg_config::probe_library("coal")
            .or_else(|_| pkg_config::probe_library("hpp-fcl"));

        match coal {
            Ok(lib) => {
                for path in &lib.link_paths {
                    println!("cargo:rustc-link-search=native={}", path.display());
                }
                for lib_name in &lib.libs {
                    println!("cargo:rustc-link-lib=dylib={}", lib_name);
                }
            }
            Err(err) => {
                // Feature enabled but the Coal collision backend is unavailable:
                // panic so the build exits non-zero and produces no artifact.
                panic!(
                    "The `collision` feature is enabled but the Coal (hpp-fcl) \
                     collision dependency could not be found via pkg-config \
                     (tried `coal`, then fallback `hpp-fcl`): {err}. Install the \
                     Coal/hpp-fcl development package or build without the \
                     `collision` feature."
                );
            }
        }
    }

    if cfg!(target_os = "macos") {
        let homebrew_prefix = std::env::var("HOMEBREW_PREFIX")
            .unwrap_or_else(|_| {
                if cfg!(target_arch = "aarch64") {
                    "/opt/homebrew".to_string()
                } else {
                    "/usr/local".to_string()
                }
            });
        println!("cargo:rustc-link-search=native={}/lib", homebrew_prefix);
    }

    println!("cargo:rerun-if-changed=pinocchio_wrapper/");
}
