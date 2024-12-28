use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    // Check for CUDA availability
    let cuda_available = Command::new("nvcc")
        .arg("--version")
        .output()
        .map(|output| output.status.success())
        .unwrap_or(false);

    if !cuda_available {
        println!("cargo:rustc-cfg=feature=\"no_cuda\"");
        return;
    }
    println!("cargo:rustc-cfg=feature=\"cuda\"");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let lib_paths = match std::env::var("CARGO_CFG_TARGET_OS").unwrap().as_str() {
        "linux" => vec!["/usr/lib/x86_64-linux-gnu"],
        "macos" => vec!["/usr/local/lib", "/opt/homebrew/lib"],
        "windows" => vec!["C:\\path\\to\\windows\\libs"],
        _ => {
            panic!("Unsupported operating system");
        }
    };

    // Loop over lib paths and out path, adding them to the link search path
    for search_path in lib_paths
        .iter()
        .chain(std::iter::once(&out_path.to_str().unwrap()))
    {
        println!("cargo:rustc-link-search=native={}", search_path);
    }

    let numerics_path = PathBuf::from("src/numerics");

    // Generate bindings for the interface
    let interface_filename = "cuda_interface";
    let interface_path = numerics_path.join(format!("{}.h", interface_filename));
    println!(
        "cargo:rerun-if-changed={}",
        interface_path.to_str().unwrap()
    );
    bindgen::Builder::default()
        .header(interface_path.to_str().unwrap())
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(out_path.join(format!("{}_bindings.rs", interface_filename)))
        .expect("Couldn't write bindings!");

    println!(
        "cargo:rerun-if-changed={}/cuda_common.h",
        numerics_path.to_str().unwrap()
    );

    // Inform cargo to link the CUDA library
    println!("cargo:rustc-link-lib=cudart");

    for component in ["fluid", "electro"] {
        let lib_out_path = out_path.join(format!("lib{}.so", component));

        let lib_src_path = numerics_path.join(format!("{}.cu", component));
        // Run the CUDA compiler
        let output = Command::new("nvcc")
            .args([
                "-shared",
                "-Xcompiler",
                "-fPIC",
                lib_src_path.to_str().unwrap(),
                "-I",
                numerics_path.to_str().unwrap(),
                "-o",
                lib_out_path.to_str().unwrap(),
            ])
            .output()
            .expect("Failed to execute nvcc");

        if !output.status.success() {
            panic!(
                "Failed to compile {}. Error:\n{}",
                lib_src_path.display(),
                String::from_utf8_lossy(&output.stderr)
            );
        }

        println!("cargo:rustc-link-lib={}", component);
        println!("cargo:rerun-if-changed={}", lib_src_path.to_str().unwrap());
    }
}
