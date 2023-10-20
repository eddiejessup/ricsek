use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let bindings = bindgen::Builder::default()
        .header("src/cuda/linear_fields.h")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");

    let cuda_out_path = out_path.join("liblinear_fields.so");
    let cuda_out_path_str = cuda_out_path.to_str().unwrap();
    // Run the CUDA compiler
    Command::new("nvcc")
        .args(&[
            "-shared",
            "-Xcompiler",
            "-fPIC",
            "src/cuda/linear_fields.cu",
            "-I",
            "src/cuda",
            "-o",
            cuda_out_path_str,
        ])
        .status()
        .unwrap();

    // Inform cargo to link the CUDA library
    println!("cargo:rustc-link-search=native=/usr/lib/x86_64-linux-gnu");
    println!("cargo:rustc-link-lib=cudart");

    // Link the compiled CUDA object file
    println!(
        "cargo:rustc-link-search=native={}",
        out_path.to_str().unwrap()
    );
    println!("cargo:rustc-link-lib=linear_fields");

    // Tell cargo to invalidate the built crate whenever the wrapper changes
    println!("cargo:rerun-if-changed=src/cuda/linear_fields.h");
    println!("cargo:rerun-if-changed=src/cuda/linear_fields.cu");
}
