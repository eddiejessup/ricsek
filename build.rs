use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let cuda_path = PathBuf::from("src/cuda");

    println!("cargo:rustc-link-search=native=/usr/lib/x86_64-linux-gnu");
    println!(
        "cargo:rustc-link-search=native={}",
        out_path.to_str().unwrap()
    );
    // Inform cargo to link the CUDA library
    println!("cargo:rustc-link-lib=cudart");

    println!(
        "cargo:rerun-if-changed={}/common.h",
        cuda_path.to_str().unwrap()
    );
    println!(
        "cargo:rerun-if-changed={}/common.cu",
        cuda_path.to_str().unwrap()
    );

    let header_filename = "cuda_ricsek";
    let header_path = cuda_path.join(format!("{}.h", header_filename));
    println!("cargo:rerun-if-changed={}", header_path.to_str().unwrap());

    bindgen::Builder::default()
        .header(header_path.to_str().unwrap())
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings")
        .write_to_file(out_path.join(format!("{}_bindings.rs", header_filename)))
        .expect("Couldn't write bindings!");

    for component in vec!["fluid", "electro"] {
        let lib_src_path = cuda_path.join(format!("{}.cu", component));
        let lib_out_path = out_path.join(format!("lib{}.so", component));

        // Run the CUDA compiler
        Command::new("nvcc")
            .args(&[
                "-shared",
                "-Xcompiler",
                "-fPIC",
                lib_src_path.to_str().unwrap(),
                "-I",
                cuda_path.to_str().unwrap(),
                "-o",
                lib_out_path.to_str().unwrap(),
            ])
            .status()
            .unwrap();

        println!("cargo:rustc-link-lib={}", component);
        println!("cargo:rerun-if-changed={}", lib_src_path.to_str().unwrap());
    }
}
