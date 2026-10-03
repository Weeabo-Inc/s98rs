//! Build script for the USB driver crate.
//!
//! The library itself needs nothing from here. The `hid_keyboard` example is a
//! real Cortex-M binary and needs a linker script memory map, so this writes
//! `memory.x` into `OUT_DIR` and points the linker at it, in the same way
//! `../fw/build.rs` does.
//!
//! On a host build (`cargo test --target x86_64-...`) none of this is used; the
//! file is written and the link search path set, and nothing links against it.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
}
