# Contributing to obd_session_rs

Thanks for looking. Small, focused pull requests are easiest to review.

## Before you open a PR

```
cargo fmt
cargo test --all-features                       # unit + integration + FFI tests
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps  # must build without warnings
```

- **The goldens are the oracle.** `tests/fixtures/golden/*.golden` pin the
  identify output (VIN, ECUs, supported PIDs, addressing) for every fixture in
  `mock_data/`. A change that alters one must say why in the PR; regenerate
  deliberately with `UPDATE_GOLDEN=1 cargo test golden_` and read the diff
  line by line.
- **No captured vehicle data.** Never add a transcript that carries a real VIN,
  calibration id, or serial. Run `tools/synthesize_fixtures.py` on any new
  fixture before committing it; unit tests use synthetic bytes.
- **Adapter behaviour is evidence.** A change to a link handler
  (`src/link/` — ELM327, STN, DVI) should name the adapter and firmware it was
  observed on and, where possible, come with a replay test built from the
  wire log.
- **Transports are feature-gated** (`ble`, `bluetooth-classic`,
  `full-bluetooth`). CI runs `--all-features` on Linux; keep them compiling
  there. Nothing in the core may depend on a transport feature.
- **FFI changes** need matching updates in `cbindgen.toml` and the consumers
  that link the C header. Say in the PR that the header changed.

## Toolchain

`rust-toolchain.toml` pins the compiler because the produced static libraries
are linked together with `obd_equation_rs` into one binary; both crates must
share a toolchain. Bump it in both repos at once.

## License

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this crate by you, as defined in the Apache-2.0 license, shall
be dual licensed as MIT OR Apache-2.0 (see `LICENSE-MIT` and `LICENSE-APACHE`),
without any additional terms or conditions.
