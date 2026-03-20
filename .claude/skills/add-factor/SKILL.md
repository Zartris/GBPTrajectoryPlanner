---
name: add-factor
description: Add a new factor type to the GBP factor graph, touching exactly the 3 required FACTOR EXTENSION POINTs
user-invocable: false
---

When adding a new factor, follow docs/adding_a_factor.md exactly.
Search for `// FACTOR EXTENSION POINT` in crates/gbp-core/ to find all 3 locations.

Changes required in exactly 3 places:
1. `crates/gbp-core/src/factor_node.rs` — add variant to `FactorKind` enum
2. `crates/gbp-core/src/factor_node.rs` — add match arms in `as_factor()` / `as_factor_mut()`
3. `crates/gbp-core/src/lib.rs` — export the new module

Create the factor implementation in `crates/gbp-core/src/<factor_name>.rs`.
The factor must implement the `Factor` trait with `linearise(&self, x: &[f32]) -> (f32, f32)`.
All types must be no_std, no alloc. Use heapless only.

After changes, verify: `cargo check -p gbp-core`
