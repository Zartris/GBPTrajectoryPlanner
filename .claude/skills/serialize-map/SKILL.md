---
name: serialize-map
description: Serialize a YAML map file to binary format for firmware and simulator use
disable-model-invocation: true
---

Run: `cargo run -p simulator -- --serialize-map maps/test_loop_map.yaml maps/test_loop.bin`

After serialization, verify the binary was created and report its size.
If the user specifies a different map file, substitute it in the command.
