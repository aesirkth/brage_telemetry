## Brage firmware
### Building
Brage is built using Rust and the Embassy framework. `cargo build --release`. I don't have a proper build guide yet but cargo should point out the missing dependencies (I think). Cargo should fetch most of the dependencies itself. You might need to install at least rustup and Probe-rs yourself.

To flash the board, just run `cargo run --release`
