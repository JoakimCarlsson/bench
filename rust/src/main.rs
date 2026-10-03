//! Entry point: `bench_rust [reps] [warmup]`, one output line per kernel.
mod boxbox;
mod bvh;
mod chunkmap;
mod colour;
mod dda;
mod decompose;
mod flood;
mod harness;
mod hash;
mod integrate;
mod islands;
mod mass;
mod mips;
mod raycast;
mod slotmap;
mod solve;
mod sort;
mod surface;
mod sweep;
mod transform;
mod unproject;
mod vecmath;

use harness::run_case;
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let reps = parse_or(args.next(), 10);
    let warmup = parse_or(args.next(), 2);
    if reps < 1 || reps > 1024 {
        eprintln!("reps must be 1..1024");
        return ExitCode::from(1);
    }

    let ok = run_case::<dda::Dda>(reps, warmup)
        && run_case::<flood::Flood>(reps, warmup)
        && run_case::<surface::Surface>(reps, warmup)
        && run_case::<mips::Mips>(reps, warmup)
        && run_case::<mass::Mass>(reps, warmup)
        && run_case::<sweep::Sweep>(reps, warmup)
        && run_case::<bvh::Bvh>(reps, warmup)
        && run_case::<islands::Islands>(reps, warmup)
        && run_case::<colour::Colour>(reps, warmup)
        && run_case::<solve::Solve>(reps, warmup)
        && run_case::<integrate::Integrate>(reps, warmup)
        && run_case::<transform::Transform>(reps, warmup)
        && run_case::<unproject::Unproject>(reps, warmup)
        && run_case::<decompose::Decompose>(reps, warmup)
        && run_case::<raycast::Raycast>(reps, warmup)
        && run_case::<boxbox::BoxBox>(reps, warmup)
        && run_case::<slotmap::SlotMap>(reps, warmup)
        && run_case::<chunkmap::ChunkMap>(reps, warmup)
        && run_case::<sort::Sort>(reps, warmup);

    if ok { ExitCode::SUCCESS } else { ExitCode::from(3) }
}

fn parse_or(arg: Option<String>, default: usize) -> usize {
    arg.and_then(|s| s.parse().ok()).unwrap_or(default)
}
