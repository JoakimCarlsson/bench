//! Entry point: `bench_rust [reps] [warmup]`, one output line per kernel.
mod aabb_tree;
mod box_collision;
mod boxbox;
mod broad_phase;
mod broadphase;
mod bvh;
mod chunkmap;
mod colour;
mod constraint_graph;
mod contact;
mod contact_recycle;
mod contact_solver;
mod dda;
mod decompose;
mod flood;
mod gas;
mod harness;
mod hash;
mod integrate;
mod islands;
mod mass;
mod mips;
mod raycast;
mod rigid_body;
mod simd;
mod slotmap;
mod solve;
mod sort;
mod surface;
mod sweep;
mod task_pool;
mod transform;
mod unproject;
mod vecmath;
mod wide;
mod world;

use harness::run_case;
use std::process::ExitCode;

/// Parses the benchmark arguments and runs every kernel in order.
fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let reps = parse_or(args.next(), 10);
    let warmup = parse_or(args.next(), 2);
    if !(1..=1024).contains(&reps) {
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
        && run_case::<wide::Wide>(reps, warmup)
        && run_case::<broadphase::BroadPhaseCase>(reps, warmup)
        && run_case::<gas::Gas>(reps, warmup)
        && run_case::<world::WorldCase<1>>(reps, warmup)
        && run_case::<world::WorldCase<4>>(reps, warmup)
        && run_case::<slotmap::SlotMap>(reps, warmup)
        && run_case::<chunkmap::ChunkMap>(reps, warmup)
        && run_case::<sort::Sort>(reps, warmup);

    if ok { ExitCode::SUCCESS } else { ExitCode::from(3) }
}

/// The argument as a number, or `default` when absent or malformed.
fn parse_or(arg: Option<String>, default: usize) -> usize {
    arg.and_then(|s| s.parse().ok()).unwrap_or(default)
}
