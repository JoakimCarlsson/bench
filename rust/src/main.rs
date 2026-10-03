//! Entry point: `bench_rust [reps] [warmup] [kernel...]`, one output line per kernel.
mod aabb_tree;
mod anim;
mod animation;
mod animation_clip;
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
mod contact_solver_safe;
mod dda;
mod decompose;
mod flood;
mod gas;
mod harness;
mod hash;
mod integrate;
mod islands;
mod json;
mod json_value;
mod lanes;
mod mass;
mod mips;
mod particles;
mod raycast;
mod rigid_body;
mod simd;
mod slotmap;
mod solve;
mod sort;
mod surface;
mod sweep;
mod task_pool;
mod task_pool_safe;
mod transform;
mod ui;
mod ui_container;
mod ui_control;
mod ui_draw;
mod ui_math;
mod ui_theme;
mod ui_widgets;
mod unproject;
mod vecmath;
mod wide;
mod world;
mod world_safe;

use harness::{Case, run_case};
use std::process::ExitCode;

/// Runs a kernel unless kernel names were given and it is not among them.
fn run_selected<C: Case>(names: &[String], reps: usize, warmup: usize) -> bool {
    if !names.is_empty() && !names.iter().any(|n| n == C::NAME) {
        return true;
    }
    run_case::<C>(reps, warmup)
}

/// Parses `[reps] [warmup] [kernel...]` and runs the selected kernels in order.
fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let reps = parse_or(args.next(), 10);
    let warmup = parse_or(args.next(), 2);
    let names: Vec<String> = args.collect();
    if !(1..=1024).contains(&reps) {
        eprintln!("reps must be 1..1024");
        return ExitCode::from(1);
    }

    let run = |reps, warmup| {
        run_selected::<dda::Dda>(&names, reps, warmup)
            && run_selected::<flood::Flood>(&names, reps, warmup)
            && run_selected::<surface::Surface>(&names, reps, warmup)
            && run_selected::<mips::Mips>(&names, reps, warmup)
            && run_selected::<mass::Mass>(&names, reps, warmup)
            && run_selected::<sweep::Sweep>(&names, reps, warmup)
            && run_selected::<bvh::Bvh>(&names, reps, warmup)
            && run_selected::<islands::Islands>(&names, reps, warmup)
            && run_selected::<colour::Colour>(&names, reps, warmup)
            && run_selected::<solve::Solve>(&names, reps, warmup)
            && run_selected::<integrate::Integrate>(&names, reps, warmup)
            && run_selected::<transform::Transform>(&names, reps, warmup)
            && run_selected::<unproject::Unproject>(&names, reps, warmup)
            && run_selected::<decompose::Decompose>(&names, reps, warmup)
            && run_selected::<raycast::Raycast>(&names, reps, warmup)
            && run_selected::<boxbox::BoxBox>(&names, reps, warmup)
            && run_selected::<wide::Wide>(&names, reps, warmup)
            && run_selected::<broadphase::BroadPhaseCase>(&names, reps, warmup)
            && run_selected::<gas::Gas>(&names, reps, warmup)
            && run_selected::<world::WorldCase<1>>(&names, reps, warmup)
            && run_selected::<world::WorldCase<2>>(&names, reps, warmup)
            && run_selected::<world::WorldCase<4>>(&names, reps, warmup)
            && run_selected::<world::WorldCase<8>>(&names, reps, warmup)
            && run_selected::<world::WorldCase<16>>(&names, reps, warmup)
            && run_selected::<world_safe::WorldSafeCase>(&names, reps, warmup)
            && run_selected::<particles::Particles>(&names, reps, warmup)
            && run_selected::<json::Json>(&names, reps, warmup)
            && run_selected::<anim::Anim>(&names, reps, warmup)
            && run_selected::<ui::Ui>(&names, reps, warmup)
            && run_selected::<slotmap::SlotMap>(&names, reps, warmup)
            && run_selected::<chunkmap::ChunkMap>(&names, reps, warmup)
            && run_selected::<sort::Sort>(&names, reps, warmup)
    };

    if run(reps, warmup) { ExitCode::SUCCESS } else { ExitCode::from(3) }
}

/// The argument as a number, or `default` when absent or malformed.
fn parse_or(arg: Option<String>, default: usize) -> usize {
    arg.and_then(|s| s.parse().ok()).unwrap_or(default)
}
