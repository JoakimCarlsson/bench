"""Time how long each language takes to build the benchmark sources.

    python buildtime.py                  3 timings of each case
    python buildtime.py --repeats 5
    python buildtime.py --zig /path/to/zig

Works on a copy of the sources under ~/.cache/scratch/buildtime so out/ and
rust/target are left alone, and writes results/build-times.json and
results/build-times.md. Quit other CPU-heavy work first.

Cases per language:
  clean        everything from nothing, the way the Makefile builds it
  clean -j     C and C++ only: one compile per file, run in parallel
  one file     edit one mid-size source and rebuild
  one header   edit the math header every other file includes and rebuild
               (C and C++ recompile exactly the files that depend on it;
               Zig and Rust always rebuild the whole program)
  check        Rust only: `cargo check` after editing one file, the loop an
               editor runs
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import re
import shutil
import statistics
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
SCRATCH = Path.home() / ".cache" / "scratch" / "buildtime"
LABEL = {"c": "C", "cpp": "C++", "zig": "Zig", "rust": "Rust"}


def makefile_flags() -> dict[str, str]:
    """CFLAGS and CXXFLAGS as the Makefile defines them."""
    text = (ROOT / "Makefile").read_text(encoding="utf-8")
    flags = {}
    for name in ("CFLAGS", "CXXFLAGS"):
        match = re.search(rf"^{name}\s*:=\s*(.+)$", text, re.MULTILINE)
        flags[name] = match.group(1).replace("$(VKFLAGS)", "").strip() if match else ""
    return flags


def run(cmd: list[str], cwd: Path, env: dict[str, str] | None = None) -> float:
    """Run a command and return its wall time in seconds."""
    start = time.perf_counter()
    subprocess.run(cmd, cwd=cwd, env=env, check=True, capture_output=True)
    return time.perf_counter() - start


def fresh_copy() -> Path:
    """A clean copy of the sources in the scratch directory."""
    if SCRATCH.exists():
        shutil.rmtree(SCRATCH)
    SCRATCH.mkdir(parents=True)
    for name in ("c", "cpp", "zig", "rust"):
        shutil.copytree(ROOT / name, SCRATCH / name, ignore=shutil.ignore_patterns("target", ".zig-cache", "zig-out"))
    return SCRATCH


def touch_content(path: Path) -> None:
    """Change a file's content so every build system sees an edit."""
    with path.open("a", encoding="utf-8") as handle:
        handle.write("\n")


def sources(directory: Path, suffix: str) -> list[Path]:
    """Source files of a language, sorted."""
    return sorted((directory / "src").glob(f"*{suffix}"))


def compile_one(compiler: str, flags: list[str], source: Path, out_dir: Path) -> float:
    """Compile one translation unit to an object file."""
    return run([compiler, *flags, "-c", str(source), "-o", str(out_dir / (source.stem + ".o"))], source.parent)


def link(compiler: str, out_dir: Path, binary: Path) -> float:
    """Link every object in a directory."""
    objects = sorted(str(p) for p in out_dir.glob("*.o"))
    return run([compiler, *objects, "-o", str(binary), "-lm", "-pthread"], out_dir)


def dependents(compiler: str, flags: list[str], files: list[Path], header: Path) -> list[Path]:
    """The sources whose preprocessed dependency list contains `header`."""
    found = []
    for source in files:
        deps = subprocess.run([compiler, *flags, "-MM", str(source)], cwd=source.parent, check=True, capture_output=True, text=True).stdout
        if header.name in {Path(token).name for token in deps.replace("\\\n", " ").split()}:
            found.append(source)
    return found


def build_c_family(key: str, compiler: str, flags_text: str, suffix: str, header_name: str, repeats: int, jobs: int) -> dict[str, float]:
    """Time the C or C++ cases."""
    directory = SCRATCH / key
    out_dir = SCRATCH / f"{key}-obj"
    binary = SCRATCH / f"{key}-bin"
    flags = flags_text.split()
    files = sources(directory, suffix)
    header = directory / "src" / header_name
    edit = directory / "src" / f"world{suffix}"

    def clean_monolithic() -> float:
        return run([compiler, *flags, "-o", str(binary), *[str(f) for f in files], "-lm", "-pthread"], directory)

    def clean_parallel() -> float:
        if out_dir.exists():
            shutil.rmtree(out_dir)
        out_dir.mkdir()
        start = time.perf_counter()
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
            list(pool.map(lambda f: compile_one(compiler, flags, f, out_dir), files))
        link(compiler, out_dir, binary)
        return time.perf_counter() - start

    def one_file() -> float:
        touch_content(edit)
        return compile_one(compiler, flags, edit, out_dir) + link(compiler, out_dir, binary)

    def one_header() -> float:
        touch_content(header)
        affected = dependents(compiler, flags, files, header)
        start = time.perf_counter()
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
            list(pool.map(lambda f: compile_one(compiler, flags, f, out_dir), affected))
        link(compiler, out_dir, binary)
        return time.perf_counter() - start

    results = {"clean": statistics.median(clean_monolithic() for _ in range(max(1, repeats - 1)))}
    results["clean -j"] = statistics.median(clean_parallel() for _ in range(repeats))
    results["one file"] = statistics.median(one_file() for _ in range(repeats))
    results["one header"] = statistics.median(one_header() for _ in range(repeats))
    results["header dependents"] = float(len(dependents(compiler, flags, files, header)))
    results["files"] = float(len(files))
    return results


def build_zig(zig: str, repeats: int) -> dict[str, float]:
    """Time the Zig cases."""
    directory = SCRATCH / "zig"
    cache = SCRATCH / "zig-cache"
    prefix = SCRATCH / "zig-out"
    cmd = [zig, "build", "--cache-dir", str(cache), "--prefix", str(prefix), "--prefix-exe-dir", "."]

    def clean() -> float:
        if cache.exists():
            shutil.rmtree(cache)
        return run(cmd, directory)

    def edit_file(name: str) -> float:
        touch_content(directory / "src" / name)
        return run(cmd, directory)

    results = {"clean": statistics.median(clean() for _ in range(max(1, repeats - 1)))}
    results["one file"] = statistics.median(edit_file("world.zig") for _ in range(repeats))
    results["one header"] = statistics.median(edit_file("vecmath.zig") for _ in range(repeats))
    results["files"] = float(len(sources(directory, ".zig")))
    return results


def build_rust(cargo: str, repeats: int) -> dict[str, float]:
    """Time the Rust cases with the benchmark's profile and with Cargo's default release profile."""
    directory = SCRATCH / "rust"
    target = SCRATCH / "rust-target"
    bench = ["--release", "--target-dir", str(target)]
    default_env = dict(os.environ, CARGO_PROFILE_RELEASE_LTO="false", CARGO_PROFILE_RELEASE_CODEGEN_UNITS="16")
    quiet = dict(os.environ)

    def clean(env: dict[str, str]) -> float:
        if target.exists():
            shutil.rmtree(target)
        return run([cargo, "build", *bench], directory, env)

    def edit(name: str, env: dict[str, str], extra: list[str]) -> float:
        touch_content(directory / "src" / name)
        return run([cargo, *extra, *bench], directory, env)

    results = {"clean": statistics.median(clean(quiet) for _ in range(max(1, repeats - 1)))}
    results["one file"] = statistics.median(edit("world.rs", quiet, ["build"]) for _ in range(repeats))
    results["one header"] = statistics.median(edit("vecmath.rs", quiet, ["build"]) for _ in range(repeats))
    results["clean default release"] = statistics.median(clean(default_env) for _ in range(max(1, repeats - 1)))
    results["one file default release"] = statistics.median(edit("world.rs", default_env, ["build"]) for _ in range(repeats))
    run([cargo, "check", *bench], directory, quiet)
    results["check"] = statistics.median(edit("world.rs", quiet, ["check"]) for _ in range(repeats))
    results["files"] = float(len(sources(directory, ".rs")))
    return results


def lines_and_sizes() -> dict[str, dict[str, float]]:
    """Source lines per language and the size of the built executables, when present."""
    out: dict[str, dict[str, float]] = {}
    for key, suffixes in (("c", (".c", ".h")), ("cpp", (".cpp", ".hpp")), ("zig", (".zig",)), ("rust", (".rs",))):
        total = 0
        for suffix in suffixes:
            for path in (ROOT / key / "src").glob(f"*{suffix}"):
                total += len(path.read_text(encoding="utf-8").splitlines())
        binary = ROOT / "out" / f"bench_{key}"
        out[key] = {"lines": float(total), "binary_kib": binary.stat().st_size / 1024 if binary.exists() else 0.0}
    return out


def fmt(seconds: float) -> str:
    """Seconds with one decimal below ten and none above."""
    return f"{seconds:.1f}" if seconds < 10 else f"{seconds:.0f}"


def render(data: dict[str, dict[str, float]], extra: dict[str, dict[str, float]], meta: dict[str, str]) -> str:
    """The Markdown report."""
    lines = ["# Build times", ""]
    for key, value in meta.items():
        lines.append(f"- {key}: {value}")
    lines += ["", "Seconds, median of the repeats; the `-j` rows use every hardware thread.", ""]
    lines += ["| case | C | C++ | Zig | Rust |", "|---|---:|---:|---:|---:|"]
    for case in ("clean", "clean -j", "one file", "one header"):
        cells = [fmt(data[k][case]) if case in data[k] else "n/a" for k in ("c", "cpp", "zig", "rust")]
        lines.append(f"| {case} | " + " | ".join(cells) + " |")
    rust = data["rust"]
    lines += ["", "Rust with Cargo's default release profile (no LTO, 16 codegen units) instead of the benchmark's (fat LTO, 1 codegen unit):", ""]
    lines += [f"- clean: {fmt(rust['clean default release'])} s, one file: {fmt(rust['one file default release'])} s", f"- `cargo check` after one edit: {fmt(rust['check'])} s", ""]
    lines += ["| | C | C++ | Zig | Rust |", "|---|---:|---:|---:|---:|"]
    lines.append("| source lines | " + " | ".join(f"{extra[k]['lines']:,.0f}" for k in ("c", "cpp", "zig", "rust")) + " |")
    lines.append("| executable KiB | " + " | ".join(f"{extra[k]['binary_kib']:,.0f}" for k in ("c", "cpp", "zig", "rust")) + " |")
    lines.append("")
    lines.append(f"Files recompiled when the math header changes: {data['c']['header dependents']:.0f} of {data['c']['files']:.0f} in C, {data['cpp']['header dependents']:.0f} of {data['cpp']['files']:.0f} in C++.")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    """Run every case and write the reports."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--langs", nargs="+", choices=["c", "cpp", "zig", "rust"], default=["c", "cpp", "zig", "rust"], help="rerun only these; the rest are read from results/build-times.json")
    ap.add_argument("--zig", default=os.environ.get("ZIG", "zig"))
    ap.add_argument("--cargo", default=os.environ.get("CARGO", "cargo"))
    ap.add_argument("--cc", default=os.environ.get("CC", "clang"))
    ap.add_argument("--cxx", default=os.environ.get("CXX", "clang++"))
    args = ap.parse_args()
    fresh_copy()
    flags = makefile_flags()
    jobs = os.cpu_count() or 4
    previous = json.loads((RESULTS / "build-times.json").read_text(encoding="utf-8"))["build"] if (RESULTS / "build-times.json").exists() else {}
    runners = {
        "c": lambda: build_c_family("c", args.cc, flags["CFLAGS"], ".c", "vecmath.h", args.repeats, jobs),
        "cpp": lambda: build_c_family("cpp", args.cxx, flags["CXXFLAGS"], ".cpp", "vecmath.hpp", args.repeats, jobs),
        "zig": lambda: build_zig(args.zig, args.repeats),
        "rust": lambda: build_rust(args.cargo, args.repeats),
    }
    data = {key: runners[key]() if key in args.langs else previous[key] for key in runners}
    extra = lines_and_sizes()
    meta = {
        "date": time.strftime("%Y-%m-%d"),
        "cpu": next((l.split(":", 1)[1].strip() for l in Path("/proc/cpuinfo").read_text().splitlines() if l.startswith("model name")), "unknown") if Path("/proc/cpuinfo").exists() else "unknown",
        "threads": str(jobs),
        "repeats": str(args.repeats),
    }
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "build-times.json").write_text(json.dumps({"meta": meta, "build": data, "extra": extra}, indent=2), encoding="utf-8")
    (RESULTS / "build-times.md").write_text(render(data, extra, meta), encoding="utf-8")
    print(render(data, extra, meta))
    shutil.rmtree(SCRATCH, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
