"""Measure the peak memory of every kernel in every language.

    python memory.py                 one process per kernel and language
    python memory.py --langs c rust

Each executable takes `[reps] [warmup] [kernel...]`; here it runs a single
kernel for one timed repetition, in a process of its own, started by a tiny C
launcher that reports the child's peak resident set size (ru_maxrss). The size of an
idle process (a kernel name that matches nothing) is subtracted, so the figure
is what the kernel's own state and temporaries add on top of the runtime.

Writes results/memory.json and results/memory.md. Linux only.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out"
RESULTS = ROOT / "results"
LANGS = ["c", "cpp", "zig", "rust"]
LABEL = {"c": "C", "cpp": "C++", "zig": "Zig", "rust": "Rust"}


LAUNCHER = r"""
#include <stdio.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    pid_t pid = fork();
    if (pid == 0) {
        execv(argv[1], argv + 1);
        return 127;
    }
    int status = 0;
    struct rusage usage;
    wait4(pid, &status, 0, &usage);
    fprintf(stderr, "%ld\n", usage.ru_maxrss);
    return status == 0 ? 0 : 1;
}
"""
SCRATCH = Path.home() / ".cache" / "scratch" / "memory"


def build_launcher(cc: str) -> Path:
    """Compile the tiny launcher that reports a child's peak RSS.

    A child inherits its parent's high-water mark, so the measuring process
    must itself be small: Python's own few MiB would hide anything smaller.
    """
    SCRATCH.mkdir(parents=True, exist_ok=True)
    source = SCRATCH / "maxrss.c"
    binary = SCRATCH / "maxrss"
    source.write_text(LAUNCHER, encoding="utf-8")
    subprocess.run([cc, "-O2", "-o", str(binary), str(source)], check=True)
    return binary


def peak_kib(launcher: Path, exe: Path, kernel: str) -> tuple[int, str]:
    """Peak resident KiB of one run of `exe` on one kernel, and its output."""
    proc = subprocess.run([str(launcher), str(exe), "1", "0", kernel], capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"{exe.name} {kernel} failed: {proc.stderr.strip()}")
    return int(proc.stderr.strip().splitlines()[-1]), proc.stdout


def kernel_names(exe: Path) -> list[str]:
    """The kernel names an executable runs, in order."""
    proc = subprocess.run([str(exe), "1", "0"], capture_output=True, text=True, check=True)
    return [line.split()[0] for line in proc.stdout.splitlines()]


def main() -> int:
    """Measure everything and write the reports."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--langs", nargs="+", choices=LANGS, default=LANGS)
    args = ap.parse_args()
    langs: list[str] = args.langs
    launcher = build_launcher(os.environ.get("CC", "clang"))
    names = kernel_names(OUT / f"bench_{langs[0]}")
    baseline = {lang: min(peak_kib(launcher, OUT / f"bench_{lang}", "no-such-kernel")[0] for _ in range(3)) for lang in langs}
    usage: dict[str, dict[str, float]] = {}
    for name in names:
        usage[name] = {}
        for lang in langs:
            peaks = []
            for _ in range(3):
                kib, output = peak_kib(launcher, OUT / f"bench_{lang}", name)
                if not output.startswith(name + " "):
                    raise RuntimeError(f"{lang} printed no line for {name}")
                peaks.append(kib)
            usage[name][lang] = (min(peaks) - baseline[lang]) / 1024
        print(name, {l: round(v, 1) for l, v in usage[name].items()}, file=sys.stderr)

    lines = ["# Memory", "", "Peak resident MiB a kernel adds over an idle process of the same executable (one repetition, minimum of three runs).", ""]
    lines += ["| kernel | " + " | ".join(LABEL[l] for l in langs) + " |", "|---|" + "---:|" * len(langs)]
    for name, per in usage.items():
        lines.append(f"| {name} | " + " | ".join(f"{per[l]:.1f}" for l in langs) + " |")
    lines += ["", "| idle process (MiB) | " + " | ".join(f"{baseline[l] / 1024:.1f}" for l in langs) + " |", ""]
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "memory.json").write_text(json.dumps({"baseline_kib": baseline, "added_mib": usage}, indent=2), encoding="utf-8")
    (RESULTS / "memory.md").write_text("\n".join(lines), encoding="utf-8")
    shutil.rmtree(SCRATCH, ignore_errors=True)
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
