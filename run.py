"""Run the four benchmark executables interleaved and compare them.

Every round runs each executable once, rotating the order, so thermal and
frequency drift lands on every language equally instead of on whichever ran
last. Checksums must agree across every run of every language; a mismatch is
a failed comparison, not a data point.

    python run.py                 5 rounds, 5 reps each
    python run.py --rounds 10
    python run.py --langs c zig

Writes results/latest.md, results/latest.json and the SVG charts.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import platform
import statistics
import subprocess
import sys

import chart
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "out"
RESULTS = ROOT / "results"
EXE = ".exe" if os.name == "nt" else ""
LANGS = ["c", "cpp", "zig", "rust"]
LABEL = {"c": "C", "cpp": "C++", "zig": "Zig", "rust": "Rust"}
VARIANTS = {"world4safe": "world4"}


@dataclass
class Sample:
    min_ns: int
    median_ns: int
    checksum: str


@dataclass
class Series:
    samples: list[Sample] = field(default_factory=list)

    def best_ns(self) -> int:
        return min(s.min_ns for s in self.samples)

    def median_ns(self) -> float:
        return statistics.median(s.median_ns for s in self.samples)

    def checksums(self) -> set[str]:
        return {s.checksum for s in self.samples}


def run_once(lang: str, reps: int, warmup: int) -> dict[str, Sample]:
    exe = OUT / f"bench_{lang}{EXE}"
    proc = subprocess.run([str(exe), str(reps), str(warmup)], capture_output=True, text=True, check=True)
    out: dict[str, Sample] = {}
    for line in proc.stdout.splitlines():
        name, min_ns, median_ns, checksum = line.split()
        out[name] = Sample(int(min_ns), int(median_ns), checksum)
    return out


def cpu_name() -> str:
    if os.name == "nt":
        try:
            import winreg

            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0")
            return winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
        except OSError:
            pass
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or platform.machine()


def tool_version(cmd: list[str]) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout.strip().splitlines()[0]
    except (OSError, subprocess.CalledProcessError, IndexError):
        return "unknown"


def build(zig: str, cargo: str) -> None:
    env = dict(os.environ, ZIG=zig, CARGO=cargo)
    subprocess.run(["make", "-s", "build"], cwd=ROOT, env=env, check=True)


def fmt_ms(ns: float) -> str:
    return f"{ns / 1e6:.2f}"


def render(results: dict[str, dict[str, Series]], langs: list[str], meta: dict[str, str]) -> str:
    lines = ["# Results", ""]
    for k, v in meta.items():
        lines.append(f"- {k}: {v}")
    lines.append("")
    lines.append("Times in milliseconds per repetition. `best` is the fastest single repetition seen; `median` is the median of each run's median. `rel` is `median` relative to the fastest language for that kernel.")
    lines.append("")
    for case, per_lang in results.items():
        if case in VARIANTS:
            continue
        fastest = min(per_lang[l].median_ns() for l in langs)
        lines.append(f"## {case}")
        lines.append("")
        lines.append("| language | best | median | rel |")
        lines.append("|---|---:|---:|---:|")
        for lang in langs:
            s = per_lang[lang]
            lines.append(f"| {LABEL[lang]} | {fmt_ms(s.best_ns())} | {fmt_ms(s.median_ns())} | {s.median_ns() / fastest:.2f}x |")
        lines.append("")
    for case, base in VARIANTS.items():
        if case not in results:
            continue
        lines.append(f"## {case}")
        lines.append("")
        lines.append(f"A variant of `{base}`, same checksum.")
        lines.append("")
        lines.append("| language | best | median | rel to base |")
        lines.append("|---|---:|---:|---:|")
        for lang, s in results[case].items():
            lines.append(f"| {LABEL[lang]} | {fmt_ms(s.best_ns())} | {fmt_ms(s.median_ns())} | {s.median_ns() / results[base][lang].median_ns():.2f}x |")
        lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rounds", type=int, default=5, help="interleaved rounds (default 5)")
    ap.add_argument("--reps", type=int, default=5, help="repetitions per executable per round (default 5)")
    ap.add_argument("--warmup", type=int, default=2, help="untimed repetitions before each run (default 2)")
    ap.add_argument("--langs", nargs="+", choices=LANGS, default=LANGS)
    ap.add_argument("--no-build", action="store_true", help="skip make; use what is in out/")
    ap.add_argument("--zig", default=os.environ.get("ZIG", "zig"), help="zig executable for the build")
    ap.add_argument("--cargo", default=os.environ.get("CARGO", "cargo"), help="cargo executable for the build")
    args = ap.parse_args()

    if not args.no_build:
        build(args.zig, args.cargo)

    langs: list[str] = args.langs
    results: dict[str, dict[str, Series]] = {}
    for round_i in range(args.rounds):
        order = langs[round_i % len(langs):] + langs[: round_i % len(langs)]
        for lang in order:
            for case, sample in run_once(lang, args.reps, args.warmup).items():
                results.setdefault(case, {}).setdefault(lang, Series()).samples.append(sample)
        print(f"round {round_i + 1}/{args.rounds} done", file=sys.stderr)

    mismatched = False
    for case, per_lang in results.items():
        sums = {lang: s.checksums() for lang, s in per_lang.items()}
        union = set().union(*sums.values())
        if case in VARIANTS:
            union |= set().union(*(results[VARIANTS[case]][lang].checksums() for lang in per_lang))
        if len(union) != 1:
            mismatched = True
            print(f"checksum mismatch in {case}: {sums}", file=sys.stderr)
    if mismatched:
        return 1

    meta = {
        "date": dt.date.today().isoformat(),
        "cpu": cpu_name(),
        "os": f"{platform.system()} {platform.release()}",
        "c/c++": tool_version([os.environ.get("CC", "clang"), "--version"]),
        "zig": tool_version([args.zig, "version"]),
        "rust": tool_version([args.cargo, "--version"]),
        "rounds": f"{args.rounds} x {args.reps} reps, {args.warmup} warmup",
    }
    report = render(results, langs, meta)
    print()
    print(report)

    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "latest.md").write_text(report + "\n", encoding="utf-8")
    (RESULTS / "latest.json").write_text(
        json.dumps(
            {
                "meta": meta,
                "results": {
                    case: {lang: [s.__dict__ for s in series.samples] for lang, series in per_lang.items()}
                    for case, per_lang in results.items()
                },
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    chart.main()
    return 0


if __name__ == "__main__":
    sys.exit(main())
