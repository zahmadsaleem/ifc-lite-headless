#!/usr/bin/env python3
"""Benchmark ifc-lite-headless vs IfcConvert 0.8.4 in Docker."""

import os
import subprocess
import sys
import time
from pathlib import Path


OUTPUT_DIR = Path("/tmp/benchmark_output")


def build_containers():
    print("Building containers...")
    for name, dockerfile in [
        ("ifc-lite-headless", "docker/Dockerfile.ifc2glb"),
        ("ifcconvert", "docker/Dockerfile.ifcconvert"),
    ]:
        subprocess.run(
            ["docker", "build", "-f", dockerfile, "-t", name, ".", "-q"],
            check=True,
            capture_output=True,
        )
    print("Containers ready.\n")


def run_container(image, args, volumes):
    vol_flags = []
    for src, dst, mode in volumes:
        vol_flags.extend(["-v", f"{src}:{dst}:{mode}"])
    return subprocess.run(
        ["docker", "run", "--rm", *vol_flags, image, *args],
        capture_output=True,
        text=True,
    )


def format_size(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f}{unit}" if unit != "B" else f"{n}{unit}"
        n /= 1024
    return f"{n:.1f}TB"


def benchmark_file(ifc_path):
    ifc_path = Path(ifc_path).resolve()
    basename = ifc_path.stem
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"=== {basename} ({format_size(ifc_path.stat().st_size)}) ===")

    # ifc-lite-headless
    out_glb = OUTPUT_DIR / f"{basename}_ifc-lite.glb"
    t0 = time.time()
    result = run_container(
        "ifc-lite-headless",
        ["glb", "/input.ifc", "-o", f"/output/{out_glb.name}"],
        [(str(ifc_path), "/input.ifc", "ro"), (str(OUTPUT_DIR), "/output", "rw")],
    )
    t1 = time.time()
    ifc_lite_time = t1 - t0
    ifc_lite_size = out_glb.stat().st_size if out_glb.exists() else 0
    status = "OK" if result.returncode == 0 else "FAIL"
    print(f"  ifc-lite-headless: {ifc_lite_time:.2f}s  {format_size(ifc_lite_size)}  [{status}]")
    if result.returncode != 0:
        print(f"    stderr: {result.stderr.strip()[:200]}")

    # IfcConvert
    out_glb2 = OUTPUT_DIR / f"{basename}_ifcconvert.glb"
    t0 = time.time()
    result = run_container(
        "ifcconvert",
        ["/input.ifc", f"/output/{out_glb2.name}"],
        [(str(ifc_path), "/input.ifc", "ro"), (str(OUTPUT_DIR), "/output", "rw")],
    )
    t1 = time.time()
    ifcconv_time = t1 - t0
    ifcconv_size = out_glb2.stat().st_size if out_glb2.exists() else 0
    status = "OK" if result.returncode == 0 else "FAIL"
    print(f"  IfcConvert 0.8.4:  {ifcconv_time:.2f}s  {format_size(ifcconv_size)}  [{status}]")
    if result.returncode != 0:
        print(f"    stderr: {result.stderr.strip()[:200]}")

    if ifc_lite_time > 0 and ifcconv_time > 0:
        ratio = ifcconv_time / ifc_lite_time
        print(f"  Speedup: {ratio:.1f}x")
    print()


def main():
    os.chdir(Path(__file__).resolve().parent.parent)
    build_containers()

    if len(sys.argv) > 1:
        files = [Path(f) for f in sys.argv[1:]]
    else:
        files = sorted(Path(".private").glob("*.ifc"))

    if not files:
        print("No IFC files found. Pass file paths or place them in .private/")
        sys.exit(1)

    for f in files:
        benchmark_file(f)


if __name__ == "__main__":
    main()
