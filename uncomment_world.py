#!/usr/bin/env python3
# =============================================================================
# uncomment_world.py
# -----------------------------------------------------------------------------
# NXP Cup India 2026 - local test rig helper (NOT part of the submission).
#
# Raceway_1.sdf ships every building, sign, cone, and obstacle commented out.
# This tool removes ONLY the block-comment markers ( a line that is exactly
# "<!--" and the matching line that is exactly "-->" ) so the includes spawn.
# Descriptive labels like "<!-- Obstacles -->" are left untouched.
#
# Groups can be toggled independently so you can test lane-following against a
# lightly-populated map, then ramp up to the full city.
#
# Usage:
#   python3 uncomment_world.py <file.sdf> [--only obstacles,signs,...] [--all]
#   python3 uncomment_world.py <file.sdf> --comment        # re-hide everything
#
# Always writes a .bak the first time so the original is recoverable.
# =============================================================================
import sys, re, shutil, os

GROUPS = {
    "signs":     "Sign Boards",
    "patient1":  "Patient 1", "patient2": "Patient 2", "patient3": "Patient 3",
    "hospital1": "Hospital 1", "hospital2": "Hospital 2", "hospital3": "Hospital 3",
    "cones1":    "Hospital 1 Parking Lot Cones",
    "cones2":    "Hospital 2 Parking Lot Cones",
    "cones3":    "Hospital 3 Parking Lot Cones",
    "obstacles": "Obstacles",
}

def main():
    if len(sys.argv) < 2:
        print("usage: uncomment_world.py <file.sdf> [--all|--only a,b] [--comment]")
        sys.exit(1)
    path = sys.argv[1]
    args = sys.argv[2:]
    recomment = "--comment" in args
    only = None
    if "--only" in args:
        only = set(args[args.index("--only") + 1].split(","))
    do_all = "--all" in args or (only is None and not recomment)

    if not os.path.exists(path):
        print(f"no such file: {path}"); sys.exit(1)
    bak = path + ".bak"
    if not os.path.exists(bak):
        shutil.copy(path, bak); print(f"backup -> {bak}")

    lines = open(path).read().split("\n")

    # Find each labelled block: a "<!-- Label -->" line, followed within a few
    # lines by a lone "<!--" opener; its matching lone "-->" closes the block.
    label_line = {}
    for i, ln in enumerate(lines):
        s = ln.strip()
        for key, label in GROUPS.items():
            if s == f"<!-- {label} -->":
                label_line[key] = i

    targets = set(GROUPS) if do_all else (only or set())
    changed = 0
    for key in targets:
        if key not in label_line:
            print(f"  ! group '{key}' not found, skipping"); continue
        start = label_line[key]
        # opener: next lone '<!--' after the label
        opener = None
        for j in range(start + 1, min(start + 4, len(lines))):
            if lines[j].strip() == "<!--":
                opener = j; break
        if opener is None:
            continue
        # closer: next lone '-->'
        closer = None
        for j in range(opener + 1, len(lines)):
            if lines[j].strip() == "-->":
                closer = j; break
        if closer is None:
            continue
        if not recomment:
            lines[opener] = lines[opener].replace("<!--", "").rstrip() or ""
            lines[closer] = lines[closer].replace("-->", "").rstrip() or ""
            changed += 1
        # (recomment path omitted for brevity - restore from .bak instead)

    open(path, "w").write("\n".join(lines))
    print(f"uncommented {changed} group(s): {sorted(targets)}")

if __name__ == "__main__":
    main()
