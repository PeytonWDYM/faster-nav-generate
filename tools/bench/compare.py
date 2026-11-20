"""Replay both maps and compare them with saved vanilla meshes (requires Pillow and Shapely).

python tools/bench/compare.py --cli build/Release/navfast_cli.exe --jobs /path/to/navjobs
    --vanilla /path/to/vanilla-maps --baseline /path/to/pre-change-navs
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess

from PIL import Image, ImageDraw
from shapely.geometry import box
from shapely.ops import unary_union

from navparse import parse


MODES = {"vanilla": "#e8a83e", "classic": "#30b9ec", "exact": "#eb63a4"}
MAPS = ("gm_construct", "gm_flatgrass")


def footprint(area):
    return (*area["nw"][:2], *area["se"][:2])


def floor_size(area):
    x1, y1, x2, y2 = footprint(area)
    return (x2 - x1) * (y2 - y1)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(args, job, mode, threads, out):
    report = out.with_suffix(".json")
    subprocess.run([args.cli, "run", job, out, f"--{mode}", "--threads", str(threads),
                    "--json", report], check=True)
    return json.loads(report.read_text())["seconds"]


def clearance(cli, job, areas, samples):
    rng = random.Random(768531)
    rows = []
    for area in rng.choices(areas, weights=[floor_size(a) for a in areas], k=samples):
        u, v = rng.random(), rng.random()
        x1, y1, x2, y2 = footprint(area)
        nz, ez, sz, wz = area["nw"][2], area["nez"], area["se"][2], area["swz"]
        north, south = nz + u * (ez - nz), wz + u * (sz - wz)
        dx = ((ez - nz) * (1 - v) + (sz - wz) * v) / (x2 - x1)
        dy = (south - north) / (y2 - y1)
        norm = math.sqrt(dx * dx + dy * dy + 1)
        p = (x1 + u * (x2 - x1), y1 + v * (y2 - y1), north + v * (south - north),
             -dx / norm, -dy / norm, 1 / norm, area["attr"] & 1)
        rows.append(" ".join(map(str, p)))
    result = subprocess.run([cli, "check-clearance", job], input="\n".join(rows),
                            stdout=subprocess.PIPE, text=True, check=True)
    return json.loads(result.stdout)


def extent(areas):
    bounds = list(map(footprint, areas))
    return (min(b[0] for b in bounds), min(b[1] for b in bounds),
            max(b[2] for b in bounds), max(b[3] for b in bounds))


def panel(meshes, view, side):
    left, top, right, bottom = view
    width, height = right - left, bottom - top
    scale = side / max(width, height)
    size = (round(width * scale) + 1, round(height * scale) + 1)
    image = Image.new("RGBA", size, "#13181d")
    for mode, color in MODES.items():
        layer = Image.new("RGBA", size)
        draw = ImageDraw.Draw(layer)
        for area in meshes[mode]:
            x1, y1, x2, y2 = footprint(area)
            x1, y1, x2, y2 = max(x1, left), max(y1, top), min(x2, right), min(y2, bottom)
            if x2 <= x1 or y2 <= y1:
                continue
            bounds = ((x1 - left) * scale, (y1 - top) * scale,
                      (x2 - left) * scale, (y2 - top) * scale)
            draw.rectangle(bounds, fill=color + "38", outline=color + "a0")
        image = Image.alpha_composite(image, layer)
    return image


def overlay(path, label, meshes):
    view = extent(a for areas in meshes.values() for a in areas)
    detail = extent(meshes["classic"] + meshes["exact"])
    width, height = view[2] - view[0], view[3] - view[1]
    detail_width, detail_height = detail[2] - detail[0], detail[3] - detail[1]
    if max(detail_width, detail_height) < 0.5 * max(width, height):
        margin = 0.05 * max(detail_width, detail_height)
        detail = (detail[0] - margin, detail[1] - margin, detail[2] + margin, detail[3] + margin)
        panels = [("Full map", panel(meshes, view, 580)), ("Generated footprint", panel(meshes, detail, 580))]
    else:
        panels = [("", panel(meshes, view, 1200))]
    size = (max(350, sum(p.width + 16 for _, p in panels) + 16), max(p.height for _, p in panels) + 96)
    image = Image.new("RGBA", size, "#13181d")
    draw = ImageDraw.Draw(image)
    draw.text((16, 12), label, fill="#f5f7f8")
    for i, (mode, color) in enumerate(MODES.items()):
        x = 16 + 105 * i
        draw.rectangle((x, 35, x + 9, 44), fill=color)
        draw.text((x + 15, 33), mode.title(), fill="#f5f7f8")
    x = 16
    for title, layer in panels:
        if title:
            draw.text((x, 60), title, fill="#f5f7f8")
        image.paste(layer, (x, 80))
        x += layer.width + 16
    image.convert("RGB").save(path)


def compare(args, map_name, vanilla_time):
    job = args.jobs / f"{map_name}.navjob"
    files = {mode: args.out / f"{map_name}-{mode}.nav" for mode in MODES}
    files["vanilla"] = args.vanilla / f"{map_name}.nav"
    times = {"vanilla": vanilla_time}
    for mode in ("classic", "exact"):
        times[mode] = replay(args, job, mode, args.threads, files[mode])
    repeat = args.out / f"{map_name}-exact-{args.check_threads}t.nav"
    replay(args, job, "exact", args.check_threads, repeat)

    meshes = {mode: parse(file)["areas"] for mode, file in files.items()}
    union = {mode: unary_union([box(*footprint(a)) for a in areas]) for mode, areas in meshes.items()}
    vanilla = union["vanilla"]
    rows = {
        mode: {"areas": len(areas), "connections": sum(a["conns"] for a in areas),
               "seconds": times[mode], "floorUnits2": sum(map(floor_size, areas)),
               "projectedUnits2": union[mode].area,
               "vanillaIoU": union[mode].intersection(vanilla).area / union[mode].union(vanilla).area,
               "sha256": sha256(files[mode])}
        for mode, areas in meshes.items()
    }
    rows["exact"]["clearance"] = clearance(args.cli, job, meshes["exact"], args.samples)
    rows["exact"]["byteIdenticalAcrossThreads"] = sha256(repeat) == rows["exact"]["sha256"]
    if args.baseline:
        baseline = sha256(args.baseline / f"{map_name}.nav")
        rows["classic"]["byteIdenticalToBaseline"] = baseline == rows["classic"]["sha256"]
    overlay(args.out / f"{map_name}.png", map_name, meshes)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("cli", "jobs", "vanilla"):
        parser.add_argument(f"--{flag}", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--out", type=Path, default=Path("artifacts/exact-nav"))
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--check-threads", type=int, default=1)
    parser.add_argument("--samples", type=int, default=10000)
    for map_name in MAPS:
        parser.add_argument(f"--{map_name}-vanilla-seconds", type=float)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    result = {m: compare(args, m, getattr(args, f"{m}_vanilla_seconds")) for m in MAPS}
    (args.out / "comparison.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    if any(not rows["exact"]["byteIdenticalAcrossThreads"] or
           not rows["classic"].get("byteIdenticalToBaseline", True) for rows in result.values()):
        raise SystemExit("Replay byte comparison failed; see comparison.json")


if __name__ == "__main__":
    main()
