"""Render the README benchmark charts as static light/dark SVGs.

Usage: python tools/bench/render.py artifacts/bench docs/bench
The bench dir (local, gitignored) holds:
  navfast.json          our mesh, from `python tools/bench/navparse.py <map>.nav navfast.json`
  navfast-ingame.json   in-game nav_generate_fast runs ({"runs": [{wall, stats}]})
  scale-<t>-<rep>.json  `navfast_cli run <job> <out> --threads <t> --json` reports, reps 1-3
Vanilla's time is the "Generation complete! N seconds elapsed." line from the server console.
"""
import json, statistics, sys
from pathlib import Path

bench, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)

UNIT_M = 0.01905  # Source player scale: 16 units = 1 ft
VANILLA_S = 154.3  # "Generation complete! 154.3 seconds elapsed." (default settings)
CPU = "i7-14700KF (28 threads)"

THEMES = {
    "light": dict(surface="#fcfcfb", text="#0b0b0b", text2="#52514e", muted="#8a8984", grid="#e4e3df",
                  fast="#2a78d6", vanilla="#eb6834",
                  cats=["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"],
                  ramp=["#b7d3f6", "#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281"]),
    "dark": dict(surface="#1a1a19", text="#f0efec", text2="#c3c2b7", muted="#8f8e86", grid="#2f2f2c",
                 fast="#3987e5", vanilla="#d95926",
                 cats=["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181"],
                 ramp=["#104281", "#184f95", "#256abf", "#3987e5", "#6da7ec", "#9ec5f4"]),
}
FONT = "font-family=\"-apple-system,BlinkMacSystemFont,'Segoe UI',Helvetica,Arial,sans-serif\""


def svg(w, h, t, body, title):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" {FONT} '
            f'role="img" aria-label="{title}"><title>{title}</title>'
            f'<rect width="{w}" height="{h}" rx="12" fill="{t["surface"]}"/>{body}</svg>')


def text(x, y, s, fill, size=13, weight=400, anchor="start"):
    return f'<text x="{x}" y="{y}" fill="{fill}" font-size="{size}" font-weight="{weight}" text-anchor="{anchor}">{s}</text>'


def bar(x, y, w, h, fill):
    # Rounded data end, square baseline end (4px radius per mark spec).
    w = max(w, 4)
    return (f'<path d="M{x},{y} H{x + w - 4} Q{x + w},{y} {x + w},{y + 4} V{y + h - 4} '
            f'Q{x + w},{y + h} {x + w - 4},{y + h} H{x} Z" fill="{fill}"/>')


# ---------- data ----------
mesh = json.load(open(bench / "navfast.json"))
areas = mesh["areas"]
ingame = json.load(open(bench / "navfast-ingame.json"))
good = [r for r in ingame["runs"] if r["wall"] > 0.5]  # completed runs (others were retry artifacts)
fast_wall = statistics.median(r["wall"] for r in good)
fast_job = statistics.median(r["stats"]["seconds"] for r in good)
phase = good[0]["stats"]["phaseSeconds"]
stats = good[0]["stats"]
threads = [1, 2, 4, 8, 16, 28]
scaling = {t: statistics.median(json.load(open(bench / f"scale-{t}-{r}.json"))["seconds"] for r in (1, 2, 3))
           for t in threads}

minx = min(a["nw"][0] for a in areas); maxx = max(a["se"][0] for a in areas)
miny = min(a["nw"][1] for a in areas); maxy = max(a["se"][1] for a in areas)
minz = min(min(a["nw"][2], a["se"][2]) for a in areas); maxz = max(max(a["nw"][2], a["se"][2]) for a in areas)
walk_m2 = sum((a["se"][0] - a["nw"][0]) * (a["se"][1] - a["nw"][1]) for a in areas) * UNIT_M ** 2
size_m = ((maxx - minx) * UNIT_M, (maxy - miny) * UNIT_M)
height_m = (maxz - minz) * UNIT_M
bsp_mb = mesh["bspSize"] / 1e6


def speed_chart(t):
    w, h, left, right = 760, 196, 190, 190
    plot = w - left - right
    scale = plot / 160
    body = text(24, 34, "Time to generate the nav mesh", t["text"], 16, 600)
    body += text(24, 54, f"Same map, same server · {CPU}", t["text2"], 12)
    for s in range(0, 161, 40):
        x = left + s * scale
        body += f'<line x1="{x}" y1="70" x2="{x}" y2="160" stroke="{t["grid"]}"/>'
        body += text(x, 178, f"{s} s", t["muted"], 11, anchor="middle")
    rows = [("nav_generate (default)", VANILLA_S, t["vanilla"]), ("nav_generate_fast", fast_wall, t["fast"])]
    for i, (label, v, color) in enumerate(rows):
        y = 82 + i * 40
        body += text(left - 12, y + 16, label, t["text"], 13, 500, "end")
        body += bar(left, y, v * scale, 24, color)
        body += text(left + max(v * scale, 4) + 8, y + 17, f"{v:.1f} s", t["text"], 13, 600)
    body += text(w - 24, 112, f"{VANILLA_S / fast_wall:.0f}×", t["text"], 34, 700, "end")
    body += text(w - 24, 132, "faster", t["text2"], 13, 400, "end")
    return svg(w, h, t, body, f"nav_generate {VANILLA_S:.1f} s vs nav_generate_fast {fast_wall:.1f} s")


def phases_chart(t):
    w, h, left = 760, 170, 24
    plot = w - 48
    segs = [
        ("Sampling (all cores)", phase["SampleWalkableSpace"]),
        ("Merging areas", phase["MergeGeneratedAreas"]),
        ("Building areas", phase["CreateAreasFromSamples"]),
        ("Everything else", fast_job - phase["SampleWalkableSpace"] - phase["MergeGeneratedAreas"] - phase["CreateAreasFromSamples"]),
        ("Lua: read map + load mesh", fast_wall - fast_job),
    ]
    total = sum(v for _, v in segs)
    body = text(24, 34, f"Where nav_generate_fast's {fast_wall:.1f} s goes", t["text"], 16, 600)
    body += text(24, 54, "In-game run on the same server", t["text2"], 12)
    x = left
    for i, (label, v) in enumerate(segs):
        sw = v / total * plot
        body += f'<rect x="{x}" y="72" width="{max(sw - 2, 1)}" height="28" rx="3" fill="{t["cats"][i]}"/>'
        x += sw
    # Legend with values (identity never color-alone).
    lx, ly = left, 128
    for i, (label, v) in enumerate(segs):
        item = f"{label} {v:.2f} s"
        body += f'<rect x="{lx}" y="{ly - 10}" width="10" height="10" rx="2" fill="{t["cats"][i]}"/>'
        body += text(lx + 16, ly, item, t["text2"], 12)
        lx += 16 + len(item) * 6.6 + 18
        if lx > w - 180:
            lx, ly = left, ly + 22
    return svg(w, h, t, body, "Phase breakdown of nav_generate_fast")


def scaling_chart(t):
    w, h, left, top, bottom = 760, 280, 70, 72, 236
    plot_w = w - left - 60
    ymax = 3.5
    xs = {th: left + i * plot_w / (len(threads) - 1) for i, th in enumerate(threads)}
    y = lambda s: bottom - s / ymax * (bottom - top)
    body = text(24, 34, "More cores, less waiting", t["text"], 16, 600)
    body += text(24, 54, "Generation time by worker threads · offline replay of the same map, median of 3", t["text2"], 12)
    for s in (0, 1, 2, 3):
        body += f'<line x1="{left}" y1="{y(s)}" x2="{w - 40}" y2="{y(s)}" stroke="{t["grid"]}"/>'
        body += text(left - 10, y(s) + 4, f"{s} s", t["muted"], 11, anchor="end")
    for th in threads:
        body += text(xs[th], bottom + 20, str(th), t["muted"], 11, anchor="middle")
    body += text((left + w - 40) / 2, bottom + 38, "threads", t["muted"], 11, anchor="middle")
    pts = " ".join(f"{xs[th]:.1f},{y(scaling[th]):.1f}" for th in threads)
    body += f'<polyline points="{pts}" fill="none" stroke="{t["fast"]}" stroke-width="2" stroke-linejoin="round"/>'
    for th in threads:
        body += f'<circle cx="{xs[th]}" cy="{y(scaling[th])}" r="4.5" fill="{t["fast"]}" stroke="{t["surface"]}" stroke-width="2"/>'
    for th, anchor, dx in ((1, "start", 10), (28, "end", -10)):
        body += text(xs[th] + dx, y(scaling[th]) - 12, f"{scaling[th]:.2f} s", t["text"], 12, 600, anchor)
    return svg(w, h, t, body, "Generation time by thread count")


def map_chart(t):
    w, h, pad, side = 760, 480, 24, 400
    sx = side / (maxx - minx)
    ox, oy = pad, 72
    body = text(24, 34, "The test map", t["text"], 16, 600)
    body += text(24, 54, "Top-down view of the generated nav mesh, colored by height", t["text2"], 12)
    ramp = t["ramp"]
    for a in sorted(areas, key=lambda a: a["nw"][2]):
        z = (a["nw"][2] + a["se"][2]) / 2
        c = ramp[min(len(ramp) - 1, int((z - minz) / (maxz - minz + 1e-6) * len(ramp)))]
        x = ox + (a["nw"][0] - minx) * sx
        yy = oy + (a["nw"][1] - miny) * sx
        body += f'<rect x="{x:.1f}" y="{yy:.1f}" width="{max((a["se"][0] - a["nw"][0]) * sx, 0.6):.1f}" height="{max((a["se"][1] - a["nw"][1]) * sx, 0.6):.1f}" fill="{c}"/>'
    # Scale bar: 20 m.
    bar_px = 20 / UNIT_M * sx
    by = oy + side + 18
    body += f'<line x1="{ox}" y1="{by}" x2="{ox + bar_px}" y2="{by}" stroke="{t["text"]}" stroke-width="2"/>'
    body += f'<line x1="{ox}" y1="{by - 4}" x2="{ox}" y2="{by + 4}" stroke="{t["text"]}" stroke-width="2"/>'
    body += f'<line x1="{ox + bar_px}" y1="{by - 4}" x2="{ox + bar_px}" y2="{by + 4}" stroke="{t["text"]}" stroke-width="2"/>'
    body += text(ox + bar_px + 8, by + 4, "20 m", t["text2"], 11)
    # Height legend.
    lx = ox + side - len(ramp) * 18
    for i, c in enumerate(ramp):
        body += f'<rect x="{lx + i * 18}" y="{by - 5}" width="16" height="10" rx="2" fill="{c}"/>'
    body += text(lx - 6, by + 4, "low", t["muted"], 11, anchor="end")
    body += text(lx + len(ramp) * 18 + 4, by + 4, "high", t["muted"], 11)
    # Stat tiles.
    tiles = [
        (f"{size_m[0]:.0f} × {size_m[1]:.0f} m", "footprint"),
        (f"{walk_m2:,.0f} m²", "walkable floor"),
        (f"{height_m:.0f} m", "from lowest to highest floor"),
        (f"{bsp_mb:.0f} MB", "map file"),
        (f"{stats['nodes']:,}", "sampled points"),
        (f"{stats['areas']:,} / {stats['connections']:,}", "areas / connections"),
    ]
    tx = ox + side + 40
    for i, (value, label) in enumerate(tiles):
        ty = oy + 18 + i * 64
        body += text(tx, ty + 14, value, t["text"], 22, 700)
        body += text(tx, ty + 34, label, t["text2"], 12)
    return svg(w, h, t, body, "Top-down view of the test map's nav mesh with size stats")


charts = {"speed": speed_chart, "phases": phases_chart, "scaling": scaling_chart, "map": map_chart}
for name, fn in charts.items():
    for mode, theme in THEMES.items():
        (out / f"{name}-{mode}.svg").write_text(fn(theme), encoding="utf-8")

print(json.dumps(dict(fast_wall=fast_wall, fast_job=fast_job, speedup=VANILLA_S / fast_wall, size_m=size_m,
                      walk_m2=walk_m2, height_m=height_m, bsp_mb=bsp_mb, scaling=scaling)))
