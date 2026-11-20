"""Parse a Source .nav (v16) into areas; prints JSON stats. Usage: navparse.py file.nav [out.json]"""
import json, struct, sys


class R:
    def __init__(self, b): self.b, self.o = b, 0
    def u(self, fmt):
        v = struct.unpack_from("<" + fmt, self.b, self.o); self.o += struct.calcsize("<" + fmt); return v
    def one(self, fmt): return self.u(fmt)[0]


def parse(path):
    r = R(open(path, "rb").read())
    magic, version = r.u("II")
    assert magic == 0xFEEDFACE and version == 16, (hex(magic), version)
    sub, bsp = r.u("II")
    analyzed = r.one("B")
    for _ in range(r.one("H")):
        n = r.one("H"); r.o += n
    r.one("B")  # has unnamed areas
    areas = []
    for _ in range(r.one("I")):
        aid, attr = r.u("II")
        nw = r.u("3f"); se = r.u("3f"); nez, swz = r.u("2f")
        conns = 0
        for _ in range(4):
            n = r.one("I"); r.o += 4 * n; conns += n
        for _ in range(r.one("B")):
            r.o += 4 + 12 + 1
        for _ in range(r.one("I")):
            r.o += 4 + 1 + 4 + 1
            n = r.one("B"); r.o += 5 * n
        r.one("H")
        for _ in range(2):
            n = r.one("I"); r.o += 4 * n
        r.o += 8 + 16
        n = r.one("I"); r.o += 5 * n
        r.one("I")
        areas.append({"id": aid, "attr": attr, "nw": nw, "se": se, "conns": conns})
    return {"bspSize": bsp, "analyzed": analyzed, "areas": areas}


if __name__ == "__main__":
    d = parse(sys.argv[1])
    a = d["areas"]
    walk = sum((x["se"][0] - x["nw"][0]) * (x["se"][1] - x["nw"][1]) for x in a)
    stats = {
        "areas": len(a), "connections": sum(x["conns"] for x in a), "analyzed": d["analyzed"],
        "walkableUnits2": walk,
        "minX": min(x["nw"][0] for x in a), "maxX": max(x["se"][0] for x in a),
        "minY": min(x["nw"][1] for x in a), "maxY": max(x["se"][1] for x in a),
        "minZ": min(min(x["nw"][2], x["se"][2]) for x in a), "maxZ": max(max(x["nw"][2], x["se"][2]) for x in a),
    }
    print(json.dumps(stats))
    if len(sys.argv) > 2:
        json.dump(d, open(sys.argv[2], "w"))
