#!/usr/bin/env python3
"""Inject dimensions.json, the printed-part STLs and the vendor meshes into the viewer template
-> docs/assembly-3d.html.

The Three.js viewer and the OpenSCAD model both read dimensions.json, and the
viewer shows the *actual* exported STLs for every printed part, so the render
on the web page and the files on the printer cannot disagree.

STLs are packed as raw little-endian float32 triangle soup (x y z per vertex,
three vertices per facet), base64-encoded, keyed by part name.  Normals are
recomputed in the browser.
"""
import base64
import glob
import json
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "docs", "assembly-3d.template.html")
OUT = os.path.join(ROOT, "docs", "assembly-3d.html")
STL_DIRS = [os.path.join(HERE, "stl"), os.path.join(HERE, "mesh")]


def read_stl(path):
    """Return a flat list of floats (9 per facet) from an ASCII or binary STL."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        out = []
        for line in data.decode("ascii", "replace").splitlines():
            line = line.strip()
            if line.startswith("vertex"):
                out.extend(float(v) for v in line.split()[1:4])
        return out
    n = struct.unpack_from("<I", data, 80)[0]
    out = []
    off = 84
    for _ in range(n):
        vals = struct.unpack_from("<12f", data, off)
        out.extend(vals[3:12])
        off += 50
    return out


def pack(floats):
    """Weld the triangle soup, index it, and quantise positions to uint16.

    A 16-bit position over a 100 mm part is 0.0015 mm, far finer than anything
    here, and it cuts the payload roughly fourfold against raw float32 soup.
    The viewer expands it back to non-indexed so hard edges stay flat-shaded.
    """
    import numpy as np

    v = np.asarray(floats, dtype=np.float64).reshape(-1, 3)
    key = np.round(v * 10000.0).astype(np.int64)
    _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    verts = v[idx]
    faces = inv.reshape(-1, 3)

    lo = verts.min(axis=0)
    span = np.maximum(verts.max(axis=0) - lo, 1e-9)
    q = np.round((verts - lo) / span * 65535.0).astype(np.uint16)
    wide = len(verts) > 65536
    ind = faces.astype(np.uint32 if wide else np.uint16)
    return {
        "p": base64.b64encode(q.tobytes()).decode("ascii"),
        "i": base64.b64encode(ind.tobytes()).decode("ascii"),
        "w": 1 if wide else 0,
        "o": [round(float(x), 4) for x in lo],
        "s": [round(float(x), 6) for x in span],
    }


with open(os.path.join(HERE, "dimensions.json")) as f:
    dims = json.load(f)
payload = json.dumps(dims, separators=(",", ":"))

meshes, tris = {}, {}
for d in STL_DIRS:
    for path in sorted(glob.glob(os.path.join(d, "*.stl"))):
        name = os.path.splitext(os.path.basename(path))[0]
        floats = read_stl(path)
        meshes[name] = pack(floats)
        tris[name] = len(floats) // 9
mesh_payload = json.dumps(meshes, separators=(",", ":"))

with open(SRC) as f:
    html = f.read()
for marker in ("/*__DIMS__*/", "/*__MESHES__*/"):
    assert marker in html, f"template is missing the {marker} marker"
html = html.replace("/*__DIMS__*/", payload, 1).replace("/*__MESHES__*/", mesh_payload, 1)
with open(OUT, "w") as f:
    f.write(html)
print(f"wrote {OUT} ({len(html)//1024} kB: dims {len(payload)//1024} kB, meshes {len(mesh_payload)//1024} kB)")
print("  " + ", ".join(f"{k} {v} tris" for k, v in tris.items()))
