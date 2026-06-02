#!/usr/bin/env python3
"""
osm_to_tiles.py — OSM PBF → JPEG Tiles
Chunked rendering + spatial index + blank skipping (multi-province merge safe)

Features:
  - Spatial index: chunk only processes nearby features (~100x speedup)
  - Chunked rendering: 16x16 tiles/chunk, 4096px canvas
  - Blank skipping: empty tiles not saved (multi-province merge safe)
  - Auto-detect .osm.pbf files in the same directory

Usage:
  python osm_to_tiles.py input.osm.pbf -z 12-13 -b S,W,N,E -q 75
  python osm_to_tiles.py -z 12-13                        (auto-detect .osm.pbf)
  python osm_to_tiles.py -z 12-13 -b [S,W,N,E]
"""

import argparse
import glob
import math
import os
import sys
import time
from collections import defaultdict

try:
    import osmium
    from PIL import Image, ImageDraw, ImageFont
except ImportError as e:
    print(f"Missing dependencies: {e}\nInstall: pip install osmium Pillow")
    sys.exit(1)

# ═══════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════

TILE     = 256
CHUNK    = 16
CELL_DEG = 0.01

DEFAULT_OUTPUT = "gpsmap"

BG       = '#f2efe9'
WATER    = '#aad3df'
WATER_LN = '#6b97ab'
BLD_FILL = '#d9d0c9'
BLD_EDGE = '#b8a89d'

LANDUSE_COLORS = {
    'forest': '#add19e', 'grass': '#c8facc', 'meadow': '#c8facc',
    'park': '#c8facc', 'garden': '#c8facc', 'cemetery': '#b5d2a3',
    'farmland': '#eef0d5', 'farmyard': '#f5e6c8',
    'industrial': '#ebdbe8', 'commercial': '#f2dad9',
    'retail': '#f2dad9', 'residential': '#e0dfdf',
}
LEISURE_COLORS = {
    'park': '#c8facc', 'garden': '#c8facc',
    'pitch': '#aae0a0', 'golf_course': '#b5d2a3',
}

ROAD_STYLES = {
    'motorway':      ('#e892a2', 7, 10),
    'trunk':         ('#f9b29c', 6, 10),
    'primary':       ('#fcd6a4', 5, 12),
    'secondary':     ('#f7fabf', 4, 13),
    'tertiary':      ('#ffffff', 3, 14),
    'residential':   ('#ffffff', 2, 15),
    'unclassified':  ('#ffffff', 2, 15),
    'service':       ('#dddddd', 1, 16),
    'living_street': ('#ededed', 2, 15),
    'pedestrian':    ('#dddde8', 2, 15),
    'footway':       ('#f9929c', 1, 16),
    'cycleway':      ('#6bc4e8', 1, 16),
    'path':          ('#f9929c', 1, 16),
    'track':         ('#c8b688', 1, 15),
    'steps':         ('#f9929c', 1, 16),
}

ROAD_ORDER = [
    'steps', 'footway', 'path', 'cycleway', 'track',
    'service', 'living_street', 'pedestrian', 'residential',
    'unclassified', 'tertiary', 'secondary', 'primary',
    'trunk', 'motorway',
]


# ═══════════════════════════════════════════
#  Output functions (fix PowerShell buffering)
# ═══════════════════════════════════════════

def log(msg=""):
    print(msg, flush=True)


# ═══════════════════════════════════════════
#  Startup checks
# ═══════════════════════════════════════════

def detect_pbf_files():
    """Scan current directory for .osm.pbf files"""
    files = glob.glob("*.osm.pbf")
    files.sort(key=lambda f: os.path.getsize(f), reverse=True)
    return files


def check_output_dir(out_dir):
    """Check output directory, warn if tiles already exist"""
    if not os.path.isdir(out_dir):
        return

    has_tiles = False
    for z_dir in glob.glob(os.path.join(out_dir, "*")):
        if os.path.isdir(z_dir) and os.path.basename(z_dir).isdigit():
            has_tiles = True
            break

    if has_tiles:
        log()
        log("=" * 60)
        log(f"  Note: Output directory {out_dir}/ already contains tile files")
        log()
        log("  If you have multiple .osm.pbf files to merge-render,")
        log("  please merge with osmium first to avoid boundary tile issues:")
        log()
        log("    osmium merge file1.osm.pbf file2.osm.pbf -o merged.osm.pbf")
        log()
        log("  If rendering a single file, you can ignore this notice.")
        log("=" * 60)
        log()


def resolve_input_file(user_input):
    """Determine input file: user-specified > auto-detect"""
    if user_input:
        if not os.path.exists(user_input):
            log(f"Error: File not found: {user_input}")
            sys.exit(1)
        return user_input

    pbf_files = detect_pbf_files()

    if len(pbf_files) == 0:
        log()
        log("Error: No .osm.pbf files found in current directory")
        log()
        log("Please place a .osm.pbf file in the current directory, or specify a file path:")
        log("  python osm_to_tiles.py yourfile.osm.pbf -z 12-13")
        log()
        log("Data download: https://download.geofabrik.de/asia/china.html")
        sys.exit(1)

    if len(pbf_files) == 1:
        log(f"Auto-detected: {pbf_files[0]}")
        return pbf_files[0]

    # Multiple files
    log()
    log("=" * 60)
    log(f"  Detected {len(pbf_files)} .osm.pbf files:")
    log()
    for i, f in enumerate(pbf_files, 1):
        size_mb = os.path.getsize(f) / 1048576
        log(f"    {i}. {f}  ({size_mb:.1f} MB)")
    log()
    log("  Rendering multiple files directly will cause boundary tile data loss.")
    log("  It is recommended to merge first then render:")
    log()

    all_files = " ".join(pbf_files)
    log(f"    osmium merge {all_files} -o merged.osm.pbf")
    log()
    log("  Then run:")
    log("    python osm_to_tiles.py merged.osm.pbf -z 12-13")
    log("=" * 60)
    log()
    sys.exit(0)


# ═══════════════════════════════════════════
#  Coordinate utilities
# ═══════════════════════════════════════════

def deg2px(lat, lon, z):
    n = 2.0 ** z * TILE
    px = (lon + 180.0) / 360.0 * n
    py = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return px, py


def deg2tile(lat, lon, z):
    n = 2.0 ** z
    return (int((lon + 180.0) / 360.0 * n),
            int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n))


def tile2lat(y, z):
    return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / 2.0 ** z))))


def tile2lon(x, z):
    return x / (2.0 ** z) * 360.0 - 180.0


def bbox_intersect(a, b):
    return not (a[0] > b[2] or a[2] < b[0] or a[1] > b[3] or a[3] < b[1])


def compute_bbox(nids, nodes):
    la_min, la_max = 90.0, -90.0
    lo_min, lo_max = 180.0, -180.0
    found = False
    for nid in nids:
        if nid in nodes:
            la, lo = nodes[nid]
            if la < la_min: la_min = la
            if la > la_max: la_max = la
            if lo < lo_min: lo_min = lo
            if lo > lo_max: lo_max = lo
            found = True
    return (la_min, lo_min, la_max, lo_max) if found else None


# ═══════════════════════════════════════════
#  Spatial index
# ═══════════════════════════════════════════

class SpatialIndex:
    def __init__(self, cell_deg=CELL_DEG):
        self.cd = cell_deg
        self.cells = {}

    def _put(self, key, idx):
        if key in self.cells:
            self.cells[key].append(idx)
        else:
            self.cells[key] = [idx]

    def insert(self, bbox, idx):
        la0, lo0, la1, lo1 = bbox
        r0 = math.floor(la0 / self.cd)
        r1 = math.floor(la1 / self.cd)
        c0 = math.floor(lo0 / self.cd)
        c1 = math.floor(lo1 / self.cd)
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                self._put((r, c), idx)

    def insert_point(self, lat, lon, idx):
        self._put((math.floor(lat / self.cd),
                   math.floor(lon / self.cd)), idx)

    def query(self, bbox):
        la0, lo0, la1, lo1 = bbox
        r0 = math.floor(la0 / self.cd)
        r1 = math.floor(la1 / self.cd)
        c0 = math.floor(lo0 / self.cd)
        c1 = math.floor(lo1 / self.cd)
        hits = set()
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                key = (r, c)
                if key in self.cells:
                    hits.update(self.cells[key])
        return hits


# ═══════════════════════════════════════════
#  Fonts
# ═══════════════════════════════════════════

_font_cache = {}


def get_font(size):
    if size not in _font_cache:
        for p in [
            "C:/Windows/Fonts/msyh.ttc",
            "C:/Windows/Fonts/simhei.ttf",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
            "/System/Library/Fonts/PingFang.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]:
            if os.path.exists(p):
                try:
                    _font_cache[size] = ImageFont.truetype(p, size)
                    break
                except Exception:
                    continue
        if size not in _font_cache:
            _font_cache[size] = ImageFont.load_default()
    return _font_cache[size]


# ═══════════════════════════════════════════
#  OSM data collection
# ═══════════════════════════════════════════

class DataCollector(osmium.SimpleHandler):

    def __init__(self, bbox=None):
        super().__init__()
        self.bbox = bbox
        self.nodes = {}
        self.data_lat_min, self.data_lat_max = 90.0, -90.0
        self.data_lon_min, self.data_lon_max = 180.0, -180.0
        self.roads = []
        self.buildings = []
        self.water_areas = []
        self.water_lines = []
        self.landuse = []
        self.lbl_road = []
        self.lbl_poi = []
        self.idx = {}

    def _in(self, lat, lon):
        if not self.bbox:
            return True
        return (self.bbox[0] <= lat <= self.bbox[2] and
                self.bbox[1] <= lon <= self.bbox[3])

    def node(self, n):
        if not n.location.valid():
            return
        la, lo = n.location.lat, n.location.lon
        self.nodes[n.id] = (la, lo)
        if la < self.data_lat_min: self.data_lat_min = la
        if la > self.data_lat_max: self.data_lat_max = la
        if lo < self.data_lon_min: self.data_lon_min = lo
        if lo > self.data_lon_max: self.data_lon_max = lo
        name = n.tags.get('name', '')
        if name and self._in(la, lo):
            self.lbl_poi.append((la, lo, name))

    def way(self, w):
        nids = [nd.ref for nd in w.nodes]
        if len(nids) < 2:
            return
        if nids[0] in self.nodes:
            la, lo = self.nodes[nids[0]]
            if not self._in(la, lo):
                return

        bbox = compute_bbox(nids, self.nodes)
        if bbox is None:
            return

        t = dict(w.tags)

        if 'highway' in t:
            hw = t['highway']
            if hw in ROAD_STYLES:
                name = t.get('name', '')
                self.roads.append((nids, bbox, hw, name))
                if name:
                    mid = nids[len(nids) // 2]
                    if mid in self.nodes:
                        la, lo = self.nodes[mid]
                        self.lbl_road.append((la, lo, name))

        elif 'building' in t or 'building:part' in t:
            self.buildings.append((nids, bbox))

        elif ('natural' in t and t['natural'] == 'water') or \
             ('waterway' in t and t['waterway'] in ('riverbank', 'dock')):
            self.water_areas.append((nids, bbox))

        elif 'waterway' in t:
            self.water_lines.append((nids, bbox))

        elif 'landuse' in t and t['landuse'] in LANDUSE_COLORS:
            self.landuse.append((nids, bbox, LANDUSE_COLORS[t['landuse']]))

        elif 'leisure' in t and t['leisure'] in LEISURE_COLORS:
            self.landuse.append((nids, bbox, LEISURE_COLORS[t['leisure']]))

    def build_indices(self):
        for name, src_list, extractor in [
            ('roads',       self.roads,       lambda x: x[1]),
            ('buildings',   self.buildings,    lambda x: x[1]),
            ('water_areas', self.water_areas,  lambda x: x[1]),
            ('water_lines', self.water_lines,  lambda x: x[1]),
            ('landuse',     self.landuse,      lambda x: x[1]),
        ]:
            si = SpatialIndex()
            for i, item in enumerate(src_list):
                si.insert(extractor(item), i)
            self.idx[name] = si

        for name, src_list in [
            ('lbl_road', self.lbl_road),
            ('lbl_poi',  self.lbl_poi),
        ]:
            si = SpatialIndex()
            for i, (la, lo, _) in enumerate(src_list):
                si.insert_point(la, lo, i)
            self.idx[name] = si


# ═══════════════════════════════════════════
#  Tile content tracking
# ═══════════════════════════════════════════

def mark_tiles(content, feat_bbox, ox, oy, tw, th, z):
    la_min, lo_min, la_max, lo_max = feat_bbox
    gpx1, gpy1 = deg2px(la_max, lo_min, z)
    gpx2, gpy2 = deg2px(la_min, lo_max, z)
    tx0 = max(0, math.floor((gpx1 - ox) / TILE))
    ty0 = max(0, math.floor((gpy1 - oy) / TILE))
    tx1 = min(tw - 1, math.floor((gpx2 - ox) / TILE))
    ty1 = min(th - 1, math.floor((gpy2 - oy) / TILE))
    if tx0 > tx1 or ty0 > ty1:
        return
    for tx in range(tx0, tx1 + 1):
        base = tx * th
        for ty in range(ty0, ty1 + 1):
            content[base + ty] = True


def mark_point_tile(content, gpx, gpy, ox, oy, tw, th):
    tx = math.floor((gpx - ox) / TILE)
    ty = math.floor((gpy - oy) / TILE)
    if 0 <= tx < tw and 0 <= ty < th:
        content[tx * th + ty] = True


# ═══════════════════════════════════════════
#  Chunked rendering
# ═══════════════════════════════════════════

def render_chunk(data, tx0, ty0, tw, th, z, out_dir, quality):
    cw, ch = tw * TILE, th * TILE
    ox, oy = float(tx0 * TILE), float(ty0 * TILE)

    c_lat_max = tile2lat(ty0, z)
    c_lat_min = tile2lat(ty0 + th, z)
    c_lon_min = tile2lon(tx0, z)
    c_lon_max = tile2lon(tx0 + tw, z)
    cb = (c_lat_min, c_lon_min, c_lat_max, c_lon_max)

    content = [False] * (tw * th)
    img = Image.new('RGB', (cw, ch), BG)
    dr = ImageDraw.Draw(img)
    nodes = data.nodes
    mc = lambda bb: mark_tiles(content, bb, ox, oy, tw, th, z)

    def to_px(nids):
        pts = []
        for nid in nids:
            if nid in nodes:
                la, lo = nodes[nid]
                gpx, gpy = deg2px(la, lo, z)
                pts.append((gpx - ox, gpy - oy))
        return pts

    # 1. Land use
    for idx in data.idx['landuse'].query(cb):
        nids, bb, color = data.landuse[idx]
        if not bbox_intersect(bb, cb):
            continue
        pts = to_px(nids)
        if len(pts) >= 3:
            try:
                dr.polygon(pts, fill=color)
                mc(bb)
            except Exception:
                pass

    # 2. Water bodies (areas)
    for idx in data.idx['water_areas'].query(cb):
        nids, bb = data.water_areas[idx]
        if not bbox_intersect(bb, cb):
            continue
        pts = to_px(nids)
        if len(pts) >= 3:
            try:
                dr.polygon(pts, fill=WATER)
                mc(bb)
            except Exception:
                pass

    # 3. Waterways (lines)
    for idx in data.idx['water_lines'].query(cb):
        nids, bb = data.water_lines[idx]
        if not bbox_intersect(bb, cb):
            continue
        pts = to_px(nids)
        if len(pts) >= 2:
            try:
                dr.line(pts, fill=WATER_LN, width=max(1, z - 12))
                mc(bb)
            except Exception:
                pass

    # 4. Buildings (z>=15)
    if z >= 15:
        for idx in data.idx['buildings'].query(cb):
            nids, bb = data.buildings[idx]
            if not bbox_intersect(bb, cb):
                continue
            pts = to_px(nids)
            if len(pts) >= 3:
                try:
                    dr.polygon(pts, fill=BLD_FILL, outline=BLD_EDGE)
                    mc(bb)
                except Exception:
                    pass

    # 5. Roads
    road_hits = data.idx['roads'].query(cb)
    by_type = defaultdict(list)
    for idx in road_hits:
        nids, bb, hw, _ = data.roads[idx]
        if bbox_intersect(bb, cb):
            by_type[hw].append((nids, bb))

    for rt in ROAD_ORDER:
        color, base_w, zmin = ROAD_STYLES[rt]
        if z < zmin:
            continue
        w = max(1, base_w + (z - 15))
        for nids, bb in by_type.get(rt, []):
            pts = to_px(nids)
            if len(pts) >= 2:
                drew = False
                try:
                    dr.line(pts, fill=color, width=w, joint='curve')
                    drew = True
                except TypeError:
                    try:
                        dr.line(pts, fill=color, width=w)
                        drew = True
                    except Exception:
                        pass
                if drew:
                    mc(bb)

    # 6. Labels (z>=15)
    if z >= 15:
        font = get_font(max(9, 8 + (z - 15)))

        for idx in data.idx['lbl_road'].query(cb):
            la, lo, txt = data.lbl_road[idx]
            gpx, gpy = deg2px(la, lo, z)
            x, y = gpx - ox, gpy - oy
            if 0 <= x < cw and 0 <= y < ch:
                dr.text((x + 2, y - 7), txt, fill='#555555', font=font)
                mark_point_tile(content, gpx, gpy, ox, oy, tw, th)

        if z >= 16:
            for idx in data.idx['lbl_poi'].query(cb):
                la, lo, txt = data.lbl_poi[idx]
                gpx, gpy = deg2px(la, lo, z)
                x, y = gpx - ox, gpy - oy
                if 0 <= x < cw and 0 <= y < ch:
                    dr.text((x + 2, y - 7), txt, fill='#333333', font=font)
                    mark_point_tile(content, gpx, gpy, ox, oy, tw, th)

    # 7. Save non-empty tiles
    saved, skipped = 0, 0
    for tx_off in range(tw):
        for ty_off in range(th):
            if not content[tx_off * th + ty_off]:
                skipped += 1
                continue
            tx = tx0 + tx_off
            ty = ty0 + ty_off
            tdir = os.path.join(out_dir, str(z), str(tx))
            os.makedirs(tdir, exist_ok=True)
            lx = tx_off * TILE
            ly = ty_off * TILE
            img.crop((lx, ly, lx + TILE, ly + TILE)).save(
                os.path.join(tdir, f"{ty}.jpg"),
                'JPEG', quality=quality
            )
            saved += 1

    del img
    return saved, skipped


# ═══════════════════════════════════════════
#  Main entry point
# ═══════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description='OSM PBF → JPEG Tiles (chunked rendering + spatial index)')
    ap.add_argument('input', nargs='?', default=None,
                    help='.osm.pbf file (auto-detect in current directory if not specified)')
    ap.add_argument('-o', '--output', default=DEFAULT_OUTPUT,
                    help=f'Output directory (default: ./{DEFAULT_OUTPUT})')
    ap.add_argument('-z', '--zoom', default='14-16',
                    help='Zoom levels (e.g. 12-13 or 15)')
    ap.add_argument('-b', '--bbox',
                    help='Bounding box S,W,N,E (e.g. 31.4,110.3,36.4,116.7)')
    ap.add_argument('-q', '--quality', type=int, default=75,
                    help='JPEG quality 1-100 (default: 75)')
    args = ap.parse_args()

    # ── Startup banner ──
    log()
    log("=" * 55)
    log("  OSM Offline Tile Generator")
    log("  Chunked rendering + spatial index + blank skipping")
    log("=" * 55)

    # ── Determine input file ──
    input_file = resolve_input_file(args.input)

    # ── Check output directory ──
    out_dir = args.output
    check_output_dir(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # ── Parameters ──
    z_parts = args.zoom.split('-')
    z1 = int(z_parts[0])
    z2 = int(z_parts[-1])

    bbox = None
    if args.bbox:
        bbox = tuple(map(float, args.bbox.split(',')))
        assert len(bbox) == 4, "bbox format: S,W,N,E"

    log()
    log(f"  Input:   {input_file}")
    log(f"  Output:  {os.path.abspath(out_dir)}/")
    log(f"  Zoom:    z{z1} ~ z{z2}")
    log(f"  Quality: {args.quality}")
    if bbox:
        log(f"  Bounds:  S={bbox[0]} W={bbox[1]} N={bbox[2]} E={bbox[3]}")
    else:
        log(f"  Bounds:  Auto (data extent)")
    log()

    # ── Phase 1: Read data ──
    log(f"[1/3] Reading {input_file}")
    t0 = time.time()
    dc = DataCollector(bbox)
    dc.apply_file(input_file, locations=True)
    dt = time.time() - t0
    log(f"  Elapsed {dt:.1f}s")
    log(f"  Nodes {len(dc.nodes):,}  Roads {len(dc.roads):,}  "
        f"Buildings {len(dc.buildings):,}  "
        f"Water {len(dc.water_areas) + len(dc.water_lines):,}  "
        f"Landuse {len(dc.landuse):,}  "
        f"Labels {len(dc.lbl_road) + len(dc.lbl_poi):,}")

    if not dc.nodes:
        log("Error: No data")
        sys.exit(1)

    # ── Phase 2: Build spatial index ──
    log(f"\n[2/3] Building spatial index")
    t0 = time.time()
    dc.build_indices()
    dt = time.time() - t0
    idx_cells = sum(len(v.cells) for v in dc.idx.values())
    log(f"  Elapsed {dt:.1f}s  ({idx_cells:,} grid cells)")

    # ── Phase 3: Render tiles ──
    log(f"\n[3/3] Rendering zoom {z1}~{z2}  (chunk {CHUNK}x{CHUNK})")
    grand_saved = 0
    grand_skipped = 0
    t_render = time.time()

    for z in range(z1, z2 + 1):
        if bbox:
            xmin, ymax = deg2tile(bbox[0], bbox[1], z)
            xmax, ymin = deg2tile(bbox[2], bbox[3], z)
        else:
            xmin, ymax = deg2tile(dc.data_lat_min, dc.data_lon_min, z)
            xmax, ymin = deg2tile(dc.data_lat_max, dc.data_lon_max, z)

        nx = xmax - xmin + 1
        ny = ymax - ymin + 1

        cx0 = xmin // CHUNK
        cy0 = ymin // CHUNK
        cx1 = xmax // CHUNK
        cy1 = ymax // CHUNK
        n_chunks = (cx1 - cx0 + 1) * (cy1 - cy0 + 1)

        log(f"\n  Zoom {z}: {nx}x{ny} = {nx * ny:,} tiles  ({n_chunks} chunks)")

        saved, skipped = 0, 0
        chunks_done = 0
        ts = time.time()

        for ccx in range(cx0, cx1 + 1):
            for ccy in range(cy0, cy1 + 1):
                txa = max(ccx * CHUNK, xmin)
                tya = max(ccy * CHUNK, ymin)
                txb = min((ccx + 1) * CHUNK - 1, xmax)
                tyb = min((ccy + 1) * CHUNK - 1, ymax)
                tw = txb - txa + 1
                th = tyb - tya + 1
                if tw <= 0 or th <= 0:
                    continue

                s, k = render_chunk(dc, txa, tya, tw, th, z,
                                    out_dir, args.quality)
                saved += s
                skipped += k
                chunks_done += 1

                if chunks_done % 10 == 0 or chunks_done == n_chunks:
                    elapsed = time.time() - ts
                    total = saved + skipped
                    rate = total / elapsed if elapsed > 0 else 0
                    pct = chunks_done * 100 // n_chunks
                    log(f"    [{pct:3d}%] Saved {saved:,}  "
                        f"Skipped {skipped:,}  ({rate:,.0f} t/s)")

        elapsed = time.time() - ts
        total = saved + skipped
        rate = total / elapsed if elapsed > 0 else 0
        log(f"  Zoom {z} done: Saved {saved:,}  Skipped {skipped:,}  "
            f"{elapsed:.1f}s  ({rate:,.0f} t/s)")

        grand_saved += saved
        grand_skipped += skipped

    # ── Summary ──
    total_elapsed = time.time() - t_render
    total_tiles = grand_saved + grand_skipped
    avg_rate = total_tiles / total_elapsed if total_elapsed > 0 else 0

    log(f"\n{'=' * 55}")
    log(f"  Done!")
    log(f"  Saved:   {grand_saved:,} tiles")
    log(f"  Skipped: {grand_skipped:,} (blank)")
    log(f"  Elapsed: {total_elapsed:.1f}s  ({avg_rate:,.0f} tiles/s)")
    log(f"  Output:  {os.path.abspath(out_dir)}/")
    log(f"{'=' * 55}")
    log()


if __name__ == '__main__':
    main()
