#!/usr/bin/env python3
"""
osm_to_tiles.py — OSM PBF → JPEG Tiles
Chunked Rendering + Spatial Indexing + Blank Tile Skipping + Multilingual + Low Zoom Support

Features:
  - Zoom levels z6-z18 (z6 country panorama → z18 most detailed)
  - Spatial indexing for acceleration
  - Chunked rendering 16×16 tiles/chunk
  - Blank tile skipping (Cardputer side fills background color)
  - Interactive parameter input
  - Multilingual place name selection
  - Auto-detection of .osm.pbf files
  - JPEG 4:4:4 no subsampling

Usage:
  python osm_to_tiles.py input.osm.pbf -z 6-14 -l en -b S,W,N,E -q 75
  python osm_to_tiles.py -z 6-14
  python osm_to_tiles.py                                    (fully interactive)
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
    print(f"Missing dependencies: {e}\nInstall with: pip install osmium Pillow")
    sys.exit(1)

# ═══════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════

TILE     = 256
CHUNK    = 16
CELL_DEG = 0.01
DEFAULT_OUTPUT = "gpsmap"
MIN_ZOOM = 6
MAX_ZOOM = 18

# Map background color (must match BG_COLOR on the Cardputer side)
BG = '#f2efe9'

WATER    = '#aad3df'
WATER_LN = '#4a90d9'
BLD_FILL = '#d9d0c9'
BLD_EDGE = '#b8a89d'

COASTLINE_COLOR    = '#2a7fff'
BORDER_COUNTRY_CLR = '#aa4444'
BORDER_PROVINCE_CLR = '#999999'
RIVER_LABEL_CLR    = '#2a5080'
PLACE_TEXT_CLR     = '#333333'

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
    'motorway':      ('#e892a2', 7, 8),
    'trunk':         ('#f9b29c', 6, 9),
    'primary':       ('#fcd6a4', 5, 10),
    'secondary':     ('#f7fabf', 4, 11),
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

# Place types → Min display zoom & font size increment & dot radius
PLACE_TYPES = {'state', 'county', 'city', 'town', 'village', 'suburb', 'hamlet'}
PLACE_ZOOM  = {'state': 5, 'county': 6, 'city': 7,
               'town': 9, 'village': 12, 'suburb': 13, 'hamlet': 14}
PLACE_FNT   = {'state': 3, 'county': 2, 'city': 2,
               'town': 1, 'village': 0, 'suburb': 0, 'hamlet': 0}
PLACE_DOT   = {'state': 4, 'county': 3, 'city': 3,
               'town': 2, 'village': 1, 'suburb': 1, 'hamlet': 1}


# ═══════════════════════════════════════════
#  Output (flush to fix PowerShell)
# ═══════════════════════════════════════════

def log(msg=""):
    print(msg, flush=True)


# ═══════════════════════════════════════════
#  Startup Checks
# ═══════════════════════════════════════════

def detect_pbf_files():
    files = glob.glob("*.osm.pbf")
    files.sort(key=lambda f: os.path.getsize(f), reverse=True)
    return files


def check_output_dir(out_dir):
    if not os.path.isdir(out_dir):
        return
    for z_dir in glob.glob(os.path.join(out_dir, "*")):
        if os.path.isdir(z_dir) and os.path.basename(z_dir).isdigit():
            log()
            log("=" * 60)
            log(f"  Note: Output directory {out_dir}/ already contains tile files")
            log()
            log("  If you have multiple .osm.pbf files to merge, please run first:")
            log("    osmium merge f1.osm.pbf f2.osm.pbf -o merged.osm.pbf")
            log("=" * 60)
            log()
            return


def resolve_input_file(user_input):
    if user_input:
        if not os.path.exists(user_input):
            log(f"Error: File does not exist: {user_input}")
            sys.exit(1)
        return user_input
    pbf = detect_pbf_files()
    if len(pbf) == 0:
        log("\nError: No .osm.pbf files found in the current directory")
        log("Data download: https://download.geofabrik.de/asia/china.html\n")
        sys.exit(1)
    if len(pbf) == 1:
        log(f"Auto-detected: {pbf[0]}")
        return pbf[0]
    log(f"\n  Detected {len(pbf)} .osm.pbf files:")
    for i, f in enumerate(pbf, 1):
        log(f"    {i}. {f}  ({os.path.getsize(f)/1048576:.1f} MB)")
    log("\n  Please merge them first: osmium merge " + " ".join(pbf) + " -o merged.osm.pbf\n")
    sys.exit(0)


# ═══════════════════════════════════════════
#  Interactive Input
# ═══════════════════════════════════════════

def prompt_zoom():
    log(f"\n  Zoom level range: {MIN_ZOOM} ~ {MAX_ZOOM}")
    log(f"    6:  ~625km (Country panorama, coastlines + country borders)")
    log(f"    8:  ~156km (City clusters, main road networks)")
    log(f"    10: ~39km  (Urban areas, highways / national roads)")
    log(f"    12: ~9.7km (Street level)")
    log(f"    15: ~1.1km (Walking navigation)")
    while True:
        raw = input(f"  Start zoom ({MIN_ZOOM}-{MAX_ZOOM}, default {MIN_ZOOM}): ").strip()
        if raw == "":
            z1 = MIN_ZOOM; break
        try:
            z1 = int(raw)
            if MIN_ZOOM <= z1 <= MAX_ZOOM: break
        except ValueError: pass
        log(f"    Please enter a number between {MIN_ZOOM} and {MAX_ZOOM}")
    while True:
        raw = input(f"  End zoom ({z1}-{MAX_ZOOM}, default {z1}): ").strip()
        if raw == "":
            z2 = z1; break
        try:
            z2 = int(raw)
            if z1 <= z2 <= MAX_ZOOM: break
        except ValueError: pass
        log(f"    Please enter a number between {z1} and {MAX_ZOOM}")
    return z1, z2


def prompt_quality():
    while True:
        raw = input("  JPEG quality (1-100, default 75): ").strip()
        if raw == "": return 75
        try:
            q = int(raw)
            if 1 <= q <= 100: return q
        except ValueError: pass
        log("    Please enter a number between 1 and 100")


def prompt_bbox():
    raw = input("  Bounding box (leave blank for auto, format S,W,N,E): ").strip()
    if raw == "": return None
    try:
        p = tuple(map(float, raw.split(',')))
        if len(p) == 4: return p
    except ValueError: pass
    log("    Invalid format, using auto range")
    return None


def prompt_language():
    log()
    log("  Label language (place/road names):")
    log("    Press Enter = Local language (name)")
    log("    en = English    zh = Chinese    ja = Japanese")
    raw = input("  Language code: ").strip().lower()
    if raw == "" or raw == "name":
        return "name"
    return f"name:{raw}"


# ═══════════════════════════════════════════
#  Storage Estimation
# ═══════════════════════════════════════════

def estimate_storage(bbox, z1, z2, quality):
    avg_kb = 15 * (quality / 75.0)
    total = 0
    for z in range(z1, z2 + 1):
        if bbox:
            xmin, ymax = deg2tile(bbox[0], bbox[1], z)
            xmax, ymin = deg2tile(bbox[2], bbox[3], z)
            total += (xmax - xmin + 1) * (ymax - ymin + 1)
        else:
            total += max(1000, int(3400000 * (4 ** (z - 12)) / 200))
    return total, int(total * avg_kb / 1024)


# ═══════════════════════════════════════════
#  Fonts
# ═══════════════════════════════════════════

_font_base = None
_font_cache = {}
_font_info = "default"

def init_font():
    global _font_base, _font_info
    candidates = sorted(glob.glob("*.ttf") + glob.glob("*.ttc"))
    candidates.extend([
        "C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/System/Library/Fonts/PingFang.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ])
    for p in candidates:
        if os.path.exists(p):
            try:
                ImageFont.truetype(p, 10)
                _font_base = p
                _font_info = p
                return
            except Exception:
                continue
    _font_info = "PIL default (no CJK support)"


def get_font(size):
    if size not in _font_cache:
        if _font_base:
            try:
                _font_cache[size] = ImageFont.truetype(_font_base, size)
            except Exception:
                _font_cache[size] = ImageFont.load_default()
        else:
            _font_cache[size] = ImageFont.load_default()
    return _font_cache[size]


# ═══════════════════════════════════════════
#  Coordinate Tools
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
#  Spatial Index
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
        for r in range(math.floor(la0 / self.cd), math.floor(la1 / self.cd) + 1):
            for c in range(math.floor(lo0 / self.cd), math.floor(lo1 / self.cd) + 1):
                self._put((r, c), idx)

    def insert_point(self, lat, lon, idx):
        self._put((math.floor(lat / self.cd), math.floor(lon / self.cd)), idx)

    def query(self, bbox):
        la0, lo0, la1, lo1 = bbox
        hits = set()
        for r in range(math.floor(la0 / self.cd), math.floor(la1 / self.cd) + 1):
            for c in range(math.floor(lo0 / self.cd), math.floor(lo1 / self.cd) + 1):
                key = (r, c)
                if key in self.cells:
                    hits.update(self.cells[key])
        return hits


# ═══════════════════════════════════════════
#  OSM Data Collection
# ═══════════════════════════════════════════

class DataCollector(osmium.SimpleHandler):

    def __init__(self, bbox=None, name_tag="name"):
        super().__init__()
        self.bbox = bbox
        self.name_tag = name_tag
        self.nodes = {}
        self.data_lat_min, self.data_lat_max = 90.0, -90.0
        self.data_lon_min, self.data_lon_max = 180.0, -180.0
        # Existing data
        self.roads = []
        self.buildings = []
        self.water_areas = []
        self.water_lines = []
        self.landuse = []
        self.lbl_road = []
        self.lbl_poi = []
        # New data
        self.coastlines = []
        self.borders = []
        self.places = []
        self.lbl_river = []
        self.idx = {}

    def _in(self, lat, lon):
        if not self.bbox: return True
        return (self.bbox[0] <= lat <= self.bbox[2] and
                self.bbox[1] <= lon <= self.bbox[3])

    def _get_name(self, tags):
        if self.name_tag == "name":
            return tags.get("name", "")
        specific = tags.get(self.name_tag, "")
        if specific: return specific
        return tags.get("name", "")

    def node(self, n):
        if not n.location.valid(): return
        la, lo = n.location.lat, n.location.lon
        self.nodes[n.id] = (la, lo)
        if la < self.data_lat_min: self.data_lat_min = la
        if la > self.data_lat_max: self.data_lat_max = la
        if lo < self.data_lon_min: self.data_lon_min = lo
        if lo > self.data_lon_max: self.data_lon_max = lo
        if not self._in(la, lo): return

        name = self._get_name(n.tags)
        if not name: return

        place = n.tags.get('place', '')
        if place in PLACE_TYPES:
            self.places.append((la, lo, name, place))
        self.lbl_poi.append((la, lo, name))

    def way(self, w):
        nids = [nd.ref for nd in w.nodes]
        if len(nids) < 2: return
        if nids[0] in self.nodes:
            la, lo = self.nodes[nids[0]]
            if not self._in(la, lo): return

        bbox = compute_bbox(nids, self.nodes)
        if bbox is None: return

        t = dict(w.tags)
        name = self._get_name(t)

        if 'highway' in t:
            hw = t['highway']
            if hw in ROAD_STYLES:
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
            if t['waterway'] == 'river' and name:
                mid = nids[len(nids) // 2]
                if mid in self.nodes:
                    la, lo = self.nodes[mid]
                    self.lbl_river.append((la, lo, name))

        elif 'landuse' in t and t['landuse'] in LANDUSE_COLORS:
            self.landuse.append((nids, bbox, LANDUSE_COLORS[t['landuse']]))

        elif 'leisure' in t and t['leisure'] in LEISURE_COLORS:
            self.landuse.append((nids, bbox, LEISURE_COLORS[t['leisure']]))

        # Coastlines
        if 'natural' in t and t['natural'] == 'coastline':
            self.coastlines.append((nids, bbox))

        # Administrative borders
        if 'boundary' in t and t['boundary'] == 'administrative':
            al = t.get('admin_level', '')
            if al in ('2', '4'):
                self.borders.append((nids, bbox, int(al)))

    def build_indices(self):
        for name, src, ext in [
            ('roads',      self.roads,      lambda x: x[1]),
            ('buildings',  self.buildings,   lambda x: x[1]),
            ('water_areas',self.water_areas, lambda x: x[1]),
            ('water_lines',self.water_lines, lambda x: x[1]),
            ('landuse',    self.landuse,     lambda x: x[1]),
            ('coastlines', self.coastlines,  lambda x: x[1]),
            ('borders',    self.borders,     lambda x: x[1]),
        ]:
            si = SpatialIndex()
            for i, item in enumerate(src): si.insert(ext(item), i)
            self.idx[name] = si

        for name, src in [
            ('lbl_road',   self.lbl_road),
            ('lbl_poi',    self.lbl_poi),
            ('places',     self.places),
            ('lbl_river',  self.lbl_river),
        ]:
            si = SpatialIndex()
            for i, item in enumerate(src): si.insert_point(item[0], item[1], i)
            self.idx[name] = si


# ═══════════════════════════════════════════
#  Tile Content Tracking
# ═══════════════════════════════════════════

def mark_tiles(content, feat_bbox, ox, oy, tw, th, z):
    la_min, lo_min, la_max, lo_max = feat_bbox
    gpx1, gpy1 = deg2px(la_max, lo_min, z)
    gpx2, gpy2 = deg2px(la_min, lo_max, z)
    tx0 = max(0, math.floor((gpx1 - ox) / TILE))
    ty0 = max(0, math.floor((gpy1 - oy) / TILE))
    tx1 = min(tw - 1, math.floor((gpx2 - ox) / TILE))
    ty1 = min(th - 1, math.floor((gpy2 - oy) / TILE))
    if tx0 > tx1 or ty0 > ty1: return
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
#  Road Line Width (adapted for low zoom)
# ═══════════════════════════════════════════

def road_line_width(base_w, z):
    if z >= 15: return max(1, base_w + (z - 15))
    if z >= 12: return max(1, base_w)
    if z >= 10: return max(2, base_w - 1)
    return max(1, base_w - 3)


# ═══════════════════════════════════════════
#  Chunked Rendering
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

    # ── 1. Landuse ──
    for idx in data.idx['landuse'].query(cb):
        nids, bb, color = data.landuse[idx]
        if not bbox_intersect(bb, cb): continue
        pts = to_px(nids)
        if len(pts) >= 3:
            try: dr.polygon(pts, fill=color); mc(bb)
            except: pass

    # ── 2. Water areas ──
    for idx in data.idx['water_areas'].query(cb):
        nids, bb = data.water_areas[idx]
        if not bbox_intersect(bb, cb): continue
        pts = to_px(nids)
        if len(pts) >= 3:
            try: dr.polygon(pts, fill=WATER); mc(bb)
            except: pass

    # ── 3. Water lines (rivers) ──
    w_river = max(1, 2 if z < 10 else (z - 8))
    for idx in data.idx['water_lines'].query(cb):
        nids, bb = data.water_lines[idx]
        if not bbox_intersect(bb, cb): continue
        pts = to_px(nids)
        if len(pts) >= 2:
            try: dr.line(pts, fill=WATER_LN, width=w_river); mc(bb)
            except: pass

    # ── 4. Coastlines ──
    cw_coast = 3 if z <= 7 else (2 if z <= 10 else 1)
    for idx in data.idx['coastlines'].query(cb):
        nids, bb = data.coastlines[idx]
        if not bbox_intersect(bb, cb): continue
        pts = to_px(nids)
        if len(pts) >= 2:
            try: dr.line(pts, fill=COASTLINE_COLOR, width=cw_coast); mc(bb)
            except: pass

    # ── 5. Country borders ──
    bw_country = 3 if z <= 7 else (2 if z <= 10 else 1)
    for idx in data.idx['borders'].query(cb):
        nids, bb, level = data.borders[idx]
        if level != 2: continue
        if not bbox_intersect(bb, cb): continue
        pts = to_px(nids)
        if len(pts) >= 2:
            try: dr.line(pts, fill=BORDER_COUNTRY_CLR, width=bw_country); mc(bb)
            except: pass

    # ── 6. Province/State borders ──
    bw_prov = 2 if z <= 8 else 1
    if z >= 7:
        for idx in data.idx['borders'].query(cb):
            nids, bb, level = data.borders[idx]
            if level != 4: continue
            if not bbox_intersect(bb, cb): continue
            pts = to_px(nids)
            if len(pts) >= 2:
                try: dr.line(pts, fill=BORDER_PROVINCE_CLR, width=bw_prov); mc(bb)
                except: pass

    # ── 7. Roads ──
    road_hits = data.idx['roads'].query(cb)
    by_type = defaultdict(list)
    for idx in road_hits:
        nids, bb, hw, _ = data.roads[idx]
        if bbox_intersect(bb, cb): by_type[hw].append((nids, bb))
    for rt in ROAD_ORDER:
        color, base_w, zmin = ROAD_STYLES[rt]
        if z < zmin: continue
        w = road_line_width(base_w, z)
        for nids, bb in by_type.get(rt, []):
            pts = to_px(nids)
            if len(pts) >= 2:
                drew = False
                try: dr.line(pts, fill=color, width=w, joint='curve'); drew = True
                except TypeError:
                    try: dr.line(pts, fill=color, width=w); drew = True
                    except: pass
                if drew: mc(bb)

    # ── 8. Buildings (z>=15) ──
    if z >= 15:
        for idx in data.idx['buildings'].query(cb):
            nids, bb = data.buildings[idx]
            if not bbox_intersect(bb, cb): continue
            pts = to_px(nids)
            if len(pts) >= 3:
                try: dr.polygon(pts, fill=BLD_FILL, outline=BLD_EDGE); mc(bb)
                except: pass

    # ── 9. Place labels (place nodes) ──
    for idx in data.idx['places'].query(cb):
        la, lo, name, pt = data.places[idx]
        zmin = PLACE_ZOOM.get(pt, 14)
        if z < zmin: continue
        gpx, gpy = deg2px(la, lo, z)
        x, y = gpx - ox, gpy - oy
        if 0 <= x < cw and 0 <= y < ch:
            r = PLACE_DOT.get(pt, 1)
            dr.ellipse([x - r, y - r, x + r, y + r], fill='#333333')
            fs = max(8, 8 + PLACE_FNT.get(pt, 0))
            if z >= 8:
                fs = max(fs, z - 5)
            font = get_font(fs)
            dr.text((x + r + 3, y - fs // 2), name, fill=PLACE_TEXT_CLR, font=font)
            mark_point_tile(content, gpx, gpy, ox, oy, tw, th)

    # ── 10. River names (z>=10) ──
    if z >= 10:
        font = get_font(max(8, z - 6))
        for idx in data.idx['lbl_river'].query(cb):
            la, lo, name = data.lbl_river[idx]
            gpx, gpy = deg2px(la, lo, z)
            x, y = gpx - ox, gpy - oy
            if 0 <= x < cw and 0 <= y < ch:
                dr.text((x + 2, y - 7), name, fill=RIVER_LABEL_CLR, font=font)
                mark_point_tile(content, gpx, gpy, ox, oy, tw, th)

    # ── 11. Road names (z>=15) ──
    if z >= 15:
        font = get_font(max(9, 8 + (z - 15)))
        for idx in data.idx['lbl_road'].query(cb):
            la, lo, txt = data.lbl_road[idx]
            gpx, gpy = deg2px(la, lo, z)
            x, y = gpx - ox, gpy - oy
            if 0 <= x < cw and 0 <= y < ch:
                dr.text((x + 2, y - 7), txt, fill='#555555', font=font)
                mark_point_tile(content, gpx, gpy, ox, oy, tw, th)

    # ── 12. POI names (z>=16) ──
    if z >= 16:
        font = get_font(max(9, 8 + (z - 15)))
        for idx in data.idx['lbl_poi'].query(cb):
            la, lo, txt = data.lbl_poi[idx]
            gpx, gpy = deg2px(la, lo, z)
            x, y = gpx - ox, gpy - oy
            if 0 <= x < cw and 0 <= y < ch:
                dr.text((x + 2, y - 7), txt, fill='#333333', font=font)
                mark_point_tile(content, gpx, gpy, ox, oy, tw, th)

    # ── Save tiles with content ──
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
                'JPEG', quality=quality, subsampling=0
            )
            saved += 1

    del img
    return saved, skipped


# ═══════════════════════════════════════════
#  Main Entry
# ═══════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description='OSM PBF → JPEG Tiles (z6-z18)')
    ap.add_argument('input', nargs='?', default=None)
    ap.add_argument('-o', '--output', default=DEFAULT_OUTPUT)
    ap.add_argument('-z', '--zoom', default=None,
                    help='Zoom levels (e.g., 6-14, interactive if not specified)')
    ap.add_argument('-b', '--bbox', help='S,W,N,E')
    ap.add_argument('-q', '--quality', type=int, default=None)
    ap.add_argument('-l', '--lang', default=None,
                    help='Language (en/zh/ja etc., default name)')
    args = ap.parse_args()

    log("\n" + "=" * 55)
    log("  OSM Offline Tile Generator")
    log("  Chunked Rendering + Spatial Indexing + Multilingual + Low Zoom Coastlines")
    log("=" * 55)

    # Input file
    input_file = resolve_input_file(args.input)

    # Output directory
    out_dir = args.output
    check_output_dir(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    interactive = (args.zoom is None)

    # Zoom levels
    if args.zoom:
        parts = args.zoom.split('-')
        z1, z2 = int(parts[0]), int(parts[-1])
    else:
        log("\n  Zoom levels not specified, entering interactive mode...")
        z1, z2 = prompt_zoom()

    # JPEG quality
    quality = args.quality if args.quality else (prompt_quality() if interactive else 75)

    # Bounding box
    bbox = None
    if args.bbox:
        bbox = tuple(map(float, args.bbox.split(',')))
    elif interactive:
        bbox = prompt_bbox()

    # Language
    name_tag = "name"
    if args.lang:
        name_tag = f"name:{args.lang}" if args.lang != "name" else "name"
    elif interactive:
        name_tag = prompt_language()

    # Fonts
    init_font()

    # Parameter summary
    lang_display = name_tag if name_tag != "name" else "name (local language)"
    log(f"\n  Input:   {input_file}")
    log(f"  Output:  {os.path.abspath(out_dir)}/")
    log(f"  Zoom:    z{z1} ~ z{z2}")
    log(f"  Quality: {quality}")
    log(f"  Language:{lang_display}")
    log(f"  Font:    {_font_info}")
    if bbox:
        log(f"  BBox:    S={bbox[0]} W={bbox[1]} N={bbox[2]} E={bbox[3]}")
    else:
        log(f"  BBox:    Auto (data bounds)")
    est_t, est_mb = estimate_storage(bbox, z1, z2, quality)
    log(f"  Estimate:~{est_t:,} tiles, ~{est_mb:,} MB")

    # Phase 1: Reading
    log(f"\n[1/3] Reading {input_file}")
    t0 = time.time()
    dc = DataCollector(bbox, name_tag)
    dc.apply_file(input_file, locations=True)
    dt = time.time() - t0
    log(f"  Took {dt:.1f}s")
    log(f"  Nodes {len(dc.nodes):,}  Roads {len(dc.roads):,}  "
        f"Buildings {len(dc.buildings):,}  Water {len(dc.water_areas)+len(dc.water_lines):,}")
    log(f"  Coastlines {len(dc.coastlines):,}  Borders {len(dc.borders):,}  "
        f"Places {len(dc.places):,}  Rivers {len(dc.lbl_river):,}")

    if not dc.nodes:
        log("Error: No data"); sys.exit(1)

    # Phase 2: Spatial Index
    log(f"\n[2/3] Building spatial index")
    t0 = time.time()
    dc.build_indices()
    dt = time.time() - t0
    idx_cells = sum(len(v.cells) for v in dc.idx.values())
    log(f"  Took {dt:.1f}s  ({idx_cells:,} grid cells)")

    # Phase 3: Rendering
    log(f"\n[3/3] Rendering zoom {z1}~{z2}  (chunk {CHUNK}x{CHUNK})")
    grand_saved = grand_skipped = 0
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
        cx0, cy0 = xmin // CHUNK, ymin // CHUNK
        cx1, cy1 = xmax // CHUNK, ymax // CHUNK
        n_chunks = (cx1 - cx0 + 1) * (cy1 - cy0 + 1)

        log(f"\n  Zoom {z}: {nx}x{ny} = {nx*ny:,} tiles  ({n_chunks} chunks)")
        saved = skipped = 0
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
                if tw <= 0 or th <= 0: continue

                s, k = render_chunk(dc, txa, tya, tw, th, z, out_dir, quality)
                saved += s
                skipped += k
                chunks_done += 1

                if chunks_done % 10 == 0 or chunks_done == n_chunks:
                    elapsed = time.time() - ts
                    rate = (saved + skipped) / elapsed if elapsed > 0 else 0
                    pct = chunks_done * 100 // n_chunks
                    log(f"    [{pct:3d}%] Saved {saved:,}  Skipped {skipped:,}  ({rate:,.0f} t/s)")

        elapsed = time.time() - ts
        rate = (saved + skipped) / elapsed if elapsed > 0 else 0
        log(f"  Zoom {z} done: Saved {saved:,}  Skipped {skipped:,}  {elapsed:.1f}s  ({rate:,.0f} t/s)")
        grand_saved += saved
        grand_skipped += skipped

    # Summary
    total_elapsed = time.time() - t_render
    total_tiles = grand_saved + grand_skipped
    avg = total_tiles / total_elapsed if total_elapsed > 0 else 0

    log(f"\n{'=' * 55}")
    log(f"  Done!")
    log(f"  Saved: {grand_saved:,} tiles")
    log(f"  Skipped: {grand_skipped:,} (blank)")
    log(f"  Time: {total_elapsed:.1f}s  ({avg:,.0f} tiles/s)")
    log(f"  Output: {os.path.abspath(out_dir)}/")
    log(f"{'=' * 55}\n")


if __name__ == '__main__':
    main()
