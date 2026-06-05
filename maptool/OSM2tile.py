#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
osm2tiles.py — OSM PBF → Tile Conversion (OSM Carto Standard Colors)

Features:
  1. power=* / man_made=* not rendered at any zoom level
  2. Road/waterway/railway widths scaled by zoom level
  3. z6-7 simplified: motorway+trunk+river+water bodies+forest areas only
  4. z8 onwards: features progressively enabled
  5. z11/z12 icons: hospital red cross, school/university building, railway station train, airport airplane
  6. Name deduplication within each tile
  7. Block-level bbox binary scanning + numpy node storage
  8. z6-7 motorway+trunk only; z8-11 motorway+trunk+primary only
  9. Administrative boundaries removed
  10. z6-10 waterways: river+canal only; stream deferred to z11
  11. Text sized by zoom+place level + 1px white stroke
  12. z6: Province names 14px (admin_level=4) + prefecture-level city dots (admin_level=5)
  13. z7: Prefecture-level city names 14px (admin_centre level=5, fallback level=4)
  14. z8-12: admin_centres level=5 (prefecture) + level=6 (county)
  15. z13+: place_nodes + admin_centres level=6 supplement
  16. Large file staged rendering: z6-8 / z9-12 label-filtered nodes, z13-14 geographic splitting
"""

import os
import sys
import math
import time
import struct
import zlib
import tempfile
import argparse
import gc
from typing import Optional, Dict

import numpy as np
from PIL import Image, ImageDraw, ImageFont


# ════════════════════════════════════════════════════════════════
# §1  Protobuf Auto-Setup
# ════════════════════════════════════════════════════════════════

def _setup_protobuf():
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    try:
        import fileformat_pb2 as _ff
        import osmformat_pb2 as _of
        return _ff, _of
    except ImportError:
        pass

    _FF = r'''syntax = "proto2";
package OSMPBF;
message BlobHeader {
  required string type = 1;
  optional bytes indexdata = 2;
  required uint32 datasize = 3;
}
message Blob {
  optional bytes raw = 1;
  optional int32 raw_size = 2;
  optional bytes zlib_data = 3;
  optional bytes lzma_data = 4;
}
'''
    _OF = r'''syntax = "proto2";
package OSMPBF;
message HeaderBlock {
  optional HeaderBBox bbox = 1;
  repeated string required_features = 4;
  repeated string optional_features = 5;
  optional string writingprogram = 16;
  optional string source = 17;
}
message HeaderBBox {
  required sint64 left = 1;
  required sint64 right = 2;
  required sint64 top = 3;
  required sint64 bottom = 4;
}
message PrimitiveBlock {
  required StringTable stringtable = 1;
  repeated PrimitiveGroup primitivegroup = 2;
  optional int32 granularity = 17 [default = 100];
  optional int32 date_granularity = 18 [default = 1000];
  optional int64 lat_offset = 19 [default = 0];
  optional int64 lon_offset = 20 [default = 0];
}
message PrimitiveGroup {
  repeated Node nodes = 1;
  optional DenseNodes dense = 2;
  repeated Way ways = 3;
  repeated Relation relations = 4;
  repeated ChangeSet changesets = 5;
}
message StringTable { repeated bytes s = 1; }
message DenseNodes {
  repeated sint64 id = 1 [packed = true];
  optional DenseInfo denseinfo = 5;
  repeated sint64 lat = 8 [packed = true];
  repeated sint64 lon = 9 [packed = true];
  repeated int32 keys_vals = 10 [packed = true];
}
message DenseInfo {
  repeated int32 version = 1 [packed = true];
  repeated sint64 timestamp = 2 [packed = true];
  repeated sint64 changeset = 3 [packed = true];
  repeated sint32 uid = 4 [packed = true];
  repeated uint32 user_sid = 5 [packed = true];
  repeated bool visible = 6 [packed = true];
}
message Node {
  optional sint64 id = 1;
  repeated uint32 keys = 2 [packed = true];
  repeated uint32 vals = 3 [packed = true];
  optional Info info = 4;
  optional sint64 lat = 8;
  optional sint64 lon = 9;
}
message Way {
  optional int64 id = 1;
  repeated uint32 keys = 2 [packed = true];
  repeated uint32 vals = 3 [packed = true];
  optional Info info = 4;
  repeated sint64 refs = 8 [packed = true];
}
message Relation {
  enum MemberType { NODE = 0; WAY = 1; RELATION = 2; }
  optional int64 id = 1;
  repeated uint32 keys = 2 [packed = true];
  repeated uint32 vals = 3 [packed = true];
  optional Info info = 4;
  repeated int32 roles_sid = 8 [packed = true];
  repeated sint64 memids = 9 [packed = true];
  repeated MemberType types = 10 [packed = true];
}
message Info {
  optional int32 version = 1;
  optional int64 timestamp = 2;
  optional int64 changeset = 3;
  optional int32 uid = 4;
  optional uint32 user_sid = 5;
  optional bool visible = 6;
}
message ChangeSet { optional int64 id = 1; }
'''
    print("protobuf module not found, auto-generating...", flush=True)
    ok = False
    try:
        from grpc_tools import protoc as gp
        with tempfile.TemporaryDirectory() as td:
            for n, c in [("fileformat.proto", _FF),
                         ("osmformat.proto", _OF)]:
                with open(os.path.join(td, n), "w") as f:
                    f.write(c)
            ok = gp.main(["grpc_tools.protoc", f"-I{td}",
                          f"--python_out={_dir}",
                          os.path.join(td, "fileformat.proto"),
                          os.path.join(td, "osmformat.proto")]) == 0
    except ImportError:
        pass
    if not ok:
        import subprocess as sp
        try:
            with tempfile.TemporaryDirectory() as td:
                for n, c in [("fileformat.proto", _FF),
                             ("osmformat.proto", _OF)]:
                    with open(os.path.join(td, n), "w") as f:
                        f.write(c)
                ok = sp.run(
                    ["protoc", f"-I{td}", f"--python_out={_dir}",
                     os.path.join(td, "fileformat.proto"),
                     os.path.join(td, "osmformat.proto")],
                    capture_output=True).returncode == 0
        except FileNotFoundError:
            pass
    if not ok:
        print("Please run: pip install protobuf grpcio-tools")
        sys.exit(1)
    import fileformat_pb2 as _ff, osmformat_pb2 as _of
    print("protobuf module auto-generated successfully!", flush=True)
    return _ff, _of


fileformat, osmpbf = _setup_protobuf()


# ════════════════════════════════════════════════════════════════
# §2  Constants & Styles
# ════════════════════════════════════════════════════════════════

BG_COLOR = (242, 239, 233)

RENDER_TAGS = frozenset({
    "highway", "building", "building:part", "waterway", "landuse", "natural",
    "boundary", "admin_level", "name", "name:en", "name:zh", "place",
    "amenity", "shop", "tourism", "leisure", "railway", "aeroway", "bridge",
    "tunnel", "oneway", "surface", "area", "type", "route", "water",
    "wetland", "wood", "power", "man_made", "military", "craft", "office",
    "healthcare", "emergency", "public_transport", "aerialway", "sport",
    "admin_centre", "admin_center",
})

_NS = {
    "ac:4": ("#FF4444", 16, 14, "#CC0000", True, "star"),
    "ac:5": ("#FF6666", 14, 12, "#CC2222", True, "star"),
    "ac:6": ("#FF8888", 12, 11, "#CC4444", True, "star"),
    "ac:7": ("#FFAAAA", 11, 10, "#CC6666", True, "star"),
    "ac:8": ("#FFCCCC", 10, 9,  "#CC8888", True, "star"),
    "tr:highway": ("#888888", 7, 9, "#444444", False, "circle"),
    "tr:railway": ("#666666", 8, 9, "#333333", False, "square"),
    "tr:aeroway": ("#4488CC", 10, 10, "#2266AA", True, "diamond"),
    "tr:waterway": ("#4488CC", 8, 9, "#2266AA", False, "diamond"),
    "am:restaurant": ("#CC8844", 8, 9, "#886622", True, "circle"),
    "am:hospital":   ("#FF4444", 12, 10, "#CC0000", True, "hospital"),
    "am:school":     ("#4488CC", 9, 9, "#2266AA", True, "square"),
    "am:university": ("#4488CC", 11, 10, "#2266AA", True, "square"),
    "am:fuel":       ("#CC4444", 8, 8, "#882222", False, "triangle"),
    "am:bank":       ("#CCCC44", 8, 8, "#888822", False, "square"),
    "sh:supermarket":  ("#CC44CC", 9, 9, "#882288", True, "diamond"),
    "sh:convenience":  ("#CC66CC", 8, 8, "#884488", False, "diamond"),
    "tu:hotel":      ("#CC8844", 9, 9, "#886622", True, "circle"),
    "tu:museum":     ("#8866CC", 10, 10, "#664488", True, "square"),
    "tu:attraction": ("#CC4488", 10, 10, "#882266", True, "star"),
    "pl:city":    ("#CC4444", 14, 14, "#AA0000", True, "circle"),
    "pl:town":    ("#CC6666", 12, 12, "#AA2222", True, "circle"),
    "pl:village": ("#CC8888", 10, 10, "#AA4444", True, "circle"),
    "pl:hamlet":  ("#AAAAAA", 8, 8, "#888888", False, "circle"),
    "pl:suburb":  ("#888888", 8, 9, "#666666", True, "circle"),
    "mn:tower":   ("#888888", 9, 9, "#666666", True, "triangle"),
    "nat:peak":   ("#88CC44", 9, 9, "#668822", True, "triangle"),
    "default":    ("#888888", 7, 8, "#666666", False, "circle"),
}


def _node_cat(tags):
    if "admin_centre" in tags or "admin_center" in tags:
        rl = tags.get("admin_level", "8")
        if rl in "45678":
            return f"ac:{rl}"
    for k in ("highway", "railway", "aeroway", "waterway",
              "aerialway", "public_transport", "route"):
        if k in tags:
            return f"tr:{k}"
    for k in ("shop", "craft", "office"):
        if k in tags:
            return f"sh:{tags[k]}"
    for k in ("amenity", "tourism", "leisure", "healthcare", "emergency"):
        if k in tags:
            return f"am:{tags[k]}"
    if "place" in tags:
        return f"pl:{tags['place']}"
    for k in ("man_made", "military", "power", "telecom"):
        if k in tags:
            return f"mn:{tags[k]}"
    if "natural" in tags:
        return f"nat:{tags['natural']}"
    return "default"


_AREA = {
    "natural=water":       ("#aad3df", 0.85),
    "waterway=river":      ("#aad3df", 0.85),
    "waterway=canal":      ("#aad3df", 0.85),
    "water":               ("#aad3df", 0.85),
    "reservoir":           ("#aad3df", 0.85),
    "basin":               ("#aad3df", 0.85),
    "wetland":             ("#b5d2c0", 0.75),
    "swamp":               ("#b5d2c0", 0.75),
    "natural=wood":        ("#add19e", 0.80),
    "natural=forest":      ("#add19e", 0.80),
    "landuse=forest":      ("#add19e", 0.80),
    "landuse=grass":       ("#c8facc", 0.70),
    "landuse=meadow":      ("#c8facc", 0.70),
    "natural=grassland":   ("#c8facc", 0.70),
    "landuse=farmland":    ("#eef0d5", 0.80),
    "landuse=residential": ("#e0dfdf", 0.60),
    "landuse=commercial":  ("#f2dad9", 0.50),
    "landuse=industrial":  ("#ebe0db", 0.60),
    "landuse=retail":      ("#f2dad9", 0.50),
    "leisure=park":        ("#c8facc", 0.70),
    "leisure=garden":      ("#c8facc", 0.70),
    "amenity=parking":     ("#eeeeee", 0.70),
    "amenity=school":      ("#f0f0d8", 0.60),
    "amenity=university":  ("#f0f0d8", 0.60),
    "amenity=hospital":    ("#f0d0d0", 0.60),
    "building":            ("#d9d0c9", 0.85),
    "building:part":       ("#d9d0c9", 0.85),
    "default":             ("#e0dfdf", 0.50),
}

_WAY = {
    "motorway":        ("#e892a2", 6, "#b5535d", 8, 20),
    "trunk":           ("#f9b29c", 5, "#b57b5d", 7, 19),
    "primary":         ("#fcd6a4", 5, "#d4a74a", 7, 18),
    "secondary":       ("#f7fabf", 4, "#a3a37f", 6, 17),
    "tertiary":        ("#ffffff", 3, "#8f8f8f", 5, 16),
    "unclassified":    ("#dddddd", 2, "#888888", 4, 10),
    "residential":     ("#dddddd", 2, "#888888", 4, 10),
    "service":         ("#eeeeee", 1, "#aaaaaa", 2, 5),
    "living_street":   ("#f2eae3", 2, "#aaaaaa", 3, 9),
    "pedestrian":      ("#dddde8", 2, "#aaaacc", 3, 8),
    "footway":         ("#fa8072", 1, "#886666", 3, 6),
    "path":            ("#fa8072", 1, "#886666", 3, 5),
    "cycleway":        ("#4488cc", 1, "#224466", 3, 6),
    "track":           ("#996644", 1, "#664422", 2, 4),
    "steps":           ("#fa8072", 1, "#886666", 2, 5),
    "railway=rail":    ("#666666", 2, None, 0, 15),
    "railway=tram":    ("#888888", 2, None, 0, 14),
    "railway=subway":  ("#888888", 3, None, 0, 14),
    "waterway=river":  ("#4a88b0", 3, None, 0, 12),
    "waterway=canal":  ("#4a88b0", 2, None, 0, 11),
    "waterway=stream": ("#4a88b0", 1, None, 0, 10),
    "power=line":      ("#888888", 1, None, 0, 10),
    "power=minor_line":("#888888", 1, None, 0, 10),
    "default":         ("#888888", 1, None, 0, 1),
}

_HW_WIDTH_LOW = {
    (6, "motorway"):  1, (6, "trunk"):  1,
    (7, "motorway"):  1, (7, "trunk"):  1,
    (8, "motorway"):  1, (8, "trunk"):  1, (8, "primary"): 1,
    (9, "motorway"):  2, (9, "trunk"):  2, (9, "primary"): 1,
    (10,"motorway"):  3, (10,"trunk"):  2, (10,"primary"): 2,
    (11,"motorway"):  3, (11,"trunk"):  3, (11,"primary"): 2,
}

_RW_WIDTH_LOW = {
    (8, "rail"): 1,
    (9, "rail"): 1, (9, "subway"): 2, (9, "tram"): 1,
    (10,"rail"): 2, (10,"subway"): 2, (10,"tram"): 2,
    (11,"rail"): 2, (11,"subway"): 2, (11,"tram"): 2,
}

_WW_WIDTH_LOW = {
    (6, "river"): 1,
    (7, "river"): 1,
    (8, "river"): 1, (8, "canal"): 1,
    (9, "river"): 2, (9, "canal"): 1,
    (10,"river"): 2, (10,"canal"): 2,
    (11,"river"): 2, (11,"canal"): 2,
}


def _mix(color, alpha=1.0, bg=BG_COLOR):
    h = color.lstrip("#")
    r = int(h[0:2], 16); g = int(h[2:4], 16); b = int(h[4:6], 16)
    if alpha >= 1.0:
        return (r, g, b)
    a = alpha
    return (int(r * a + bg[0] * (1 - a)),
            int(g * a + bg[1] * (1 - a)),
            int(b * a + bg[2] * (1 - a)))


def _area_style(tags):
    for k in ("waterway", "natural", "landuse", "leisure", "amenity",
              "building", "building:part", "tourism"):
        if k in tags:
            combo = f"{k}={tags[k]}"
            if combo in _AREA:
                return _AREA[combo]
            if tags[k] in _AREA:
                return _AREA[tags[k]]
    return _AREA["default"]


def _way_style(tags):
    hw = tags.get("highway")
    if hw:
        return _WAY.get(hw, _WAY["residential"])
    for k in ("railway", "waterway", "aeroway", "power", "route"):
        if k in tags:
            combo = f"{k}={tags[k]}"
            if combo in _WAY:
                return _WAY[combo]
    return _WAY["default"]


_AK = frozenset({
    "building", "building:part", "landuse", "natural", "leisure",
    "amenity", "tourism", "boundary", "place", "waterway", "sport",
})
_AV = frozenset({
    "water", "wetland", "swamp", "marsh", "forest", "wood", "grass", "meadow",
    "farmland", "farmyard", "residential", "commercial", "industrial", "retail",
    "military", "cemetery", "recreation_ground", "park", "garden", "playground",
    "golf_course", "pitch", "stadium", "parking", "place_of_worship", "school",
    "university", "hospital", "attraction", "yes",
})


def is_area(tags):
    if tags.get("area") == "yes":
        return True
    if tags.get("area") == "no":
        return False
    for k in _AK:
        if k in tags:
            if k in ("building", "building:part"):
                return True
            if tags[k] in _AV:
                return True
    return False


def is_poi(tags):
    return any(k in tags for k in (
        "amenity", "shop", "tourism", "leisure", "craft", "office",
        "healthcare", "emergency", "place", "man_made", "natural", "power"))


# ════════════════════════════════════════════════════════════════
# §3  Zoom Level Filtering
# ════════════════════════════════════════════════════════════════

def _is_blocked(tags):
    if tags.get("power"):
        return True
    if tags.get("man_made"):
        return True
    return False


def _way_zoom_ok(tags, zoom):
    if _is_blocked(tags):
        return False
    if "building" in tags or "building:part" in tags:
        return zoom >= 13
    hw = tags.get("highway")
    if hw:
        if zoom <= 7:
            return hw in ("motorway", "trunk")
        if zoom <= 11:
            return hw in ("motorway", "trunk", "primary")
        return True
    if "railway" in tags:
        if zoom <= 11:
            return False
        return True
    wk = tags.get("waterway")
    if wk in ("river", "canal", "stream", "drain", "ditch"):
        if zoom <= 7:
            return wk == "river"
        if zoom <= 10:
            return wk in ("river", "canal")
        return True
    if zoom <= 8:
        nat = tags.get("natural", "")
        lu = tags.get("landuse", "")
        if nat in ("water", "wetland", "glacier", "coastline"):
            return True
        if lu == "water":
            return True
        if "water" in tags:
            return True
        if zoom <= 7:
            if lu == "forest" or nat == "wood":
                return True
            return False
        if lu in ("forest", "grass", "meadow", "farmland", "farmyard",
                  "residential", "commercial", "industrial", "retail",
                  "cemetery"):
            return True
        if nat in ("wood", "grassland", "scrub"):
            return True
        return False
    return True


def _poi_zoom_ok(tags, zoom):
    if _is_blocked(tags):
        return False
    if zoom < 11:
        return False
    if zoom <= 12:
        if tags.get("amenity") in ("hospital", "school", "university"):
            return True
        if tags.get("railway") == "station":
            return True
        if tags.get("aeroway") == "aerodrome":
            return True
        return False
    if zoom == 13:
        p = tags.get("place")
        if p:
            return p not in ("neighbourhood", "locality",
                             "isolated_dwelling", "farm")
        return bool(tags.get("amenity") or tags.get("tourism") or
                    tags.get("shop") or tags.get("man_made"))
    return True


def _label_zoom_ok(tags, zoom):
    if _is_blocked(tags):
        return False
    p = tags.get("place")
    if zoom <= 12:
        return False
    if zoom <= 14:
        if p:
            return p in ("city", "town", "village", "suburb")
        nat = tags.get("natural")
        if nat in ("peak", "volcano", "water", "bay", "cape"):
            return True
        wk = tags.get("waterway")
        if wk in ("river", "canal"):
            return True
        return False
    return True


# ════════════════════════════════════════════════════════════════
_SIMPLIFY_TOL = {6: 5.0, 7: 4.0, 8: 3.0, 9: 2.0, 10: 1.5}


def _douglas_peucker(points, epsilon):
    """Douglas-Peucker line simplification (iterative, pixel space)"""
    n = len(points)
    if n <= 2:
        return points
    eps2 = epsilon * epsilon
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        start, end = stack.pop()
        if end - start <= 1:
            continue
        dmax = 0.0
        idx = start
        x1, y1 = points[start]
        x2, y2 = points[end]
        dx, dy = x2 - x1, y2 - y1
        lsq = dx * dx + dy * dy
        if lsq < 1e-12:
            for i in range(start + 1, end):
                px, py = points[i]
                d = (px - x1) ** 2 + (py - y1) ** 2
                if d > dmax:
                    dmax = d; idx = i
        else:
            for i in range(start + 1, end):
                px, py = points[i]
                t = max(0.0, min(1.0,
                    ((px - x1) * dx + (py - y1) * dy) / lsq))
                d = (px - x1 - t * dx) ** 2 + (py - y1 - t * dy) ** 2
                if d > dmax:
                    dmax = d; idx = i
        if dmax > eps2:
            keep[idx] = True
            stack.append((start, idx))
            stack.append((idx, end))
    return [points[i] for i in range(n) if keep[i]]


# §3.1  Text font size + white stroke
# ════════════════════════════════════════════════════════════════

def _label_font_size(tags, zoom):
    p = tags.get("place")
    if zoom <= 14:
        if p == "city": return 11
        if p == "town": return 10
        return 9
    if p == "city": return 10
    return 9


def _draw_text(draw, x, y, nm, font_size, fill="#333333"):
    ft = _font(font_size)
    draw.text((x, y), nm, fill=fill, font=ft,
              stroke_width=1, stroke_fill=(255, 255, 255))


# ════════════════════════════════════════════════════════════════
# §3.2  z11/z12 small icons
# ════════════════════════════════════════════════════════════════

def _precompute_icon(fill_pixels, n=8):
    filled = set(fill_pixels)
    outline = set()
    for x in range(n):
        for y in range(n):
            if (x, y) in filled:
                continue
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)):
                if (x + dx, y + dy) in filled:
                    outline.add((x, y))
                    break
    return filled, outline


_HOSP_FILLED = set()
for i in range(8):
    _HOSP_FILLED.add((3, i)); _HOSP_FILLED.add((4, i))
for i in range(8):
    _HOSP_FILLED.add((i, 3)); _HOSP_FILLED.add((i, 4))

_SCHOOL_FILLED = set()
for y in range(0, 3):
    _SCHOOL_FILLED.add((3, y)); _SCHOOL_FILLED.add((4, y))
for y in range(3, 7):
    for x in range(2, 6):
        _SCHOOL_FILLED.add((x, y))
_SCHOOL_FILLED.add((1, 3)); _SCHOOL_FILLED.add((6, 3))

_TRAIN_FILLED = set()
for y in range(0, 3):
    _TRAIN_FILLED.add((3, y)); _TRAIN_FILLED.add((4, y))
for x in range(2, 6):
    _TRAIN_FILLED.add((x, 3))
for x in range(1, 7):
    _TRAIN_FILLED.add((x, 5)); _TRAIN_FILLED.add((x, 6))
_TRAIN_FILLED.add((2, 4)); _TRAIN_FILLED.add((5, 4))
_TRAIN_FILLED.add((2, 7)); _TRAIN_FILLED.add((5, 7))

_PLANE_FILLED = set()
for y in range(0, 6):
    _PLANE_FILLED.add((3, y)); _PLANE_FILLED.add((4, y))
for y in range(3, 5):
    for x in range(1, 7):
        _PLANE_FILLED.add((x, y))
_PLANE_FILLED.add((2, 7)); _PLANE_FILLED.add((5, 7))

_HOSP_F, _HOSP_O     = _precompute_icon(_HOSP_FILLED)
_SCHOOL_F, _SCHOOL_O  = _precompute_icon(_SCHOOL_FILLED)
_TRAIN_F, _TRAIN_O    = _precompute_icon(_TRAIN_FILLED)
_PLANE_F, _PLANE_O    = _precompute_icon(_PLANE_FILLED)

_HOSP_COLOR   = "#DD0000"
_SCHOOL_COLOR = "#556B2F"
_TRAIN_COLOR  = "#AA2222"
_PLANE_COLOR  = "#2266AA"


def _draw_small_icon(draw, cx, cy, filled, outline, fill_color):
    ox = int(cx) - 4; oy = int(cy) - 4
    for x, y in outline:
        draw.point((ox + x, oy + y), fill=(255, 255, 255))
    for x, y in filled:
        draw.point((ox + x, oy + y), fill=fill_color)


# ════════════════════════════════════════════════════════════════
# §3.3  Admin centre extraction (admin_centre, authoritative data)
# ════════════════════════════════════════════════════════════════

def build_admin_centres(relations):
    province_names = {}
    admin_centres = {}
    for _, tags, members in relations:
        if (tags.get("type") == "boundary"
                and tags.get("boundary") == "administrative"):
            al = tags.get("admin_level", "")
            nm = tags.get("name", "")
            if not nm or al not in ("4", "5", "6"):
                continue
            for m in members:
                if (m["role"] in ("admin_centre", "admin_center", "label")
                        and m["type"] == "node"):
                    nid = m["ref"]
                    if nid not in admin_centres:
                        admin_centres[nid] = {}
                    admin_centres[nid][al] = nm
                    if al == "4":
                        province_names[nid] = nm
    return province_names, admin_centres


# ════════════════════════════════════════════════════════════════
# §4  CompactNodeStore
# ════════════════════════════════════════════════════════════════

class CompactNodeStore:
    _FL = 2_000_000

    def __init__(self):
        self._bi, self._bl, self._bo = [], [], []
        self._ids: Optional[np.ndarray] = None
        self._la: Optional[np.ndarray] = None
        self._lo: Optional[np.ndarray] = None
        self._done = False

    def add(self, nid, lat, lon):
        self._bi.append(nid); self._bl.append(lat); self._bo.append(lon)
        if len(self._bi) >= self._FL:
            self._flush()

    def _flush(self):
        if not self._bi:
            return
        ni = np.array(self._bi, dtype=np.int64)
        la = np.array(self._bl, dtype=np.float32)
        lo = np.array(self._bo, dtype=np.float32)
        self._bi.clear(); self._bl.clear(); self._bo.clear()
        if self._ids is not None:
            self._ids = np.concatenate([self._ids, ni])
            self._la = np.concatenate([self._la, la])
            self._lo = np.concatenate([self._lo, lo])
        else:
            self._ids, self._la, self._lo = ni, la, lo

    def finalize(self):
        self._flush()
        if self._ids is None or len(self._ids) == 0:
            self._ids = np.empty(0, dtype=np.int64)
            self._la = np.empty(0, dtype=np.float32)
            self._lo = np.empty(0, dtype=np.float32)
        else:
            o = np.argsort(self._ids)
            self._ids = self._ids[o]; self._la = self._la[o]
            self._lo = self._lo[o]; del o
        self._done = True

    def _ensure(self):
        if not self._done: self.finalize()

    def __getitem__(self, nid):
        self._ensure()
        i = int(np.searchsorted(self._ids, nid))
        if i < len(self._ids) and self._ids[i] == nid:
            return (float(self._la[i]), float(self._lo[i]))
        raise KeyError(nid)

    def __contains__(self, nid):
        self._ensure()
        i = int(np.searchsorted(self._ids, nid))
        return i < len(self._ids) and self._ids[i] == nid

    def __len__(self):
        self._ensure(); return len(self._ids)

    def items(self):
        self._ensure()
        for i in range(len(self._ids)):
            yield int(self._ids[i]), (float(self._la[i]), float(self._lo[i]))

    def get_coords_batch(self, idc):
        self._ensure()
        if not idc:
            e = np.empty(0, dtype=np.int64)
            return e, e.astype(np.float64), e.astype(np.float64)
        t = np.fromiter(idc, dtype=np.int64)
        idx = np.searchsorted(self._ids, t)
        v = idx < len(self._ids)
        m = np.zeros(len(t), dtype=bool); m[v] = self._ids[idx[v]] == t[v]
        s = idx[m]
        return (t[m], self._la[s].astype(np.float64),
                self._lo[s].astype(np.float64))

    def filter_ids(self, cands):
        self._ensure()
        if not cands: return set()
        t = np.fromiter(cands, dtype=np.int64)
        idx = np.searchsorted(self._ids, t)
        v = idx < len(self._ids)
        m = np.zeros(len(t), dtype=bool); m[v] = self._ids[idx[v]] == t[v]
        return set(t[m].tolist())


# ════════════════════════════════════════════════════════════════
# §5  OSM PBF Parser
# ════════════════════════════════════════════════════════════════

_PLACE_TYPES = frozenset(("city", "town", "village", "suburb"))


class OSMParser:
    NEEDED = {"OsmSchema-V0.6", "DenseNodes"}
    MARGIN = 2.0

    def __init__(self, way_filter=None, node_filter=None,
                 relation_filter=None, bbox=None, strict_filter=False):
        self.way_filter = way_filter or (lambda t: True)
        self.node_filter = node_filter or (lambda t: bool(t))
        self.relation_filter = relation_filter or (lambda t: True)
        self.bbox = bbox
        self.strict_filter = strict_filter
        self.place_nodes: Dict[int, tuple] = {}

    def _in_bbox(self, lat, lon):
        if self.bbox is None: return True
        s, w, n, e = self.bbox
        return (s - 0.02 <= lat <= n + 0.02 and
                w - 0.02 <= lon <= e + 0.02)

    def _should_store(self, nid, lat, lon, tags, nif):
        ib = self._in_bbox(lat, lon)
        if nif is not None:
            if isinstance(nif, np.ndarray):
                idx = int(np.searchsorted(nif, nid))
                in_f = idx < len(nif) and nif[idx] == nid
            else:
                in_f = nid in nif
            if self.strict_filter:
                return in_f and ib
            return in_f or (ib and bool(tags))
        if self.bbox is not None and not ib: return False
        if not tags: return True
        return self.node_filter(tags)

    @staticmethod
    def _rv(data, pos):
        r = 0; s = 0
        while pos < len(data):
            b = data[pos]; r |= (b & 0x7f) << s; pos += 1
            if not (b & 0x80): return r, pos
            s += 7
        return r, pos

    @staticmethod
    def _rsv(data, pos):
        v, pos = OSMParser._rv(data, pos)
        return (v >> 1) ^ -(v & 1), pos

    @staticmethod
    def _sf(data, pos, wt):
        if wt == 0:
            while pos < len(data) and data[pos] & 0x80: pos += 1
            return pos + 1
        if wt == 1: return pos + 8
        if wt == 2:
            l, pos = OSMParser._rv(data, pos); return pos + l
        if wt == 5: return pos + 4
        return len(data)

    def _quick_bbox_check(self, data):
        if self.bbox is None: return True
        try: return self._do_bbox_check(data)
        except: return True

    def _do_bbox_check(self, data):
        s, w, n, e = self.bbox; M = self.MARGIN
        s -= M; w -= M; n += M; e += M
        gran = 100; lato = 0; lono = 0; has_dense = False
        pos = 0; length = len(data)
        while pos < length:
            tag, np_ = self._rv(data, pos)
            if np_ <= pos: break
            pos = np_; fn = tag >> 3; wt = tag & 7
            if fn == 17 and wt == 0:
                gran, pos = self._rv(data, pos)
            elif fn == 19 and wt == 0:
                lato, pos = self._rv(data, pos)
            elif fn == 20 and wt == 0:
                lono, pos = self._rv(data, pos)
            elif fn == 2 and wt == 2:
                gl, pos = self._rv(data, pos)
                ge = pos + gl; gp = pos
                while gp < ge:
                    gt, gn = self._rv(data, gp)
                    if gn <= gp: break
                    gp = gn; gf = gt >> 3; gw = gt & 7
                    if gf == 2 and gw == 2:
                        has_dense = True
                        dl, gp = self._rv(data, gp)
                        de = gp + dl; dp = gp; rl = ro = None
                        while dp < de and (rl is None or ro is None):
                            dt, dn = self._rv(data, dp)
                            if dn <= dp: break
                            dp = dn; df = dt >> 3; dw = dt & 7
                            if df == 8 and dw == 2:
                                pl, dp = self._rv(data, dp)
                                if pl > 0: rl, _ = self._rsv(data, dp)
                                dp += pl
                            elif df == 9 and dw == 2:
                                pl, dp = self._rv(data, dp)
                                if pl > 0: ro, _ = self._rsv(data, dp)
                                dp += pl
                            else: dp = self._sf(data, dp, dw)
                        if rl is not None and ro is not None:
                            lat = 1e-9 * (lato + gran * rl)
                            lon = 1e-9 * (lono + gran * ro)
                            if s <= lat <= n and w <= lon <= e: return True
                        gp = de
                    else: gp = self._sf(data, gp, gw)
                pos = ge
            else: pos = self._sf(data, pos, wt)
        return not has_dense

    def parse(self, filepath, progress_interval=5.0,
              ways_only=False, node_id_filter=None):
        nodes = None if ways_only else CompactNodeStore()
        ways, relations = [], []
        use_filter = (self.bbox is not None and not ways_only)
        with open(filepath, 'rb') as f:
            data, btype = self._read_blob(f)
            if data is None:
                print("Error: Unable to read file header", flush=True)
                return nodes, ways, relations
            hdr = osmpbf.HeaderBlock(); hdr.ParseFromString(data)
            for feat in hdr.required_features:
                if feat not in self.NEEDED:
                    print(f"Error: Unsupported feature '{feat}'", flush=True)
                    return nodes, ways, relations
            blk = 0; skipped = 0; t0 = time.time(); tp = t0; nc = 0
            while True:
                data, btype = self._read_blob(f)
                if data is None: break
                if btype != "OSMData": continue
                blk += 1
                if use_filter and not self._quick_bbox_check(data):
                    skipped += 1; continue
                try:
                    pb = osmpbf.PrimitiveBlock(); pb.ParseFromString(data)
                except Exception as e:
                    print(f"\nWarning: Block {blk} parse error: {e}", flush=True)
                    continue
                st = pb.stringtable.s
                for grp in pb.primitivegroup:
                    if not ways_only and grp.HasField('dense'):
                        nc += self._dense(grp.dense, pb, st, nodes,
                                          node_id_filter)
                    if not ways_only:
                        for nd in grp.nodes:
                            nc += self._node(nd, pb, st, nodes,
                                             node_id_filter)
                    for wy in grp.ways: self._way(wy, st, ways)
                    for rl in grp.relations: self._rel(rl, st, relations)
                now = time.time()
                if now - tp >= progress_interval:
                    print(f"\rParsing: {blk} blocks, skipped {skipped}, "
                          f"nodes {nc:,}, ways {len(ways):,}, "
                          f"{now - t0:.0f}s", end="", flush=True)
                    tp = now
        if nodes is not None: nodes.finalize()
        print(f"\nParsing complete: nodes {nc:,}, ways {len(ways):,}, "
              f"relations {len(relations):,}, place nodes {len(self.place_nodes):,}, "
              f"skipped {skipped} blocks, elapsed {time.time() - t0:.1f}s", flush=True)
        return nodes, ways, relations

    def _dense(self, dense, pb, st, nodes, nif):
        ids, lats, lons = [], [], []
        lid = lla = llo = 0
        for i in range(len(dense.id)):
            lid += dense.id[i]; lla += dense.lat[i]; llo += dense.lon[i]
            ids.append(lid)
            lats.append(1e-9 * (pb.lat_offset + pb.granularity * lla))
            lons.append(1e-9 * (pb.lon_offset + pb.granularity * llo))
        if self.bbox is not None and nif is None:
            bs, bw, bn, be = self.bbox
            if not any(bs <= la <= bn and bw <= lo <= be
                       for la, lo in zip(lats, lons)):
                return 0
        tl, cur, ki = [], {}, None
        for kv in dense.keys_vals:
            if kv == 0:
                tl.append(cur); cur = {}; ki = None
            elif ki is None: ki = kv
            else:
                try:
                    k = sys.intern(st[ki].decode())
                    if k in RENDER_TAGS: cur[k] = st[kv].decode()
                except: pass
                ki = None
        if cur or ki is not None: tl.append(cur)
        while len(tl) < len(ids): tl.append({})

        # numpy batch filtering
        if isinstance(nif, np.ndarray) and len(nif) > 0:
            ids_arr = np.array(ids, dtype=np.int64)
            nif_idx = np.searchsorted(nif, ids_arr)
            nif_idx = np.clip(nif_idx, 0, len(nif) - 1)
            in_nif = nif[nif_idx] == ids_arr

            if self.bbox is not None:
                lats_a = np.array(lats, dtype=np.float64)
                lons_a = np.array(lons, dtype=np.float64)
                bs, bw, bn, be = self.bbox
                in_bbox = ((lats_a >= bs - 0.02) & (lats_a <= bn + 0.02)
                           & (lons_a >= bw - 0.02) & (lons_a <= be + 0.02))
            else:
                in_bbox = None

            if self.strict_filter:
                store_mask = in_nif & in_bbox if in_bbox is not None else in_nif
            else:
                has_tags = np.array([bool(t) for t in tl], dtype=bool)
                if in_bbox is not None:
                    store_mask = in_nif | (in_bbox & has_tags)
                else:
                    store_mask = in_nif | has_tags

            stored = 0
            for j in range(len(ids)):
                tags = tl[j]
                p = tags.get("place")
                if p in _PLACE_TYPES:
                    nm = tags.get("name", "")
                    if nm:
                        self.place_nodes[ids[j]] = (nm, p)
                if store_mask[j]:
                    nodes.add(ids[j], lats[j], lons[j]); stored += 1
            return stored
        else:
            stored = 0
            for j in range(len(ids)):
                tags = tl[j]
                p = tags.get("place")
                if p in _PLACE_TYPES:
                    nm = tags.get("name", "")
                    if nm:
                        self.place_nodes[ids[j]] = (nm, p)
                if self._should_store(ids[j], lats[j], lons[j], tags, nif):
                    nodes.add(ids[j], lats[j], lons[j]); stored += 1
            return stored

    def _node(self, nd, pb, st, nodes, nif):
        tags = {}
        for i in range(len(nd.keys)):
            try:
                k = sys.intern(st[nd.keys[i]].decode())
                if k in RENDER_TAGS: tags[k] = st[nd.vals[i]].decode()
            except: pass
        lat = 1e-9 * (pb.lat_offset + pb.granularity * nd.lat)
        lon = 1e-9 * (pb.lon_offset + pb.granularity * nd.lon)
        p = tags.get("place")
        if p in _PLACE_TYPES:
            nm = tags.get("name", "")
            if nm:
                self.place_nodes[nd.id] = (nm, p)
        if self._should_store(nd.id, lat, lon, tags, nif):
            nodes.add(nd.id, lat, lon); return 1
        return 0

    def _way(self, wy, st, ways):
        tags = {}
        for i in range(len(wy.keys)):
            try:
                k = sys.intern(st[wy.keys[i]].decode())
                if k in RENDER_TAGS: tags[k] = st[wy.vals[i]].decode()
            except: pass
        if self.way_filter(tags):
            ref = 0; nids = []
            for d in wy.refs: ref += d; nids.append(ref)
            ways.append((wy.id, tags, nids))

    def _rel(self, rl, st, relations):
        tags = {}
        for i in range(len(rl.keys)):
            try:
                k = sys.intern(st[rl.keys[i]].decode())
                if k in RENDER_TAGS: tags[k] = st[rl.vals[i]].decode()
            except: pass
        if self.relation_filter(tags):
            members, mid = [], 0; tn = ("node", "way", "relation")
            for i in range(len(rl.memids)):
                mid += rl.memids[i]
                members.append({"ref": mid, "type": tn[rl.types[i]],
                    "role": st[rl.roles_sid[i]].decode("utf-8",
                                                        errors="replace")})
            relations.append((rl.id, tags, members))

    @staticmethod
    def _read_blob(f):
        hdr = f.read(4)
        if len(hdr) < 4: return None, None
        sz = struct.unpack('>I', hdr)[0]; hb = f.read(sz)
        if len(hb) < sz: return None, None
        bh = fileformat.BlobHeader(); bh.ParseFromString(hb)
        btype = bh.type
        blob = fileformat.Blob(); blob.ParseFromString(f.read(bh.datasize))
        if blob.HasField('raw'): return blob.raw, btype
        if blob.HasField('zlib_data'):
            return zlib.decompress(blob.zlib_data), btype
        if blob.HasField('lzma_data'):
            import lzma; return lzma.decompress(blob.lzma_data), btype
        return b'', btype


# ════════════════════════════════════════════════════════════════
# §6  Font Loading
# ════════════════════════════════════════════════════════════════

_fc: Dict[int, ImageFont.FreeTypeFont] = {}


def _font(size):
    if size in _fc: return _fc[size]
    script_dir = os.path.dirname(os.path.abspath(__file__))
    try:
        local_fonts = sorted(f for f in os.listdir(script_dir)
                             if f.lower().endswith(('.ttf', '.ttc', '.otf')))
    except OSError: local_fonts = []
    for fname in local_fonts:
        try:
            f = ImageFont.truetype(os.path.join(script_dir, fname), size)
            _fc[size] = f; return f
        except: continue
    for p in ["C:/Windows/Fonts/msyh.ttc", "C:/Windows/Fonts/simhei.ttf",
              "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
              "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
              "/System/Library/Fonts/PingFang.ttc"]:
        if os.path.exists(p):
            try:
                f = ImageFont.truetype(p, size)
                _fc[size] = f; return f
            except: continue
    try: f = ImageFont.truetype("arial.ttf", size)
    except: f = ImageFont.load_default()
    _fc[size] = f; return f


# ════════════════════════════════════════════════════════════════
# §7  render_region (z8-12 unified admin_centres)
# ════════════════════════════════════════════════════════════════

def deg2tile(lon, lat, z):
    n = 2.0 ** z
    tx = int((lon + 180.0) / 360.0 * n)
    ty = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return tx, ty


_AC_FONT = {
    8:  (11, 10),
    9:  (10, 10),
    10: (10,  9),
    11: (10,  9),
    12: (10,  9),
}


def render_region(nodes, ways, relations, zoom=14,
                  tile_size=256, tile_format="JPG", output_dir="tiles",
                  render_areas=True, render_ways=True,
                  render_pois=True, render_labels=True,
                  progress_interval=5.0,
                  place_nodes=None, admin_centres=None,
                  province_names=None):

    if place_nodes is None: place_nodes = {}
    if admin_centres is None: admin_centres = {}
    if province_names is None: province_names = {}

    simplify_eps = _SIMPLIFY_TOL.get(zoom, 0)

    print(f"Ways: {len(ways):,}", flush=True)
    print("Collecting candidate nodes...", flush=True)
    t0 = time.time()
    wn = set()
    for _, _, nids in ways: wn.update(nids)
    cands = wn | set(place_nodes.keys()) | set(admin_centres.keys())
    del wn
    fnids = nodes.filter_ids(cands); del cands
    print(f"  After filtering: {len(fnids):,}  ({time.time() - t0:.1f}s)", flush=True)
    if not fnids: print("No nodes found", flush=True); return

    f_ids, f_lats, f_lons = nodes.get_coords_batch(fnids)
    if len(f_ids) == 0: print("No nodes found", flush=True); return

    lo0, lo1 = float(f_lons.min()), float(f_lons.max())
    la0, la1 = float(f_lats.min()), float(f_lats.max())

    global_tx0, global_ty0 = deg2tile(lo0, la1, zoom)
    global_tx1, global_ty1 = deg2tile(lo1, la0, zoom)
    canvas_px0 = global_tx0 * tile_size
    canvas_py0 = global_ty0 * tile_size

    ntx = global_tx1 - global_tx0 + 1
    nty = global_ty1 - global_ty0 + 1
    cw = ntx * tile_size
    ch = nty * tile_size

    n = 2.0 ** zoom * tile_size
    gpx = ((f_lons + 180.0) / 360.0 * n).astype(np.float64)
    gpy = ((1.0 - np.arcsinh(np.tan(np.radians(f_lats))) / math.pi) / 2.0 * n).astype(np.float64)
    fpx = (gpx - canvas_px0).astype(np.float32)
    fpy = (gpy - canvas_py0).astype(np.float32)
    del f_lats, f_lons, gpx, gpy

    tx = np.floor(fpx / tile_size).astype(np.int32)
    ty = np.floor(fpy / tile_size).astype(np.int32)
    occ = set(zip(tx.tolist(), ty.tolist()))
    os.makedirs(output_dir, exist_ok=True)
    print(f"Canvas: {cw}x{ch}, Tiles: {ntx}x{nty}={ntx * nty}, "
          f"With data: {len(occ)}, Global origin: ({global_tx0},{global_ty0})",
          flush=True)

    print("Building node-to-way index...", flush=True)
    ti = time.time(); fns = set(f_ids.tolist())
    _cn, _cw, _tn, _tw = [], [], [], []
    for i, (_, _, nids) in enumerate(ways):
        for nid in nids:
            if nid in fns: _tn.append(nid); _tw.append(i)
        if len(_tn) >= 5_000_000:
            _cn.append(np.array(_tn, dtype=np.int64))
            _cw.append(np.array(_tw, dtype=np.int32))
            _tn.clear(); _tw.clear()
    if _tn:
        _cn.append(np.array(_tn, dtype=np.int64))
        _cw.append(np.array(_tw, dtype=np.int32))
    del _tn, _tw, fns

    if _cn:
        pn = np.concatenate(_cn); pw = np.concatenate(_cw); del _cn, _cw
        o = np.argsort(pn, kind='stable')
        pn = pn[o]; pw = pw[o]; del o
        uniq, fi = np.unique(pn, return_index=True)
        del pn; fi = fi.astype(np.int32)
    else:
        pw = np.array([], dtype=np.int32)
        uniq = np.array([], dtype=np.int64)
        fi = np.array([], dtype=np.int32); del _cn, _cw
    print(f"  Index: {len(uniq):,} nodes, {len(pw):,} associations, "
          f"{time.time() - ti:.1f}s", flush=True)

    rendered = 0; skipped = 0; t0 = time.time(); tp = t0
    is_jpg = tile_format.upper() in ("JPG", "JPEG")
    save_kw = {"quality": 75, "subsampling": 0} if is_jpg else {}

    for tile_y in range(nty):
        for tile_x in range(ntx):
            if (tile_x, tile_y) not in occ: skipped += 1; continue
            y0 = tile_y * tile_size; x0 = tile_x * tile_size
            y1 = min(y0 + tile_size, ch); x1 = min(x0 + tile_size, cw)
            tw = x1 - x0; th = y1 - y0
            if tw <= 0 or th <= 0: skipped += 1; continue

            mask = (fpx >= x0) & (fpx < x1) & (fpy >= y0) & (fpy < y1)
            tidx = np.where(mask)[0]
            npc = {}
            for idx in tidx:
                npc[int(f_ids[idx])] = (float(fpx[idx]), float(fpy[idx]))
            if not npc: skipped += 1; continue

            twi = set()
            for nid in npc:
                i = int(np.searchsorted(uniq, nid))
                if i < len(uniq) and uniq[i] == nid:
                    s = fi[i]
                    e = fi[i + 1] if i + 1 < len(uniq) else len(pw)
                    for w in pw[s:e]: twi.add(int(w))

            areas, pois, lines = [], [], []
            for wi in twi:
                wid, wt, nids = ways[wi]
                if not _way_zoom_ok(wt, zoom): continue
                coords = {nid: npc[nid] for nid in nids if nid in npc}
                if not coords: continue
                if is_area(wt): areas.append((wid, wt, coords))
                elif is_poi(wt): pois.append((wid, wt, coords))
                else: lines.append((wid, wt, coords))

            img = Image.new("RGB", (tile_size, tile_size), BG_COLOR)
            draw = ImageDraw.Draw(img)

            if render_areas and areas:
                for _, at, ac in areas:
                    fc, fo = _area_style(at)
                    pts = [(px - x0, py - y0) for _, (px, py) in ac.items()]
                    if simplify_eps > 0 and len(pts) > 4:
                        pts = _douglas_peucker(pts, simplify_eps)
                    if len(pts) >= 3: draw.polygon(pts, fill=_mix(fc, fo))

            if render_ways and lines:
                for _, lt, lc in lines:
                    pts = [(px - x0, py - y0) for _, (px, py) in lc.items()]
                    if len(pts) < 2: continue
                    if simplify_eps > 0:
                        pts = _douglas_peucker(pts, simplify_eps)
                    if len(pts) < 2: continue
                    style = _way_style(lt)
                    fc = style[0]; default_fw = style[1]
                    cc = style[2]; default_cw = style[3]
                    hw = lt.get("highway", "")
                    rk = lt.get("railway", "")
                    wk = lt.get("waterway", "")

                    if zoom <= 11 and hw in ("motorway", "trunk", "primary"):
                        fw = _HW_WIDTH_LOW.get((zoom, hw), default_fw)
                        draw.line(pts, fill=_mix(fc), width=fw, joint="curve")
                        if zoom >= 11 and cc:
                            draw.line(pts, fill=_mix(cc),
                                width=_HW_WIDTH_LOW.get((zoom, hw),
                                                        default_cw),
                                joint="curve")
                    elif zoom <= 11 and rk in ("rail", "subway", "tram"):
                        fw = _RW_WIDTH_LOW.get((zoom, rk), default_fw)
                        draw.line(pts, fill=_mix(fc), width=fw, joint="curve")
                    elif zoom <= 10 and wk in ("river", "canal"):
                        fw = _WW_WIDTH_LOW.get((zoom, wk), default_fw)
                        draw.line(pts, fill=_mix(fc), width=fw, joint="curve")
                    else:
                        if zoom >= 11 and cc:
                            draw.line(pts, fill=_mix(cc), width=default_cw,
                                      joint="curve")
                        draw.line(pts, fill=_mix(fc), width=default_fw,
                                  joint="curve")

            if render_pois and pois:
                for _, pt, pc in pois:
                    if not _poi_zoom_ok(pt, zoom): continue
                    am = pt.get("amenity", "")
                    rk = pt.get("railway", "")
                    ak = pt.get("aeroway", "")
                    for nid, (px, py) in pc.items():
                        cx = px - x0; cy = py - y0
                        if zoom <= 12:
                            if am == "hospital":
                                _draw_small_icon(draw, cx, cy,
                                    _HOSP_F, _HOSP_O, _HOSP_COLOR)
                            elif am in ("school", "university"):
                                _draw_small_icon(draw, cx, cy,
                                    _SCHOOL_F, _SCHOOL_O, _SCHOOL_COLOR)
                            elif rk == "station":
                                _draw_small_icon(draw, cx, cy,
                                    _TRAIN_F, _TRAIN_O, _TRAIN_COLOR)
                            elif ak == "aerodrome":
                                _draw_small_icon(draw, cx, cy,
                                    _PLANE_F, _PLANE_O, _PLANE_COLOR)
                            continue
                        cat = _node_cat(pt)
                        ic, isz, _, ifc, isn, _ = _NS.get(cat, _NS["default"])
                        hs = isz / 2
                        draw.ellipse([cx - hs, cy - hs, cx + hs, cy + hs],
                                     fill=ic)
                        nm = pt.get("name", "")
                        if isn and nm:
                            ft = _font(9)
                            bb = draw.textbbox((0, 0), nm, font=ft)
                            draw.text((cx - (bb[2] - bb[0]) / 2, cy + hs + 2),
                                      nm, fill=ifc, font=ft)

            if render_labels:
                rendered_names = set()

                if zoom >= 13:
                    all_it = lines + areas + pois if render_areas else lines
                    for _, wt, wc in all_it:
                        if not _label_zoom_ok(wt, zoom): continue
                        nm = wt.get("name", "")
                        if not nm or nm in rendered_names: continue
                        rendered_names.add(nm)
                        pts = [(px - x0, py - y0)
                               for _, (px, py) in wc.items()]
                        if len(pts) < 2: continue
                        mx, my = pts[len(pts) // 2]
                        fsz = _label_font_size(wt, zoom)
                        _draw_text(draw, mx - _font(fsz).getlength(nm) / 2,
                                   my - fsz, nm, fsz)

                if zoom == 6:
                    for nid, (px, py) in npc.items():
                        cx = px - x0; cy = py - y0
                        if nid in province_names:
                            prov_nm = province_names[nid]
                            if prov_nm not in rendered_names:
                                rendered_names.add(prov_nm)
                                fsz = 14
                                bb = draw.textbbox((0, 0), prov_nm,
                                                   font=_font(fsz))
                                _draw_text(draw,
                                    cx - (bb[2] - bb[0]) / 2,
                                    cy - fsz, prov_nm, fsz)
                        elif (nid in admin_centres
                              and "5" in admin_centres[nid]):
                            r = 2
                            draw.ellipse(
                                [cx - r, cy - r, cx + r, cy + r],
                                fill=(204, 68, 68),
                                outline=(255, 255, 255), width=1)

                elif zoom == 7:
                    for nid, (px, py) in npc.items():
                        cx = px - x0; cy = py - y0
                        if nid in admin_centres:
                            ac = admin_centres[nid]
                            city_nm = ac.get("5") or ""
                            if city_nm and city_nm not in rendered_names:
                                rendered_names.add(city_nm)
                                fsz = 14
                                bb = draw.textbbox((0, 0), city_nm,
                                                   font=_font(fsz))
                                _draw_text(draw,
                                    cx - (bb[2] - bb[0]) / 2,
                                    cy - fsz, city_nm, fsz)

                elif zoom <= 12:
                    s5, s6 = _AC_FONT.get(zoom, (10, 9))
                    for nid, (px, py) in npc.items():
                        cx = px - x0; cy = py - y0
                        if nid not in admin_centres:
                            continue
                        ac = admin_centres[nid]
                        if "5" in ac:
                            nm5 = ac["5"]
                            if nm5 not in rendered_names:
                                rendered_names.add(nm5)
                                ft = _font(s5)
                                bb = draw.textbbox((0, 0), nm5, font=ft)
                                _draw_text(draw,
                                    cx - (bb[2] - bb[0]) / 2,
                                    cy - s5, nm5, s5)
                        if "6" in ac:
                            nm6 = ac["6"]
                            if nm6 not in rendered_names:
                                rendered_names.add(nm6)
                                ft = _font(s6)
                                bb = draw.textbbox((0, 0), nm6, font=ft)
                                _draw_text(draw,
                                    cx - (bb[2] - bb[0]) / 2,
                                    cy - s6, nm6, s6)

                elif zoom >= 13:
                    for nid, (px, py) in npc.items():
                        cx = px - x0; cy = py - y0
                        if nid in place_nodes:
                            p_nm, p_type = place_nodes[nid]
                            tags = {"place": p_type, "name": p_nm}
                            if (_label_zoom_ok(tags, zoom)
                                    and p_nm not in rendered_names):
                                rendered_names.add(p_nm)
                                fsz = _label_font_size(tags, zoom)
                                ft = _font(fsz)
                                bb = draw.textbbox((0, 0), p_nm, font=ft)
                                _draw_text(draw,
                                    cx - (bb[2] - bb[0]) / 2,
                                    cy - fsz, p_nm, fsz)
                        elif (nid in admin_centres
                              and "6" in admin_centres[nid]):
                            nm6 = admin_centres[nid]["6"]
                            if nm6 not in rendered_names:
                                rendered_names.add(nm6)
                                fsz = 9
                                ft = _font(fsz)
                                bb = draw.textbbox((0, 0), nm6, font=ft)
                                _draw_text(draw,
                                    cx - (bb[2] - bb[0]) / 2,
                                    cy - fsz, nm6, fsz)

            gx = global_tx0 + tile_x
            gy = global_ty0 + tile_y
            td = os.path.join(output_dir, str(zoom), str(gx))
            os.makedirs(td, exist_ok=True)
            ext = "jpg" if is_jpg else tile_format.lower()
            img.save(os.path.join(td, f"{gy}.{ext}"), **save_kw)
            rendered += 1

            now = time.time()
            if now - tp >= progress_interval:
                spd = rendered / (now - t0) if now > t0 else 0
                rem = (len(occ) - rendered) / spd if spd > 0 else 0
                print(f"\r  {rendered}/{len(occ)}  "
                      f"{spd:.1f} tiles/sec  remaining {rem:.0f}s ",
                      end="", flush=True)
                tp = now

    print(f"\n  Rendering complete: {rendered} tiles, skipped {skipped}, "
          f"elapsed {time.time() - t0:.1f}s", flush=True)
    return output_dir


# ════════════════════════════════════════════════════════════════
# §8  Staged Rendering + Main Program
# ════════════════════════════════════════════════════════════════

def _osmium_extract(input_file, bbox, output_file):
    import subprocess
    s, w, n, e = bbox
    try:
        r = subprocess.run(
            ["osmium", "extract", "-b", f"{w},{s},{e},{n}",
             input_file, "-o", output_file, "--overwrite"],
            capture_output=True, text=True, timeout=600)
        return r.returncode == 0 and os.path.exists(output_file)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _get_pbf_header_bbox(filepath):
    """Get geographic bounding box from PBF file header (south, west, north, east)"""
    try:
        with open(filepath, 'rb') as f:
            data, btype = OSMParser._read_blob(f)
            if data is None:
                return None
            hdr = osmpbf.HeaderBlock()
            hdr.ParseFromString(data)
            if hdr.HasField('bbox'):
                bb = hdr.bbox
                return (1e-9 * bb.bottom, 1e-9 * bb.left,
                        1e-9 * bb.top, 1e-9 * bb.right)
    except Exception:
        pass
    return None


def _split_bbox(bbox, buf_deg=0.3, target_chunk_deg=15):
    """Adaptively split bbox:
    - Small extent (<10 degrees): no splitting
    - Medium/large extent: dynamically compute nx/ny based on target_chunk_deg
    Returns [(south, west, north, east), ...] list, each chunk with buf overlap
    """
    s, w, n, e = bbox
    width = e - w
    height = n - s

    if width < 10 and height < 10:
        return [bbox]

    nx = max(1, round(width / target_chunk_deg))
    ny = max(1, round(height / (target_chunk_deg * 0.87)))

    total = nx * ny
    if total > 20:
        scale = (total / 20) ** 0.5
        nx = max(1, round(nx / scale))
        ny = max(1, round(ny / scale))
        total = nx * ny

    if total <= 1:
        return [bbox]

    lat_step = height / ny
    lon_step = width / nx
    buf_lat = min(buf_deg, lat_step * 0.2)
    buf_lon = min(buf_deg, lon_step * 0.2)

    chunks = []
    for i in range(ny):
        for j in range(nx):
            cs = max(s + i * lat_step - buf_lat, s)
            cn = min(s + (i + 1) * lat_step + buf_lat, n)
            cw = max(w + j * lon_step - buf_lon, w)
            ce = min(w + (j + 1) * lon_step + buf_lon, e)
            chunks.append((cs, cw, cn, ce))
    return chunks



def _collect_needed_ids(filepath, max_zoom, bbox=None):
    """First pass scan: collect node ID set needed within the specified max_zoom range"""
    def way_filter(tags):
        return _way_zoom_ok(tags, max_zoom)

    scanner = OSMParser(way_filter=way_filter, bbox=bbox)
    _, ways, relations = scanner.parse(filepath, ways_only=True)

    # Nodes referenced by ways
    way_ids = set()
    skipped_small = 0
    for _, tags, nids in ways:
        if max_zoom <= 8 and is_area(tags) and len(nids) < 50:
            skipped_small += 1
            continue
        way_ids.update(nids)
    if skipped_small:
        print(f"  Skipped small area features: {skipped_small:,}", flush=True)

    # Admin centre nodes (extracted from relations)
    ac_ids = set()
    for _, tags, members in relations:
        if (tags.get("type") == "boundary"
                and tags.get("boundary") == "administrative"):
            al = tags.get("admin_level", "")
            if al in ("4", "5", "6"):
                for m in members:
                    if (m["role"] in ("admin_centre", "admin_center", "label")
                            and m["type"] == "node"):
                        ac_ids.add(m["ref"])

    needed = way_ids | ac_ids
    print(f"  Way nodes: {len(way_ids):,}, admin centres: {len(ac_ids):,}, "
          f"total: {len(needed):,}", flush=True)
    del way_ids, ac_ids
    needed_arr = np.fromiter(needed, dtype=np.int64)
    needed_arr.sort()
    print(f"  numpy: {needed_arr.nbytes / 1024 / 1024:.0f} MB "
          f"(set approx. {len(needed) * 40 // 1024 // 1024} MB)", flush=True)
    del needed, ways, relations
    return needed_arr


def _process_stage_two_pass(filepath, zooms, max_zoom, output_dir, args,
                             bbox=None, stage_label=""):
    """Two-pass stage: scan → load (filtered) → render → release"""
    print(f"\n{'=' * 60}", flush=True)
    print(f"  Stage [{stage_label}]  (two-pass mode)", flush=True)
    print(f"  zoom: {', '.join(str(z) for z in zooms)}, max_zoom={max_zoom}",
          flush=True)
    if bbox:
        print(f"  bbox: S={bbox[0]:.2f} W={bbox[1]:.2f} "
              f"N={bbox[2]:.2f} E={bbox[3]:.2f}", flush=True)
    print(f"{'=' * 60}", flush=True)

    # ── First pass: scan ──
    print(f"\n--- First pass: scanning way references ---", flush=True)
    t0 = time.time()
    needed_ids = _collect_needed_ids(filepath, max_zoom, bbox)
    print(f"  Elapsed: {time.time() - t0:.1f}s", flush=True)

    # ── Second pass: load nodes ──
    def way_filter(tags):
        return _way_zoom_ok(tags, max_zoom)

    print(f"\n--- Second pass: loading nodes (strict filter) ---", flush=True)
    t0 = time.time()
    parser = OSMParser(way_filter=way_filter, bbox=bbox, strict_filter=True)
    nodes, ways, relations = parser.parse(filepath, node_id_filter=needed_ids)
    del needed_ids
    print(f"  Elapsed: {time.time() - t0:.1f}s", flush=True)

    # ── Build admin centres ──
    province_names, admin_centres = build_admin_centres(relations)
    place_nodes = parser.place_nodes

    ac_levels = {}
    for nid, levels in admin_centres.items():
        for lv in levels:
            ac_levels[lv] = ac_levels.get(lv, 0) + 1
    ac_summary = ", ".join(f"level {k}: {v}"
                           for k, v in sorted(ac_levels.items()))

    print(f"  Data: nodes {len(nodes):,}, ways {len(ways):,}, "
          f"admin centres {len(admin_centres)} ({ac_summary}), "
          f"provinces {len(province_names)}", flush=True)

    # ── Render ──
    for z in zooms:
        print(f"\n--- Rendering zoom {z} ---", flush=True)
        render_region(nodes, ways, relations, zoom=z,
            tile_size=args.tile_size, tile_format=args.format,
            output_dir=output_dir, render_areas=not args.no_areas,
            render_ways=not args.no_ways, render_pois=not args.no_pois,
            render_labels=not args.no_labels,
            place_nodes=place_nodes, admin_centres=admin_centres,
            province_names=province_names)

    # ── Release memory ──
    n_count = len(nodes)
    del nodes, ways, relations, admin_centres, province_names, place_nodes
    gc.collect()
    print(f"  ✓ Stage complete, released {n_count:,} nodes from memory", flush=True)


def _process_stage_chunk(filepath, zooms, max_zoom, output_dir, args,
                          bbox=None, stage_label=""):
    """Single-pass stage (for small chunks after bbox splitting): full load → render → release"""
    def way_filter(tags):
        return _way_zoom_ok(tags, max_zoom)

    print(f"\n{'=' * 60}", flush=True)
    print(f"  Stage [{stage_label}]  (single-pass mode)", flush=True)
    print(f"  zoom: {', '.join(str(z) for z in zooms)}", flush=True)
    if bbox:
        print(f"  bbox: S={bbox[0]:.2f} W={bbox[1]:.2f} "
              f"N={bbox[2]:.2f} E={bbox[3]:.2f}", flush=True)
    print(f"{'=' * 60}", flush=True)

    t0 = time.time()
    parser = OSMParser(way_filter=way_filter, bbox=bbox)
    nodes, ways, relations = parser.parse(filepath)
    print(f"  Load elapsed: {time.time() - t0:.1f}s", flush=True)

    province_names, admin_centres = build_admin_centres(relations)
    place_nodes = parser.place_nodes

    ac_levels = {}
    for nid, levels in admin_centres.items():
        for lv in levels:
            ac_levels[lv] = ac_levels.get(lv, 0) + 1
    ac_summary = ", ".join(f"level {k}: {v}"
                           for k, v in sorted(ac_levels.items()))

    print(f"  Nodes {len(nodes):,}, ways {len(ways):,}, "
          f"admin centres {len(admin_centres)} ({ac_summary}), "
          f"provinces {len(province_names)}", flush=True)

    for z in zooms:
        print(f"\n--- Rendering zoom {z} ---", flush=True)
        render_region(nodes, ways, relations, zoom=z,
            tile_size=args.tile_size, tile_format=args.format,
            output_dir=output_dir, render_areas=not args.no_areas,
            render_ways=not args.no_ways, render_pois=not args.no_pois,
            render_labels=not args.no_labels,
            place_nodes=place_nodes, admin_centres=admin_centres,
            province_names=province_names)

    n_count = len(nodes)
    del nodes, ways, relations, admin_centres, province_names, place_nodes
    gc.collect()
    print(f"  ✓ Chunk complete, released {n_count:,} nodes from memory", flush=True)


def _get_pbf_header_bbox(filepath):
    """Get geographic bounding box from PBF file header (south, west, north, east)"""
    try:
        with open(filepath, 'rb') as f:
            data, btype = OSMParser._read_blob(f)
            if data is None:
                return None
            hdr = osmpbf.HeaderBlock()
            hdr.ParseFromString(data)
            if hdr.HasField('bbox'):
                bb = hdr.bbox
                return (1e-9 * bb.bottom, 1e-9 * bb.left,
                        1e-9 * bb.top, 1e-9 * bb.right)
    except Exception:
        pass
    return None


def _split_bbox(bbox, nx=4, ny=3, buf=0.3):
    """Split bbox into nx*ny sub-chunks, each extended outward by buf degrees of overlap"""
    s, w, n, e = bbox
    lat_step = (n - s) / ny
    lon_step = (e - w) / nx
    chunks = []
    for i in range(ny):
        for j in range(nx):
            cs = max(s + i * lat_step - buf, s)
            cn = min(s + (i + 1) * lat_step + buf, n)
            cw = max(w + j * lon_step - buf, w)
            ce = min(w + (j + 1) * lon_step + buf, e)
            chunks.append((cs, cw, cn, ce))
    return chunks


def _collect_needed_ids(filepath, max_zoom, bbox=None):
    """First pass scan: collect node ID set needed within the specified max_zoom range"""
    def way_filter(tags):
        return _way_zoom_ok(tags, max_zoom)

    scanner = OSMParser(way_filter=way_filter, bbox=bbox)
    _, ways, relations = scanner.parse(filepath, ways_only=True)

    way_ids = set()
    skipped_small = 0
    for _, tags, nids in ways:
        if max_zoom <= 8 and is_area(tags) and len(nids) < 50:
            skipped_small += 1
            continue
        way_ids.update(nids)
    if skipped_small:
        print(f"  Skipped small area features: {skipped_small:,}", flush=True)

    ac_ids = set()
    for _, tags, members in relations:
        if (tags.get("type") == "boundary"
                and tags.get("boundary") == "administrative"):
            al = tags.get("admin_level", "")
            if al in ("4", "5", "6"):
                for m in members:
                    if (m["role"] in ("admin_centre", "admin_center", "label")
                            and m["type"] == "node"):
                        ac_ids.add(m["ref"])

    needed = way_ids | ac_ids
    print(f"  Way nodes: {len(way_ids):,}, admin centres: {len(ac_ids):,}, "
          f"total: {len(needed):,}", flush=True)
    del way_ids, ac_ids, ways, relations
    return needed


def _process_stage_two_pass(filepath, zooms, max_zoom, output_dir, args,
                             bbox=None, stage_label=""):
    """Two-pass stage: scan → load (filtered) → render → release"""
    print(f"\n{'=' * 60}", flush=True)
    print(f"  Stage [{stage_label}]  (two-pass mode)", flush=True)
    print(f"  zoom: {', '.join(str(z) for z in zooms)}, max_zoom={max_zoom}",
          flush=True)
    if bbox:
        print(f"  bbox: S={bbox[0]:.2f} W={bbox[1]:.2f} "
              f"N={bbox[2]:.2f} E={bbox[3]:.2f}", flush=True)
    print(f"{'=' * 60}", flush=True)

    print(f"\n--- First pass: scanning way references ---", flush=True)
    t0 = time.time()
    needed_ids = _collect_needed_ids(filepath, max_zoom, bbox)
    print(f"  Elapsed: {time.time() - t0:.1f}s", flush=True)

    def way_filter(tags):
        return _way_zoom_ok(tags, max_zoom)

    print(f"\n--- Second pass: loading nodes (strict filter) ---", flush=True)
    t0 = time.time()
    parser = OSMParser(way_filter=way_filter, bbox=bbox, strict_filter=True)
    nodes, ways, relations = parser.parse(filepath, node_id_filter=needed_ids)
    del needed_ids
    print(f"  Elapsed: {time.time() - t0:.1f}s", flush=True)

    province_names, admin_centres = build_admin_centres(relations)
    place_nodes = parser.place_nodes

    ac_levels = {}
    for nid, levels in admin_centres.items():
        for lv in levels:
            ac_levels[lv] = ac_levels.get(lv, 0) + 1
    ac_summary = ", ".join(f"level {k}: {v}"
                           for k, v in sorted(ac_levels.items()))

    print(f"  Data: nodes {len(nodes):,}, ways {len(ways):,}, "
          f"admin centres {len(admin_centres)} ({ac_summary}), "
          f"provinces {len(province_names)}", flush=True)

    for z in zooms:
        print(f"\n--- Rendering zoom {z} ---", flush=True)
        render_region(nodes, ways, relations, zoom=z,
            tile_size=args.tile_size, tile_format=args.format,
            output_dir=output_dir, render_areas=not args.no_areas,
            render_ways=not args.no_ways, render_pois=not args.no_pois,
            render_labels=not args.no_labels,
            place_nodes=place_nodes, admin_centres=admin_centres,
            province_names=province_names)

    n_count = len(nodes)
    del nodes, ways, relations, admin_centres, province_names, place_nodes
    gc.collect()
    print(f"  Stage complete, released {n_count:,} nodes from memory", flush=True)


def _process_stage_chunk(filepath, zooms, max_zoom, output_dir, args,
                          bbox=None, stage_label=""):
    """Single-pass stage (chunks after bbox splitting): full load → render → release"""
    def way_filter(tags):
        return _way_zoom_ok(tags, max_zoom)

    print(f"\n{'=' * 60}", flush=True)
    print(f"  Stage [{stage_label}]  (single-pass mode)", flush=True)
    print(f"  zoom: {', '.join(str(z) for z in zooms)}", flush=True)
    if bbox:
        print(f"  bbox: S={bbox[0]:.2f} W={bbox[1]:.2f} "
              f"N={bbox[2]:.2f} E={bbox[3]:.2f}", flush=True)
    print(f"{'=' * 60}", flush=True)

    t0 = time.time()
    parser = OSMParser(way_filter=way_filter, bbox=bbox)
    nodes, ways, relations = parser.parse(filepath)
    print(f"  Load elapsed: {time.time() - t0:.1f}s", flush=True)

    province_names, admin_centres = build_admin_centres(relations)
    place_nodes = parser.place_nodes

    ac_levels = {}
    for nid, levels in admin_centres.items():
        for lv in levels:
            ac_levels[lv] = ac_levels.get(lv, 0) + 1
    ac_summary = ", ".join(f"level {k}: {v}"
                           for k, v in sorted(ac_levels.items()))

    print(f"  Nodes {len(nodes):,}, ways {len(ways):,}, "
          f"admin centres {len(admin_centres)} ({ac_summary}), "
          f"provinces {len(province_names)}", flush=True)

    for z in zooms:
        print(f"\n--- Rendering zoom {z} ---", flush=True)
        render_region(nodes, ways, relations, zoom=z,
            tile_size=args.tile_size, tile_format=args.format,
            output_dir=output_dir, render_areas=not args.no_areas,
            render_ways=not args.no_ways, render_pois=not args.no_pois,
            render_labels=not args.no_labels,
            place_nodes=place_nodes, admin_centres=admin_centres,
            province_names=province_names)

    n_count = len(nodes)
    del nodes, ways, relations, admin_centres, province_names, place_nodes
    gc.collect()
    print(f"  Chunk complete, released {n_count:,} nodes from memory", flush=True)


def main():
    ap = argparse.ArgumentParser(description="OSM PBF → Tile Map")
    ap.add_argument("input", help="Input .osm.pbf file")
    ap.add_argument("-z", "--zoom", default=None,
                    help="Zoom level (e.g. 14 or 10-12); interactive input if not specified")
    ap.add_argument("-b", "--bbox", default=None,
                    help="Bounding box: south,west,north,east")
    ap.add_argument("-o", "--output", default="gpsmap", help="Output directory")
    ap.add_argument("--tile-size", type=int, default=256)
    ap.add_argument("-f", "--format", default="JPG",
                    help="Output format: JPG or PNG")
    ap.add_argument("--no-areas", action="store_true")
    ap.add_argument("--no-ways", action="store_true")
    ap.add_argument("--no-pois", action="store_true")
    ap.add_argument("--no-labels", action="store_true")
    ap.add_argument("--single-pass", action="store_true",
                    help="Single-pass mode (for small files)")
    args = ap.parse_args()

    fp = args.input
    if not os.path.exists(fp):
        print(f"File not found: {fp}")
        sys.exit(1)

    if args.zoom is not None:
        z_str = str(args.zoom)
    else:
        z_input = input("Enter starting zoom level [default 10]: ").strip()
        start_z = int(z_input) if z_input else 10
        z_input = input("Enter ending zoom level [default 12]: ").strip()
        end_z = int(z_input) if z_input else 12
        if start_z > end_z:
            start_z, end_z = end_z, start_z
        z_str = f"{start_z}-{end_z}"

    if "-" in z_str:
        a, b = z_str.split("-")
        zooms = list(range(int(a), int(b) + 1))
    else:
        zooms = [int(z_str)]

    print(f"Will render zoom: {', '.join(str(z) for z in zooms)}", flush=True)

    bbox = None
    if args.bbox:
        parts = [float(x.strip()) for x in args.bbox.split(",")]
        if len(parts) == 4:
            bbox = tuple(parts)
            print(f"Bbox: S={bbox[0]} W={bbox[1]} N={bbox[2]} E={bbox[3]}",
                  flush=True)

    os.makedirs(args.output, exist_ok=True)
    fsz = os.path.getsize(fp)

    # ── Small file / single-pass mode: keep original behavior ──
    if args.single_pass or fsz < 200 * 1024 * 1024:
        print(f"\nSmall file / single-pass mode ({fsz / 1024 / 1024:.0f} MB)", flush=True)
        way_filter = lambda t: True
        actual_input = fp
        temp_file = None

        if bbox:
            temp_file = os.path.join(args.output, "_extracted.pbf")
            if _osmium_extract(fp, bbox, temp_file):
                tsz = os.path.getsize(temp_file)
                print(f"osmium extract succeeded: {tsz / 1024 / 1024:.0f} MB "
                      f"(original {fsz / 1024 / 1024:.0f} MB)", flush=True)
                actual_input = temp_file
                bbox = None
            else:
                print("osmium not available, using Python bbox parsing...", flush=True)

        parser = OSMParser(way_filter=way_filter, bbox=bbox)
        nodes, ways, relations = parser.parse(actual_input)
        province_names, admin_centres = build_admin_centres(relations)
        place_nodes = parser.place_nodes

        ac_levels = {}
        for nid, levels in admin_centres.items():
            for lv in levels:
                ac_levels[lv] = ac_levels.get(lv, 0) + 1
        ac_summary = ", ".join(f"level {k}: {v}"
                               for k, v in sorted(ac_levels.items()))

        print(f"\nData: nodes {len(nodes):,}, ways {len(ways):,}, "
              f"relations {len(relations):,}, place nodes {len(place_nodes):,}, "
              f"admin centres {len(admin_centres)} ({ac_summary}), "
              f"provinces {len(province_names)}", flush=True)

        for z in zooms:
            print(f"\n{'=' * 50}", flush=True)
            print(f"  Rendering zoom {z}", flush=True)
            print(f"{'=' * 50}", flush=True)
            render_region(nodes, ways, relations, zoom=z,
                tile_size=args.tile_size, tile_format=args.format,
                output_dir=args.output, render_areas=not args.no_areas,
                render_ways=not args.no_ways, render_pois=not args.no_pois,
                render_labels=not args.no_labels,
                place_nodes=place_nodes, admin_centres=admin_centres,
                province_names=province_names)

        if temp_file and os.path.exists(temp_file):
            os.remove(temp_file)
        print("\nAll done!", flush=True)
        return

    # ═══════════════════════════════════════════════════
    # Large file staged rendering
    # ═══════════════════════════════════════════════════
    print(f"\nLarge file staged mode ({fsz / 1024 / 1024:.0f} MB)", flush=True)

    file_bbox = None
    if bbox:
        file_bbox = bbox
    else:
        file_bbox = _get_pbf_header_bbox(fp)
        if file_bbox:
            print(f"PBF header bbox: S={file_bbox[0]:.2f} "
                  f"W={file_bbox[1]:.2f} N={file_bbox[2]:.2f} "
                  f"E={file_bbox[3]:.2f}", flush=True)
        else:
            print("Unable to get file bbox, z13-14 will use full loading", flush=True)

    low_zooms = [z for z in zooms if z <= 8]
    mid_low_zooms = [z for z in zooms if 9 <= z <= 10]
    mid_high_zooms = [z for z in zooms if 11 <= z <= 12]
    high_zooms = [z for z in zooms if z >= 13]

    stage_idx = 0

    if low_zooms:
        stage_idx += 1
        _process_stage_two_pass(
            fp, low_zooms, max(low_zooms), args.output, args,
            bbox=bbox,
            stage_label=f"{stage_idx}: z{min(low_zooms)}-{max(low_zooms)}")

    if mid_low_zooms:
        stage_idx += 1
        _process_stage_two_pass(
            fp, mid_low_zooms, max(mid_low_zooms), args.output, args,
            bbox=bbox,
            stage_label=f"{stage_idx}: z{min(mid_low_zooms)}-{max(mid_low_zooms)}")

    if mid_high_zooms:
        stage_idx += 1
        _process_stage_two_pass(
            fp, mid_high_zooms, max(mid_high_zooms), args.output, args,
            bbox=bbox,
            stage_label=f"{stage_idx}: z{min(mid_high_zooms)}-{max(mid_high_zooms)}")

    if high_zooms:
        stage_idx += 1
        if file_bbox:
            chunks = _split_bbox(file_bbox)
            if len(chunks) == 1:
                print(f"\nz{min(high_zooms)}-{max(high_zooms)} "
                      f"extent is small, no splitting needed", flush=True)
                _process_stage_two_pass(
                    fp, high_zooms, max(high_zooms), args.output, args,
                    bbox=bbox,
                    stage_label=f"{stage_idx}: "
                                f"z{min(high_zooms)}-{max(high_zooms)}")
            else:
                print(f"\nz{min(high_zooms)}-{max(high_zooms)} "
                      f"geographic split: {len(chunks)} chunks", flush=True)
                for ci, chunk_bbox in enumerate(chunks):
                    chunk_label = (
                        f"{stage_idx}: "
                        f"z{min(high_zooms)}-{max(high_zooms)} "
                        f"chunk {ci + 1}/{len(chunks)}")
                    _process_stage_chunk(
                        fp, high_zooms, max(high_zooms), args.output, args,
                        bbox=chunk_bbox, stage_label=chunk_label)
        else:
            _process_stage_two_pass(
                fp, high_zooms, max(high_zooms), args.output, args,
                bbox=None,
                stage_label=f"{stage_idx}: "
                            f"z{min(high_zooms)}-{max(high_zooms)}")


    print("\n" + "=" * 60, flush=True)
    print("  All stages complete!", flush=True)
    print("=" * 60, flush=True)


if __name__ == "__main__":
    main()
