# app_fast.py
#
# Init:
#   Configure mosaic-h
#   Configure Raspberry pi to give internet access to mosaic-h
#   make source venv
#   install requirements (may differ based on Raspberry pi OS and availabe packages)
#   run "source /venv/location"
# Run:
#   python -m uvicorn app_fast:app --host 0.0.0.0 --port 8002
#
# Notes:
# - Contains partial parts of depriciated code laptop to raspberry pi
# - GNSS_GGA_PORT should output GGA (and optionally RMC/VTG; we can compute speed from GGA deltas too)
# - GNSS_HDT_PORT should output HDT (true heading)
# - This app can:
#   * draw your GeoJSON geometry (viewport + load all)
#   * compute nearest distance to geometry for current and predicted point
#   * show predicted point (RED) + predicted connector (RED dashed)
#   * drive hardware LEDs/buzzer with a deadzone and 3 "middle LEDs"
#
# IMPORTANT:
# - The “side” decision is based on: errM = (distance_to_geometry - IMAGINARY_OFFSET_M)
#   That means “too close” vs “too far” relative to a target offset, not true left/right-of-line.

import os, time, json, math, base64, socket, ssl, threading, queue
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple

import serial
from fastapi import FastAPI, WebSocket, Query, Response
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from pyproj import Transformer

# =======================
# CONFIG
# =======================
GNSS_GGA_PORT = os.getenv("GNSS_GGA_PORT", "/dev/ttyACM0")  # USB1 (GGA)
GNSS_HDT_PORT = os.getenv("GNSS_HDT_PORT", "/dev/ttyACM1")  # USB2 (HDT)
BAUD          = int(os.getenv("GNSS_BAUD", "115200"))

# Optional internal NTRIP (mosaic-h handles rtk)
NTRIP_HOST  = os.getenv("NTRIP_HOST", "")
NTRIP_PORT  = int(os.getenv("NTRIP_PORT", "8083"))
NTRIP_MOUNT = os.getenv("NTRIP_MOUNT", "")
NTRIP_USER  = os.getenv("NTRIP_USER", "")
NTRIP_PASS  = os.getenv("NTRIP_PASS", "")
NTRIP_TLS   = bool(int(os.getenv("NTRIP_TLS", "0")))
NTRIP_GGA_INTERVAL = float(os.getenv("NTRIP_GGA_INTERVAL", "0.05"))

# Prediction params, 0.8 seconds when used from cars roof
PREDICT_T_SEC        = float(os.getenv("PREDICT_T_SEC", "0.8"))
HEADING_STALE_SEC    = float(os.getenv("HEADING_STALE_SEC", "1.0"))
SPEED_STALE_SEC      = float(os.getenv("SPEED_STALE_SEC", "1.5"))

# Speed-from-GGA (when RMC/VTG not available)
SPEED_FROM_GGA_ENABLED = bool(int(os.getenv("SPEED_FROM_GGA_ENABLED", "1")))
SPEED_MIN_DT_SEC       = float(os.getenv("SPEED_MIN_DT_SEC", "0.20"))
SPEED_MAX_DT_SEC       = float(os.getenv("SPEED_MAX_DT_SEC", "2.00"))
SPEED_SMOOTH_ALPHA     = float(os.getenv("SPEED_SMOOTH_ALPHA", "0.35"))
SPEED_MAX_MPS          = float(os.getenv("SPEED_MAX_MPS", "25.0"))

# Recording for points
RECORD_DIR = Path(os.getenv("RECORD_DIR", "recordings"))
POINTS_SAVE_HZ = float(os.getenv("POINTS_SAVE_HZ", "10"))
WRITER_FLUSH_SEC = float(os.getenv("WRITER_FLUSH_SEC", "0.5"))

# UI push
BROADCAST_HZ = float(os.getenv("BROADCAST_HZ", "5"))
TRAIL_MAX = int(os.getenv("TRAIL_MAX", "400"))

# Geo with chosen modified map
GEO_PATH = Path(os.getenv("GEO_PATH", "static/25cm3301.geojson"))
GEO_EPSG = int(os.getenv("GEO_EPSG", "3301"))
NEAR_SCAN_RADIUS_M = float(os.getenv("NEAR_SCAN_RADIUS_M", "50"))

# Browser audio worklet guidance (distance-only beep)
AUDIO_ENABLED      = bool(int(os.getenv("AUDIO_ENABLED", "1")))
IMAGINARY_OFFSET_M = float(os.getenv("IMAGINARY_OFFSET_M", "1.5"))
A_MAX_DIST_M       = float(os.getenv("A_MAX_DIST_M", "2.50"))
B_CONST_TONE_M     = float(os.getenv("B_CONST_TONE_M", "0.50"))
A_RATE_MIN_HZ      = float(os.getenv("A_RATE_MIN_HZ", "2"))
A_RATE_MAX_HZ      = float(os.getenv("A_RATE_MAX_HZ", "8.0"))
B_RATE_MIN_HZ      = float(os.getenv("B_RATE_MIN_HZ", "2"))
B_RATE_MAX_HZ      = float(os.getenv("B_RATE_MAX_HZ", "20.0"))
TONE_A_FREQ        = float(os.getenv("TONE_A_FREQ", "880"))
TONE_B_FREQ        = float(os.getenv("TONE_B_FREQ", "1560"))

# Hardware buzzer/LED (Pi)
HW_AUDIO_ENABLED   = bool(int(os.getenv("HW_AUDIO_ENABLED", "1")))
HW_BUZZER_PIN      = int(os.getenv("HW_BUZZER_PIN", "18"))
HW_LEFT_LED_PIN    = int(os.getenv("HW_LEFT_LED_PIN", "17"))
HW_RIGHT_LED_PIN   = int(os.getenv("HW_RIGHT_LED_PIN", "27"))
HW_VOLUME          = float(os.getenv("HW_VOLUME", "0.90"))
HW_GATE_ON_SEC     = float(os.getenv("HW_GATE_ON_SEC", "0.06"))
HW_LOOP_DT         = float(os.getenv("HW_LOOP_DT", "0.01"))

# 3 middle LEDs (on Raspberry)
MID_LEFT_LED_PIN   = int(os.getenv("MID_LEFT_LED_PIN", "6"))    # left-most middle LED
MID_CENTER_LED_PIN = int(os.getenv("MID_CENTER_LED_PIN", "16")) # center middle LED
MID_RIGHT_LED_PIN  = int(os.getenv("MID_RIGHT_LED_PIN", "5"))   # right-most middle LED

# deadzone thresholds (meters)
CENTER_ALL_M       = float(os.getenv("CENTER_ALL_M", "0.08"))  # <= 8cm: all 3 mid LEDs
CENTER_DEADZONE_M  = float(os.getenv("CENTER_DEADZONE_M", "0.24"))  # <= 24cm: mid side LED only, no sound

# Startup tone at app start (hardware) test to make sure buzzer works
STARTUP_TONE_SEC   = float(os.getenv("STARTUP_TONE_SEC", "2.0"))
STARTUP_TONE_FREQ  = float(os.getenv("STARTUP_TONE_FREQ", "900.0"))
STARTUP_TONE_VOL   = float(os.getenv("STARTUP_TONE_VOL", "0.35"))

# Geo viewport API tuning
GEO_API_MAX_LINES     = int(os.getenv("GEO_API_MAX_LINES", "800"))
GEO_SIMPLIFY_STRIDE   = int(os.getenv("GEO_SIMPLIFY_STRIDE", "3"))
GEO_API_ENABLE        = bool(int(os.getenv("GEO_API_ENABLE", "1")))

# =======================
# App + globals
# =======================
app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")
clients = set()

latest_lock = threading.Lock()
latest: Dict[str, Any] = {}

record_lock = threading.Lock()
record_on = False
record_file: Optional[Path] = None
record_fh = None
record_count = 0

writer_lock = threading.Lock()
writer_buf: List[str] = []
writer_alive = True

rtk_lock = threading.Lock()
rtk_state = {
    "connected": False,
    "last_error": "",
    "bytes_in": 0,
    "bytes_out": 0,
    "last_gga": 0.0,
    "reconnects": 0,
    "using_tls": NTRIP_TLS,
}

guidance_lock = threading.Lock()
guidance_state = {"predictive": False}

# Geo cache
GEO_GEOMS: List[Dict[str, Any]] = []
ALL_GEO_FC: Dict[str, Any] = {"type":"FeatureCollection","features":[]}
_TRANSFORMER: Optional[Transformer] = None

# Hz counters
hz_lock = threading.Lock()
_hz = {
    "save": {"count": 0, "ts": time.time(), "hz": 0.0},
    "push": {"count": 0, "ts": time.time(), "hz": 0.0},
}
def _hz_mark(kind: str):
    now = time.time()
    with hz_lock:
        s = _hz[kind]
        s["count"] += 1
        dt = now - s["ts"]
        if dt >= 1.0:
            s["hz"] = s["count"] / dt if dt > 0 else 0.0
            s["count"] = 0
            s["ts"] = now
def _hz_snapshot() -> Tuple[float, float]:
    with hz_lock:
        return float(_hz["save"]["hz"]), float(_hz["push"]["hz"])

# Nearest work queue: (lat, lon, plat, plon)
nearest_q: "queue.Queue[Tuple[float,float,Optional[float],Optional[float]]]" = queue.Queue(maxsize=1)

def _now_ts() -> float:
    return time.time()
def _now_str() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())

# =======================
# Writer / Recording
# =======================
def _writer_thread():
    global writer_alive, record_fh, writer_buf, record_count
    last_flush = time.time()
    while writer_alive:
        time.sleep(0.05)
        now = time.time()
        do_flush = (now - last_flush) >= WRITER_FLUSH_SEC
        lines: List[str] = []
        with writer_lock:
            if writer_buf:
                lines, writer_buf = writer_buf, []
        if lines:
            with record_lock:
                if record_on and record_fh:
                    try:
                        record_fh.writelines(lines)
                        record_count += len(lines)
                        if do_flush:
                            record_fh.flush()
                    except Exception:
                        pass
        if do_flush:
            last_flush = now

def _record_dir() -> Path:
    RECORD_DIR.mkdir(parents=True, exist_ok=True)
    return RECORD_DIR

def _record_start():
    global record_on, record_file, record_fh, record_count
    with record_lock:
        if record_on and record_fh:
            return {"ok": True, "on": True, "file": str(record_file), "count": record_count}
        fpath = _record_dir() / f"rec_{_now_str()}.csv"
        fh = open(fpath, "a", encoding="utf-8", buffering=1, newline="\n")
        if fpath.stat().st_size == 0:
            fh.write("ts,lat,lon,heading_true,speed_mps,pred_t,pred_lat,pred_lon,dmin,pred_dmin\n")
        record_on = True
        record_file = fpath
        record_fh = fh
        record_count = 0
        return {"ok": True, "on": True, "file": str(fpath), "count": 0}

def _record_stop():
    global record_on, record_fh
    with record_lock:
        on = record_on
        fh = record_fh
        record_on = False
        record_fh = None
    if on and fh:
        try:
            fh.flush()
            fh.close()
        except Exception:
            pass
    return {"ok": True, "on": False, "file": str(record_file), "count": record_count}

def _record_status():
    with record_lock:
        save_hz, push_hz = _hz_snapshot()
        with guidance_lock:
            predictive = bool(guidance_state["predictive"])
        return {
            "ok": True,
            "on": record_on,
            "file": str(record_file) if record_file else "",
            "count": record_count,
            "saveHz": round(save_hz, 2),
            "pushHz": round(push_hz, 2),
            "predictive": predictive,
            "predictT": PREDICT_T_SEC,
        }

def _fmt_opt(x: Any, fmt: str) -> str:
    if x is None or not isinstance(x, (int, float)) or not math.isfinite(x):
        return ""
    return format(float(x), fmt)

def _enqueue_line(
    ts: float,
    lat: float,
    lon: float,
    heading_true: Optional[float],
    speed_mps: Optional[float],
    pred_t: float,
    pred_lat: Optional[float],
    pred_lon: Optional[float],
    dmin: Optional[float],
    pdmin: Optional[float],
):
    line = (
        f"{ts:.3f},{lat:.8f},{lon:.8f},"
        f"{_fmt_opt(heading_true, '.2f')},"
        f"{_fmt_opt(speed_mps, '.3f')},"
        f"{pred_t:.3f},"
        f"{_fmt_opt(pred_lat, '.8f')},"
        f"{_fmt_opt(pred_lon, '.8f')},"
        f"{_fmt_opt(dmin, '.3f')},"
        f"{_fmt_opt(pdmin, '.3f')}\n"
    )
    with writer_lock:
        writer_buf.append(line)

# =======================
# Geo / nearest helpers
# =======================
def _get_transformer():
    global _TRANSFORMER
    if _TRANSFORMER is None and GEO_EPSG != 4326:
        _TRANSFORMER = Transformer.from_crs(GEO_EPSG, 4326, always_xy=True)
    return _TRANSFORMER

def _transform_coords_to_wgs84(geom_type: str, coords):
    tr = _get_transformer()
    if tr is None:
        return coords
    def tx(x, y):
        lon, lat = tr.transform(x, y)
        return [lon, lat]
    def walk(t, c):
        if t == "Point":
            x, y = c; return tx(x, y)
        if t in ("MultiPoint", "LineString"):
            return [tx(x, y) for x, y in c]
        if t == "MultiLineString":
            return [[tx(x, y) for x, y in part] for part in c]
        if t == "Polygon":
            return [[tx(x, y) for x, y in ring] for ring in c]
        if t == "MultiPolygon":
            return [[[tx(x, y) for x, y in ring] for ring in poly] for poly in c]
        return c
    return walk(geom_type, coords)

def _bbox_of_coords(coords) -> Tuple[float, float, float, float]:
    mnx = mny = float("inf"); mxx = mxy = float("-inf")
    def walk(c):
        nonlocal mnx, mny, mxx, mxy
        if isinstance(c, list):
            if c and isinstance(c[0], (int, float)):
                x, y = c[0], c[1]
                mnx = min(mnx, x); mny = min(mny, y)
                mxx = max(mxx, x); mxy = max(mxy, y)
            else:
                for k in c: walk(k)
    walk(coords)
    if mnx == float("inf"): return (0, 0, 0, 0)
    return (mnx, mny, mxx, mxy)

def _explode_geometry(geom):
    t = geom.get("type"); c = geom.get("coordinates", [])
    if t == "MultiPolygon":
        for poly in c: yield {"type": "Polygon", "coordinates": poly}
    elif t == "MultiLineString":
        for ls in c: yield {"type": "LineString", "coordinates": ls}
    else:
        yield geom

def _to_latlon_line(coords, stride: int) -> Optional[List[List[float]]]:
    if not coords:
        return None
    if stride > 1:
        sliced = coords[::stride]
        if sliced[-1] != coords[-1]:
            sliced.append(coords[-1])
        coords = sliced
    ll = [[pt[1], pt[0]] for pt in coords]
    return ll if len(ll) >= 2 else None

def _preload_geoms():
    global GEO_GEOMS, ALL_GEO_FC
    GEO_GEOMS = []
    ALL_GEO_FC = {"type":"FeatureCollection","features":[]}

    if not GEO_PATH.exists():
        print(f"[GEO] file not found: {GEO_PATH}")
        return

    with GEO_PATH.open("r", encoding="utf-8") as f:
        data = json.load(f)

    def push_geom(g):
        for part in _explode_geometry(g):
            t = part["type"]
            coords_wgs84 = _transform_coords_to_wgs84(t, part["coordinates"])
            b = _bbox_of_coords(coords_wgs84)
            GEO_GEOMS.append({"type": t, "coordinates": coords_wgs84, "bbox": b})
            ALL_GEO_FC["features"].append({
                "type":"Feature",
                "properties":{},
                "geometry":{"type": t, "coordinates": coords_wgs84}
            })

    t = data.get("type")
    if t == "FeatureCollection":
        for feat in data.get("features", []):
            g = feat.get("geometry")
            if g: push_geom(g)
    elif t == "GeometryCollection":
        for g in data.get("geometries", []): push_geom(g)
    elif t in ("Polygon", "MultiPolygon", "LineString", "MultiLineString", "Point", "MultiPoint"):
        push_geom({"type": t, "coordinates": data.get("coordinates", [])})
    elif t == "Feature":
        g = data.get("geometry")
        if g: push_geom(g)

    print(f"[GEO] loaded {len(GEO_GEOMS)} parts from {GEO_PATH} (EPSG:{GEO_EPSG} → 4326); features in /api/geo/all: {len(ALL_GEO_FC['features'])}")

def _meters_per_deg(lat_deg: float) -> Tuple[float, float]:
    lat = math.radians(lat_deg)
    m_per_deg_lat = 111132.92 - 559.82*math.cos(2*lat) + 1.175*math.cos(4*lat) - 0.0023*math.cos(6*lat)
    m_per_deg_lon = 111412.84*math.cos(lat) - 93.5*math.cos(3*lat) + 0.118*math.cos(5*lat)
    return m_per_deg_lat, max(1e-6, m_per_deg_lon)

def _point_seg_dist_m(lat: float, lon: float, a: Tuple[float,float], b: Tuple[float,float]) -> float:
    mlat, mlon = _meters_per_deg(lat)
    ax, ay = ((a[0]-lon)*mlon, (a[1]-lat)*mlat)
    bx, by = ((b[0]-lon)*mlon, (b[1]-lat)*mlat)
    abx, aby = (bx-ax, by-ay)
    ab2 = abx*abx + aby*aby
    t = 0.0 if ab2 == 0 else max(0.0, min(1.0, (-(ax)*abx + (-(ay))*aby) / ab2))
    px, py = (ax + t*abx, ay + t*aby)
    return math.hypot(px, py)

def _ring_min_dist(lat: float, lon: float, coords) -> float:
    m = float("inf")
    if not coords or len(coords) < 2:
        return m
    for i in range(1, len(coords)):
        m = min(m, _point_seg_dist_m(lat, lon, coords[i-1], coords[i]))
    return m

def _geom_min_dist_m(lat: float, lon: float, g: Dict[str, Any]) -> float:
    t = g["type"]; c = g["coordinates"]
    if t == "LineString":
        return _ring_min_dist(lat, lon, c)
    if t == "MultiLineString":
        return min((_ring_min_dist(lat, lon, ls) for ls in c), default=float("inf"))
    if t == "Polygon":
        return min((_ring_min_dist(lat, lon, ring) for ring in c if ring and len(ring) >= 2), default=float("inf"))
    if t == "MultiPolygon":
        return min((_ring_min_dist(lat, lon, ring)
                    for poly in c if poly
                    for ring in poly if ring and len(ring) >= 2), default=float("inf"))
    return float("inf")

def _nearest_distance_m(lat: float, lon: float) -> Optional[float]:
    if not GEO_GEOMS: return None
    r = max(NEAR_SCAN_RADIUS_M, 1.0)
    dlat = r / 111_320.0
    dlon = r / (111_320.0 * max(1e-6, math.cos(math.radians(lat))))
    xmin, xmax = lon - dlon, lon + dlon
    ymin, ymax = lat - dlat, lat + dlat
    best = float("inf")
    for g in GEO_GEOMS:
        minx, miny, maxx, maxy = g["bbox"]
        if maxx < xmin or minx > xmax or maxy < ymin or miny > ymax:
            continue
        d = _geom_min_dist_m(lat, lon, {"type": g["type"], "coordinates": g["coordinates"]})
        if d < best:
            best = d
    return None if best == float("inf") else best

_preload_geoms()

# =======================
# Prediction + freshness
# =======================
def _predict_latlon(lat: float, lon: float, heading_deg: float, speed_mps: float, dt: float) -> Tuple[float, float]:
    mlat, mlon = _meters_per_deg(lat)
    rad = math.radians(heading_deg)
    d_n = math.cos(rad) * speed_mps * dt
    d_e = math.sin(rad) * speed_mps * dt
    return (lat + d_n / mlat, lon + d_e / mlon)

def _is_fresh(ts: Optional[float], max_age: float) -> bool:
    if ts is None or not isinstance(ts, (int, float)) or not math.isfinite(ts):
        return False
    return (_now_ts() - float(ts)) <= max(0.05, max_age)

# speed from consecutive points (meters)
def _dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    mlat, mlon = _meters_per_deg((lat1 + lat2) * 0.5)
    dx = (lon2 - lon1) * mlon
    dy = (lat2 - lat1) * mlat
    return math.hypot(dx, dy)

# =======================
# NTRIP (optional) replaced by mosaic-h internal NTRIP
# =======================
def _rtk_set(**kw):
    with rtk_lock:
        rtk_state.update(kw)

def _gga_from_latest() -> Optional[str]:
    with latest_lock:
        lt = dict(latest)
    lat = lt.get("lat"); lon = lt.get("lon")
    if lat is None or lon is None: return None
    lat = float(lat); lon = float(lon)
    lat_deg = int(abs(lat)); lat_min = (abs(lat) - lat_deg) * 60
    lon_deg = int(abs(lon)); lon_min = (abs(lon) - lon_deg) * 60
    lat_hem = "N" if lat >= 0 else "S"
    lon_hem = "E" if lon >= 0 else "W"
    gps_qual = int(lt.get("ggaFix", 1))
    num_sats = int(lt.get("sats", 0))
    hdop = float(lt.get("hdop", 1.0))
    alt = float(lt.get("alt", 0.0))
    core = (
        f"GPGGA,000000,{lat_deg:02d}{lat_min:06.3f},{lat_hem},"
        f"{lon_deg:03d}{lon_min:06.3f},{lon_hem},"
        f"{gps_qual},{num_sats:02d},{hdop:.1f},{alt:.1f},M,0.0,M,,"
    )
    cs = 0
    for ch in core: cs ^= ord(ch)
    return f"${core}*{cs:02X}\r\n"

def _ntrip_open(host: str, port: int, mount: str, tls: bool, user: str, pwd: str, timeout=10.0):
    auth_b64 = base64.b64encode(f"{user}:{pwd}".encode()).decode() if user else None
    def _connect():
        s = socket.create_connection((host, port), timeout=timeout)
        if tls:
            ctx = ssl.create_default_context()
            s = ctx.wrap_socket(s, server_hostname=host)
        return s
    s = _connect()
    req = (
        f"GET /{mount} HTTP/1.0\r\n"
        "User-Agent: NTRIP py-client\r\n"
        "Accept: */*\r\n"
        + (f"Authorization: Basic {auth_b64}\r\n" if auth_b64 else "")
        + "Connection: keep-alive\r\n\r\n"
    ).encode("ascii")
    s.sendall(req)
    hdr = b""
    s.settimeout(timeout)
    while b"\r\n\r\n" not in hdr and len(hdr) < 8192:
        chunk = s.recv(1024)
        if not chunk: break
        hdr += chunk
    first = hdr.split(b"\r\n", 1)[0].decode(errors="ignore") if hdr else ""
    if "200" not in first and not first.startswith("ICY 200"):
        raise IOError(f"NTRIP handshake failed: {first}")
    return s, first

def ntrip_thread_fn(ser: serial.Serial):
    if not (NTRIP_HOST and NTRIP_MOUNT):
        _rtk_set(last_error="NTRIP disabled"); return
    print(f"[NTRIP] connecting to {NTRIP_HOST}:{NTRIP_PORT}/{NTRIP_MOUNT} tls={NTRIP_TLS}")
    while True:
        try:
            _rtk_set(connected=False, last_error="connecting...")
            sock, first = _ntrip_open(NTRIP_HOST, NTRIP_PORT, NTRIP_MOUNT, NTRIP_TLS, NTRIP_USER, NTRIP_PASS)
            print(f"[NTRIP] {first}")
            _rtk_set(connected=True, last_error="", bytes_in=0, bytes_out=0)
            sock.settimeout(15.0)
            last_gga = 0.0
            while True:
                now = time.time()
                if NTRIP_GGA_INTERVAL > 0 and now - last_gga > NTRIP_GGA_INTERVAL:
                    s = _gga_from_latest()
                    if s:
                        try:
                            sock.sendall(s.encode("ascii"))
                            _rtk_set(last_gga=now)
                        except Exception as e:
                            _rtk_set(last_error=f"GGA send: {e}")
                    last_gga = now
                data = sock.recv(4096)
                if not data: raise IOError("NTRIP disconnected")
                with rtk_lock: rtk_state["bytes_in"] += len(data)
                try:
                    n = ser.write(data)
                    with rtk_lock: rtk_state["bytes_out"] += int(n or 0)
                except Exception as e:
                    _rtk_set(last_error=f"serial write: {e}")
        except Exception as e:
            print("[NTRIP] reconnect in 2s:", e)
            with rtk_lock:
                rtk_state["reconnects"] += 1
                rtk_state["connected"] = False
                rtk_state["last_error"] = str(e)
            time.sleep(2)

# =======================
# Fast NMEA parsing
# =======================
def _nmea_split_no_checksum(line: str) -> List[str]:
    star = line.find("*")
    if star != -1:
        line = line[:star]
    return line.split(",")

def _dm_to_deg(dm: str, hemi: str) -> Optional[float]:
    if not dm or not hemi:
        return None
    try:
        deg_len = 2 if hemi in ("N", "S") else 3
        deg = float(dm[:deg_len])
        minutes = float(dm[deg_len:])
        val = deg + minutes / 60.0
        if hemi in ("S", "W"):
            val = -val
        return val
    except Exception:
        return None

def _parse_gga(fields: List[str]) -> Optional[Dict[str, Any]]:
    # $GPGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,...
    if len(fields) < 10:
        return None
    lat = _dm_to_deg(fields[2], fields[3])
    lon = _dm_to_deg(fields[4], fields[5])
    if lat is None or lon is None:
        return None
    out: Dict[str, Any] = {"lat": lat, "lon": lon}
    try: out["ggaFix"] = int(fields[6]) if fields[6] else 0
    except Exception: pass
    try: out["sats"] = int(fields[7]) if fields[7] else 0
    except Exception: pass
    try: out["hdop"] = float(fields[8]) if fields[8] else None
    except Exception: pass
    try: out["alt"] = float(fields[9]) if fields[9] else None
    except Exception: pass
    return out

def _parse_hdt(fields: List[str]) -> Optional[float]:
    if len(fields) < 2:
        return None
    v = fields[1]
    if not v:
        return None
    try:
        return float(v)
    except Exception:
        return None

# =======================
# Threads: readers + workers + timers
# =======================
def _serial_open(port: str) -> serial.Serial:
    return serial.Serial(port, BAUD, timeout=0.2)

def _update_prediction_locked(now: float):
    """
    Uses heading + speedMps to compute plat/plon; if speed unknown -> no prediction.
    Must hold latest_lock.
    """
    lat = latest.get("lat"); lon = latest.get("lon")
    if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
        latest["plat"] = None
        latest["plon"] = None
        latest["pdt"] = PREDICT_T_SEC
        return

    hdg = latest.get("headingTrue")
    hdg_ts = latest.get("headingTs")
    spd = latest.get("speedMps")
    spd_ts = latest.get("speedTs")

    if not (isinstance(hdg, (int, float)) and math.isfinite(hdg) and _is_fresh(hdg_ts, HEADING_STALE_SEC)):
        latest["plat"] = None
        latest["plon"] = None
        latest["pdt"] = PREDICT_T_SEC
        return

    if not (isinstance(spd, (int, float)) and math.isfinite(spd) and _is_fresh(spd_ts, SPEED_STALE_SEC)):
        latest["plat"] = None
        latest["plon"] = None
        latest["pdt"] = PREDICT_T_SEC
        return

    speed_mps = float(spd)
    if speed_mps < 0.05:
        latest["plat"] = None
        latest["plon"] = None
        latest["pdt"] = PREDICT_T_SEC
        return

    plat, plon = _predict_latlon(float(lat), float(lon), float(hdg), speed_mps, float(PREDICT_T_SEC))
    latest["plat"] = plat
    latest["plon"] = plon
    latest["pdt"] = float(PREDICT_T_SEC)

def gga_reader_thread_fn():
    ser = _serial_open(GNSS_GGA_PORT)
    print(f"[GNSS] GGA port {GNSS_GGA_PORT} @ {BAUD}")

    if NTRIP_HOST and NTRIP_MOUNT:
        threading.Thread(target=ntrip_thread_fn, args=(ser,), daemon=True).start()

    buf = b""

    # for speed-from-GGA
    prev_lat: Optional[float] = None
    prev_lon: Optional[float] = None
    prev_ts: Optional[float] = None
    ema_speed: Optional[float] = None

    while True:
        try:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip(b"\r").strip()
                if not line or line[:1] != b"$":
                    continue
                s = line.decode("ascii", errors="ignore")
                if len(s) < 6 or s[3:6] != "GGA":
                    continue
                fields = _nmea_split_no_checksum(s)
                parsed = _parse_gga(fields)
                if not parsed:
                    continue

                now = _now_ts()
                parsed["ts"] = now

                # compute speed from consecutive positions (if enabled)
                if SPEED_FROM_GGA_ENABLED:
                    lat = parsed.get("lat"); lon = parsed.get("lon")
                    if (
                        isinstance(lat, (int, float)) and isinstance(lon, (int, float))
                        and prev_lat is not None and prev_lon is not None and prev_ts is not None
                    ):
                        dt = now - prev_ts
                        if SPEED_MIN_DT_SEC <= dt <= SPEED_MAX_DT_SEC:
                            d = _dist_m(prev_lat, prev_lon, float(lat), float(lon))
                            raw = d / max(1e-6, dt)
                            # reject absurd spikes
                            if 0.0 <= raw <= SPEED_MAX_MPS:
                                if ema_speed is None:
                                    ema_speed = raw
                                else:
                                    a = max(0.01, min(0.99, SPEED_SMOOTH_ALPHA))
                                    ema_speed = a * raw + (1.0 - a) * ema_speed

                    prev_lat = float(lat) if isinstance(lat, (int, float)) else prev_lat
                    prev_lon = float(lon) if isinstance(lon, (int, float)) else prev_lon
                    prev_ts = now

                with latest_lock:
                    latest.update(parsed)

                    # publish speed estimate (if we have it)
                    if SPEED_FROM_GGA_ENABLED and ema_speed is not None:
                        latest["speedMps"] = float(ema_speed)
                        latest["speedKmh"] = float(ema_speed) * 3.6
                        latest["speedTs"] = now

                    _update_prediction_locked(now)

                    lat = latest.get("lat"); lon = latest.get("lon")
                    plat = latest.get("plat"); plon = latest.get("plon")

                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    try:
                        while not nearest_q.empty():
                            nearest_q.get_nowait()
                        nearest_q.put_nowait(
                            (
                                float(lat),
                                float(lon),
                                float(plat) if isinstance(plat, (int, float)) else None,
                                float(plon) if isinstance(plon, (int, float)) else None,
                            )
                        )
                    except queue.Full:
                        pass

        except Exception:
            pass

def hdt_reader_thread_fn():
    ser = _serial_open(GNSS_HDT_PORT)
    print(f"[GNSS] HDT port {GNSS_HDT_PORT} @ {BAUD}")

    buf = b""
    while True:
        try:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip(b"\r").strip()
                if not line or line[:1] != b"$":
                    continue
                s = line.decode("ascii", errors="ignore")
                if len(s) < 6 or s[3:6] != "HDT":
                    continue
                fields = _nmea_split_no_checksum(s)
                heading = _parse_hdt(fields)
                if heading is None:
                    continue
                now = _now_ts()
                with latest_lock:
                    latest["headingTrue"] = float(heading)
                    latest["headingTs"] = now
                    if "ts" not in latest:
                        latest["ts"] = now
                    _update_prediction_locked(now)
                    lat = latest.get("lat"); lon = latest.get("lon")
                    plat = latest.get("plat"); plon = latest.get("plon")

                if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                    try:
                        while not nearest_q.empty():
                            nearest_q.get_nowait()
                        nearest_q.put_nowait(
                            (
                                float(lat),
                                float(lon),
                                float(plat) if isinstance(plat, (int, float)) else None,
                                float(plon) if isinstance(plon, (int, float)) else None,
                            )
                        )
                    except queue.Full:
                        pass
        except Exception:
            pass

def nearest_worker_thread():
    while True:
        try:
            lat, lon, plat, plon = nearest_q.get(timeout=0.5)
        except queue.Empty:
            continue

        d = None
        pd = None
        try:
            d = _nearest_distance_m(lat, lon)
        except Exception:
            d = None

        if isinstance(plat, (int, float)) and isinstance(plon, (int, float)):
            try:
                pd = _nearest_distance_m(float(plat), float(plon))
            except Exception:
                pd = None

        with latest_lock:
            latest["dmin"] = None if d is None or not (isinstance(d, (int, float)) and math.isfinite(d)) else round(float(d), 3)
            latest["pdmin"] = None if pd is None or not (isinstance(pd, (int, float)) and math.isfinite(pd)) else round(float(pd), 3)

def recorder_timer_thread():
    if POINTS_SAVE_HZ <= 0:
        return
    period = 1.0 / max(0.1, POINTS_SAVE_HZ)
    next_t = time.time()
    while True:
        now = time.time()
        if now < next_t:
            time.sleep(min(0.02, next_t - now))
            continue
        next_t += period

        with record_lock:
            on = record_on
        if not on:
            continue

        with latest_lock:
            lt = dict(latest)

        lat = lt.get("lat"); lon = lt.get("lon")
        if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
            _enqueue_line(
                float(lt.get("ts", now)),
                float(lat),
                float(lon),
                lt.get("headingTrue"),
                lt.get("speedMps"),
                float(lt.get("pdt", PREDICT_T_SEC)),
                lt.get("plat"),
                lt.get("plon"),
                lt.get("dmin"),
                lt.get("pdmin"),
            )
            _hz_mark("save")

def broadcaster_timer_thread():
    if BROADCAST_HZ <= 0:
        return
    period = 1.0 / max(0.5, BROADCAST_HZ)
    next_t = time.time()
    while True:
        now = time.time()
        if now < next_t:
            time.sleep(min(0.01, next_t - now))
            continue
        next_t += period

        _hz_mark("push")
        save_hz, push_hz = _hz_snapshot()

        with guidance_lock:
            predictive = bool(guidance_state["predictive"])

        with latest_lock:
            if not latest:
                continue
            d_for_audio = latest.get("pdmin") if predictive else latest.get("dmin")
            payload_obj = {
                "v": 1,
                "saveHz": round(save_hz, 2),
                "pushHz": round(push_hz, 2),
                "predictive": predictive,
                "predictT": PREDICT_T_SEC,
                "dForAudio": d_for_audio,
                **latest,
            }
            payload = json.dumps(payload_obj, separators=(",", ":"))

        for ws in list(clients):
            try:
                import asyncio
                asyncio.run_coroutine_threadsafe(ws.send_text(payload), ws.app_state.loop)
            except Exception:
                clients.discard(ws)

# =======================
# Hardware buzzer + LED guidance (Pi)
# =======================
def _clamp(x: float, a: float, b: float) -> float:
    return a if x < a else b if x > b else x

def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def _rate_for_A(d: float) -> float:
    X = IMAGINARY_OFFSET_M
    Amax = A_MAX_DIST_M
    t = _clamp((d - X) / max(1e-6, (Amax - X)), 0.0, 1.0)
    return _lerp(A_RATE_MIN_HZ, A_RATE_MAX_HZ, t)

def _rate_for_B(d: float) -> float:
    Bc = B_CONST_TONE_M
    X = IMAGINARY_OFFSET_M
    if d <= Bc:
        return float("inf")
    t = _clamp((d - Bc) / max(1e-6, (X - Bc)), 0.0, 1.0)
    return _lerp(B_RATE_MAX_HZ, B_RATE_MIN_HZ, t)

def hw_audio_thread():
    # Import gpiozero only inside the thread so the app still runs on laptop if GPIO isn't available
    try:
        from gpiozero import Device, LED, PWMOutputDevice
        from gpiozero.pins.lgpio import LGPIOFactory
        Device.pin_factory = LGPIOFactory()
    except Exception as e:
        print(f"[HW] GPIO init failed (disable with HW_AUDIO_ENABLED=0): {e}")
        return

    buzzer = PWMOutputDevice(HW_BUZZER_PIN)

    # Side LEDs
    led_left = LED(HW_LEFT_LED_PIN)
    led_right = LED(HW_RIGHT_LED_PIN)

    # 3 middle LEDs
    mid_left = LED(MID_LEFT_LED_PIN)
    mid_center = LED(MID_CENTER_LED_PIN)
    mid_right = LED(MID_RIGHT_LED_PIN)

    def all_off():
        buzzer.value = 0.0
        led_left.off(); led_right.off()
        mid_left.off(); mid_center.off(); mid_right.off()

    print(f"[HW] GPIO: buzzer={HW_BUZZER_PIN}, sideL={HW_LEFT_LED_PIN}, sideR={HW_RIGHT_LED_PIN}, midL={MID_LEFT_LED_PIN}, midC={MID_CENTER_LED_PIN}, midR={MID_RIGHT_LED_PIN}")

    # Startup tone
    try:
        buzzer.frequency = STARTUP_TONE_FREQ
        buzzer.value = STARTUP_TONE_VOL
        time.sleep(max(0.0, STARTUP_TONE_SEC))
    finally:
        buzzer.value = 0.0

    # Timers for gated beeps
    tA = 0.0
    tB = 0.0
    last = time.time()

    while True:
        now = time.time()
        dt = now - last
        if dt <= 0:
            dt = HW_LOOP_DT
        last = now

        with guidance_lock:
            predictive = bool(guidance_state["predictive"])

        with latest_lock:
            d = latest.get("pdmin") if predictive else latest.get("dmin")

        if d is None or not isinstance(d, (int, float)) or not math.isfinite(d):
            all_off()
            time.sleep(HW_LOOP_DT)
            continue

        d = float(d)

        # Error relative to target offset (meters)
        err = d - IMAGINARY_OFFSET_M
        abs_err = abs(err)

        # DEADZONE LED logic:
        # - <= CENTER_ALL_M: show all 3 middle LEDs, no sound, side LEDs off
        # - <= CENTER_DEADZONE_M: show only the side middle LED (too close => mid_left, too far => mid_right), no sound
        # - >  CENTER_DEADZONE_M: middle LEDs off; use side LEDs + buzzer logic as before
        if abs_err <= CENTER_ALL_M:
            buzzer.value = 0.0
            led_left.off(); led_right.off()
            mid_left.on(); mid_center.on(); mid_right.on()
            tA = 0.0; tB = 0.0
            time.sleep(HW_LOOP_DT)
            continue

        if abs_err <= CENTER_DEADZONE_M:
            buzzer.value = 0.0
            led_left.off(); led_right.off()
            mid_center.off()
            if err < 0:   # too close
                mid_left.on(); mid_right.off()
            else:         # too far
                mid_left.off(); mid_right.on()
            tA = 0.0; tB = 0.0
            time.sleep(HW_LOOP_DT)
            continue

        # Outside deadzone: middle LEDs OFF
        mid_left.off(); mid_center.off(); mid_right.off()

        # Hard silence beyond max distance
        if d >= A_MAX_DIST_M:
            buzzer.value = 0.0
            led_left.off(); led_right.off()
            tA = 0.0; tB = 0.0
            time.sleep(HW_LOOP_DT)
            continue

        # Use the old two-lane beeping (based on absolute distance threshold X),
        # but still choose which side LED to light based on err sign:
        #   err < 0 => too close => LEFT side LED
        #   err > 0 => too far   => RIGHT side LED
        if err < 0:
            # too close: LEFT lane (tone A)
            led_right.off()
            led_left.on()

            rate = _rate_for_A(d)
            period = 1.0 / max(1e-6, rate)

            tA += dt
            if tA >= period:
                tA -= period

            if tA < HW_GATE_ON_SEC:
                buzzer.frequency = TONE_A_FREQ
                buzzer.value = HW_VOLUME
            else:
                buzzer.value = 0.0

            tB = 0.0
        else:
            # too far: RIGHT lane (tone B)
            led_left.off()
            led_right.on()

            if d <= B_CONST_TONE_M:
                buzzer.frequency = TONE_B_FREQ
                buzzer.value = HW_VOLUME
                tB = 0.0
            else:
                rate = _rate_for_B(d)
                period = 1.0 / max(1e-6, rate)

                tB += dt
                if tB >= period:
                    tB -= period

                if tB < HW_GATE_ON_SEC:
                    buzzer.frequency = TONE_B_FREQ
                    buzzer.value = HW_VOLUME
                else:
                    buzzer.value = 0.0

            tA = 0.0

        time.sleep(HW_LOOP_DT)

# =======================
# HTML (Live UI) available from Raspberry PI ip:8002 or app start up port
# =======================
INDEX_HTML = r"""
<!doctype html><meta charset="utf-8">
<title>GNSS Fast Live</title>
<link rel="icon" href="/favicon.ico">
<style>
html,body{height:100%;margin:0}
#map{height:100%}
#hud{
  position:absolute;top:10px;left:10px;z-index:1000;
  background:#fff; border:1px solid #d0d7de; border-radius:8px; padding:8px 10px;
  font:14px/1.2 system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
  box-shadow:0 1px 2px rgba(0,0,0,0.05); min-width:560px
}
#hud b{font-weight:600}
#hud button{padding:4px 8px;border:1px solid #d0d7de;border-radius:6px;background:#fff;cursor:pointer}
#hud button.active{background:#e6f4ff;border-color:#8ac1ff}
#hud .value{font-weight:600}
#emptyBadge{
  display:none;margin-left:6px;padding:2px 6px;border-radius:999px;
  background:#ffe7e7;color:#b80000;font-size:12px;border:1px solid #f4caca;
}
</style>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<div id="map"></div>
<div id="hud">
  <div><b>Fix:</b> <span class="value" id="fix">?</span></div>
  <div><b>Sats:</b> <span class="value" id="sats">?</span> | <b>HDOP:</b> <span class="value" id="hdop">?</span> | <b>Speed:</b> <span class="value" id="spd">?</span></div>
  <div><b>Lat:</b> <span id="lat">?</span> | <b>Lon:</b> <span id="lon">?</span></div>
  <div><b>Heading:</b> <span class="value" id="hdg">?</span> | <b>Predict:</b> <span class="value" id="pt">?</span></div>
  <div><b>Nearest:</b> <span class="value" id="dmin">–</span> | <b>PredNearest:</b> <span class="value" id="pdmin">–</span></div>
  <div><b>AudioDist:</b> <span class="value" id="dused">–</span></div>
  <div><b>CSV Hz:</b> <span class="value" id="saveHz">0.0</span> | <b>UI Hz:</b> <span class="value" id="pushHz">0.0</span></div>
  <div style="margin-top:6px; display:flex; gap:6px; align-items:center; flex-wrap:wrap">
    <button id="followBtn" class="active">Follow: ON</button>
    <button id="recordBtn">Record: OFF</button>
    <button id="predBtn">Predictive: OFF</button>
    <button id="audioBtn" style="display:none">Audio: OFF</button>
    <button id="linesBtn">Lines: OFF</button>
    <button id="allBtn">Load All</button>
    <button id="clearBtn">Clear</button>
    <span id="emptyBadge">no lines in view</span>
    <span id="fname" style="font-size:12px;color:#666"></span>
  </div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const CFG = {
  enableAudioDefault: __AUDIO_ENABLED__,
  X: __IMAG_X__,
  Amax: __A_MAX_DIST__,
  Bconst: __B_CONST__,
  ArateMin: __A_RATE_MIN__, ArateMax: __A_RATE_MAX__,
  BrateMin: __B_RATE_MIN__, BrateMax: __B_RATE_MAX__,
  freqA: __FREQ_A__, freqB: __FREQ_B__,
  pulseOnSec: 0.06,
  gainA: 0.2,
  gainB: 0.25
};

const map = L.map('map', { preferCanvas:true }).setView([58.38, 26.73], 17);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19, attribution: '&copy; OSM' }).addTo(map);

const ws = new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws');
const el = id => document.getElementById(id);
function humanFix(q){ const m={0:'No Fix',1:'GNSS',2:'DGPS',4:'RTK Fix',5:'RTK Float'}; return m[q] ?? String(q ?? '?'); }

let marker=null;
let predMarker=null;
let follow=true;

const path = L.polyline([], { color:'#ff2a6d', weight:2, opacity:0.9 }).addTo(map);
const trail= [];

const headingLine = L.polyline([], { color:'#111', weight:3, opacity:0.85 }).addTo(map);
const predLine    = L.polyline([], { color:'#8b0000', weight:2, opacity:0.90, dashArray:"6 6" }).addTo(map);

function setFollow(on){
  follow = !!on;
  el('followBtn').textContent = 'Follow: ' + (follow?'ON':'OFF');
  el('followBtn').classList.toggle('active', follow);
}
el('followBtn').onclick = () => setFollow(!follow);

async function setPredictive(on){
  try{
    const r = await fetch('/api/guidance/predictive', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ predictive: !!on })
    });
    if(!r.ok) return;
    const j = await r.json();
    el('predBtn').textContent = 'Predictive: ' + (j.predictive ? 'ON' : 'OFF');
    el('predBtn').classList.toggle('active', !!j.predictive);
  }catch(e){}
}
el('predBtn').onclick = () => {
  const isOn = el('predBtn').textContent.includes('ON');
  setPredictive(!isOn);
};

// ---------- AudioWorklet ----------
let audioOn = false, ac=null, node=null;
async function setupAudioWorklet() {
  if (ac) return;
  ac = new (window.AudioContext || window.webkitAudioContext)();
  const code = `
  class BeepProcessor extends AudioWorkletProcessor {
    constructor(){
      super();
      this.cfg = { X:0.2,Amax:1.0,Bconst:0.1,ArateMin:2,ArateMax:8,BrateMin:2,BrateMax:8,freqA:880,freqB:660,pulseOnSec:0.06,gainA:0.2,gainB:0.25 };
      this.enabled=false; this.dmin=null;
      this.phaseA=0; this.phaseB=0; this.tA=0; this.tB=0;
      this.port.onmessage=(e)=>{ const m=e.data||{}; if(m.cfg)Object.assign(this.cfg,m.cfg); if('on'in m)this.enabled=!!m.on; if('dmin'in m)this.dmin=(m.dmin==null?null:Number(m.dmin)); };
    }
    clamp(x,a,b){ return Math.max(a, Math.min(b, x)); }
    lerp(a,b,t){ return a + (b-a)*t; }
    rateForA(d){ const {X,Amax,ArateMin,ArateMax}=this.cfg; const t=this.clamp((d-X)/Math.max(1e-6,(Amax-X)),0,1); return this.lerp(ArateMin,ArateMax,t); }
    rateForB(d){ const {Bconst,X,BrateMin,BrateMax}=this.cfg; if(d<=Bconst) return Infinity; const t=this.clamp((d-Bconst)/Math.max(1e-6,(X-Bconst)),0,1); return this.lerp(BrateMax,BrateMin,t); }
    process(inputs, outputs){
      const out=outputs[0]; const L=out[0], R=out[1]||out[0]; const sr=sampleRate;
      if(!this.enabled || this.dmin==null || !isFinite(this.dmin)){ for(let i=0;i<L.length;i++){L[i]=0; if(R!==L)R[i]=0;} return true; }
      const cfg=this.cfg;
      for(let i=0;i<L.length;i++){
        const d=Math.max(0,this.dmin); let sample=0.0;
        if(d < cfg.Amax){
          if(d >= cfg.X){
            const rate=this.rateForA(d); const T=(isFinite(rate)&&rate>0)?(1/rate):1e9;
            this.tA+=1/sr; if(this.tA>=T) this.tA-=T;
            const gate=(this.tA < cfg.pulseOnSec)?cfg.gainA:0.0;
            this.phaseA+=(2*Math.PI*cfg.freqA)/sr; if(this.phaseA>1e9) this.phaseA-=1e9;
            sample += Math.sin(this.phaseA)*gate; this.tB=0;
          } else if(d>0){
            let gate=0.0;
            if(d <= cfg.Bconst){ gate = cfg.gainB*Math.max(0,Math.min(1,d/cfg.Bconst)); this.tB=0; }
            else { const rate=this.rateForB(d); const T=(isFinite(rate)&&rate>0)?(1/rate):1e9; this.tB+=1/sr; if(this.tB>=T) this.tB-=T; gate=(this.tB < cfg.pulseOnSec)?cfg.gainB:0.0; }
            this.phaseB+=(2*Math.PI*cfg.freqB)/sr; if(this.phaseB>1e9) this.phaseB-=1e9;
            sample += Math.sin(this.phaseB)*gate; this.tA=0;
          }
        }
        L[i]=sample; if(R!==L) R[i]=sample;
      }
      return true;
    }
  }
  registerProcessor('beep-processor', BeepProcessor);
  `;
  const blob = new Blob([code], {type:'application/javascript'});
  const url = URL.createObjectURL(blob);
  await ac.audioWorklet.addModule(url);
  node = new AudioWorkletNode(ac, 'beep-processor', { outputChannelCount: [2] });
  node.connect(ac.destination);
  node.port.postMessage({ cfg: CFG });
  node.port.postMessage({ on: false });
}
function setAudioOn(on){
  audioOn = !!on;
  if (!ac) return;
  node && node.port.postMessage({ on: audioOn });
  el('audioBtn').textContent = 'Audio: ' + (audioOn ? 'ON' : 'OFF');
}
function pushDminToWorklet(val){ if(node) node.port.postMessage({ dmin: val }); }
(function initAudioUI(){
  const btn = el('audioBtn');
  if (!CFG.enableAudioDefault) return;
  btn.style.display = 'inline-block';
  btn.onclick = async () => {
    await setupAudioWorklet();
    if (ac.state === 'suspended') await ac.resume();
    setAudioOn(!audioOn);
  };
})();

// Lines layer (viewport fetch)
let linesOn = false;
const linesBtn  = el('linesBtn');
const allBtn    = el('allBtn');
const clearBtn  = el('clearBtn');
const emptyBadge= el('emptyBadge');
const LINE_STYLE= { color:'#7b00ff', weight:2, opacity:0.85 };
const linesLayer= L.layerGroup().addTo(map);
let allGeoLayer = null;

function setLines(on){
  linesOn = !!on;
  linesBtn.textContent = 'Lines: ' + (linesOn ? 'ON' : 'OFF');
  if (!linesOn) { linesLayer.clearLayers(); emptyBadge.style.display='none'; return; }
  lastQueryKey = '';
  fetchViewport();
}
linesBtn.onclick = ()=> setLines(!linesOn);
clearBtn.onclick = ()=> { linesLayer.clearLayers(); if (allGeoLayer){ map.removeLayer(allGeoLayer); allGeoLayer=null; } emptyBadge.style.display='none'; };

allBtn.onclick = async ()=>{
  try{
    const r = await fetch('/api/geo/all', { cache:'no-store' });
    if (!r.ok) throw 0;
    const fc = await r.json();
    linesLayer.clearLayers();
    if (allGeoLayer){ map.removeLayer(allGeoLayer); allGeoLayer=null; }
    allGeoLayer = L.geoJSON(fc, { style: ()=> ({ color:'#0066ff', weight:1, opacity:0.9, fill:false }) }).addTo(map);
    emptyBadge.style.display = (fc && fc.features && fc.features.length) ? 'none':'inline-block';
  }catch(e){}
};

let linesFetchBusy = false, linesFetchQueued = false, debounceTimer = null, lastQueryKey = '';
function scheduleFetch(){
  if (!linesOn) return;
  if (debounceTimer) clearTimeout(debounceTimer);
  debounceTimer = setTimeout(()=>{ if (linesFetchBusy) { linesFetchQueued = true; return; } fetchViewport(); }, 300);
}
map.on('moveend', scheduleFetch);
map.on('zoomend', scheduleFetch);

async function fetchViewport(){
  if (!linesOn) return;
  linesFetchBusy = true;
  try{
    const b = map.getBounds();
    const params = new URLSearchParams({
      minLat: b.getSouth().toFixed(7),
      minLon: b.getWest().toFixed(7),
      maxLat: b.getNorth().toFixed(7),
      maxLon: b.getEast().toFixed(7),
    });
    const key = params.toString();
    if (key === lastQueryKey) { linesFetchBusy=false; return; }
    lastQueryKey = key;

    const r = await fetch('/api/geo/viewport?' + key, { cache:'no-store' });
    if (!r.ok) throw 0;
    const j = await r.json();

    linesLayer.clearLayers();
    if (allGeoLayer){ map.removeLayer(allGeoLayer); allGeoLayer=null; }
    let n=0;
    for (const line of (j.lines || [])) {
      if (Array.isArray(line) && line.length >= 2) { L.polyline(line, LINE_STYLE).addTo(linesLayer); n++; }
    }
    emptyBadge.style.display = n ? 'none' : 'inline-block';
  }catch(e){}finally{
    linesFetchBusy = false;
    if (linesFetchQueued) { linesFetchQueued=false; setTimeout(fetchViewport, 200); }
  }
}

async function recStatus(){
  try{
    const r = await fetch('/api/record/status'); if(!r.ok) return;
    const j = await r.json(); if(!j.ok) return;
    el('recordBtn').textContent = 'Record: ' + (j.on?'ON':'OFF');
    el('fname').textContent = j.on ? ('File: '+j.file) : '';
    if (typeof j.saveHz === 'number') el('saveHz').textContent = j.saveHz.toFixed(2);
    if (typeof j.pushHz === 'number') el('pushHz').textContent = j.pushHz.toFixed(2);
    el('predBtn').textContent = 'Predictive: ' + (j.predictive ? 'ON' : 'OFF');
    el('predBtn').classList.toggle('active', !!j.predictive);
  }catch(e){}
}
async function recToggle(){
  try{
    const on = el('recordBtn').textContent.includes('ON');
    await fetch(on?'/api/record/stop':'/api/record/start', {method:'POST'});
    await recStatus();
  }catch(e){}
}
el('recordBtn').onclick = recToggle;
recStatus();

// heading arrow math
function metersPerDeg(latDeg){
  const lat = latDeg * Math.PI/180;
  const mLat = 111132.92 - 559.82*Math.cos(2*lat) + 1.175*Math.cos(4*lat) - 0.0023*Math.cos(6*lat);
  const mLon = 111412.84*Math.cos(lat) - 93.5*Math.cos(3*lat) + 0.118*Math.cos(5*lat);
  return [mLat, Math.max(1e-6, mLon)];
}

const showDist = (x) => (typeof x === "number" && isFinite(x))
  ? (x >= 1000 ? (x/1000).toFixed(3) + " km" : x.toFixed(3) + " m")
  : "–";

ws.onmessage = async (ev)=>{
  const d = JSON.parse(ev.data);

  if (d.ggaFix != null) el('fix').textContent = humanFix(d.ggaFix);
  if (d.sats != null) el('sats').textContent = d.sats;
  if (d.hdop != null) el('hdop').textContent = (+d.hdop).toFixed(1);

  if (typeof d.speedMps === 'number' && isFinite(d.speedMps)) el('spd').textContent = d.speedMps.toFixed(2) + ' m/s';
  else el('spd').textContent = '?';

  if (typeof d.headingTrue === 'number' && isFinite(d.headingTrue)) el('hdg').textContent = d.headingTrue.toFixed(2) + '°';
  else el('hdg').textContent = '?';

  if (typeof d.predictT === 'number') el('pt').textContent = d.predictT.toFixed(2) + ' s';

  el('dmin').textContent = showDist(d.dmin);
  el('pdmin').textContent = showDist(d.pdmin);
  el('dused').textContent = showDist(d.dForAudio);

  if (!ac && CFG.enableAudioDefault) { await setupAudioWorklet(); }
  if (ac && audioOn && node) { pushDminToWorklet(d.dForAudio); }

  if ('predictive' in d) {
    el('predBtn').textContent = 'Predictive: ' + (d.predictive ? 'ON' : 'OFF');
    el('predBtn').classList.toggle('active', !!d.predictive);
  }

  if (typeof d.lat === 'number' && typeof d.lon === 'number'){
    el('lat').textContent = (+d.lat).toFixed(7);
    el('lon').textContent = (+d.lon).toFixed(7);

    const ll = [d.lat, d.lon];
    if (!marker){
      marker = L.circleMarker(ll, {radius:6,color:'#111',fillColor:'#00d084',fillOpacity:0.9,weight:1}).addTo(map);
      map.panTo(ll);
    } else marker.setLatLng(ll);

    // Heading arrow (6m)
    if (typeof d.headingTrue === 'number' && isFinite(d.headingTrue)) {
      const [mLat, mLon] = metersPerDeg(d.lat);
      const lenM = 6.0;
      const rad = d.headingTrue * Math.PI/180;
      const dLat = (Math.cos(rad) * lenM) / mLat;
      const dLon = (Math.sin(rad) * lenM) / mLon;
      const end = [d.lat + dLat, d.lon + dLon];
      headingLine.setLatLngs([ll, end]);
    } else headingLine.setLatLngs([]);

    // Predicted point marker + dashed connector (RED)
    if (typeof d.plat === 'number' && typeof d.plon === 'number' && isFinite(d.plat) && isFinite(d.plon)) {
      const pll = [d.plat, d.plon];
      if (!predMarker){
        predMarker = L.circleMarker(pll, {radius:6,color:'#8b0000',fillColor:'#ff0000',fillOpacity:0.85,weight:2}).addTo(map);
      } else predMarker.setLatLng(pll);
      predLine.setLatLngs([ll, pll]);
    } else {
      if (predMarker) { try { map.removeLayer(predMarker); } catch(e){} predMarker=null; }
      predLine.setLatLngs([]);
    }

    trail.push(ll);
    if (trail.length > %(TRAIL_MAX)d) trail.shift();
    path.setLatLngs(trail);
    if (follow) map.panTo(ll, { animate:true });
  }
};
</script>
"""

# =======================
# Routes
# =======================
@app.get("/")
def index():
    html = INDEX_HTML
    repl = {
        "%(TRAIL_MAX)d": str(TRAIL_MAX),
        "__AUDIO_ENABLED__": "true" if AUDIO_ENABLED else "false",
        "__IMAG_X__": f"{IMAGINARY_OFFSET_M:.6f}",
        "__A_MAX_DIST__": f"{A_MAX_DIST_M:.6f}",
        "__B_CONST__": f"{B_CONST_TONE_M:.6f}",
        "__A_RATE_MIN__": f"{A_RATE_MIN_HZ:.6f}",
        "__A_RATE_MAX__": f"{A_RATE_MAX_HZ:.6f}",
        "__B_RATE_MIN__": f"{B_RATE_MIN_HZ:.6f}",
        "__B_RATE_MAX__": f"{B_RATE_MAX_HZ:.6f}",
        "__FREQ_A__": f"{TONE_A_FREQ:.2f}",
        "__FREQ_B__": f"{TONE_B_FREQ:.2f}",
    }
    for k, v in repl.items():
        html = html.replace(k, v)
    return HTMLResponse(html)

@app.get("/favicon.ico")
def favicon():
    svg = '<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16"><rect width="16" height="16" fill="#7b00ff"/></svg>'
    return Response(content=svg, media_type="image/svg+xml")

@app.post("/api/record/start")
def api_record_start():
    return JSONResponse(_record_start())

@app.post("/api/record/stop")
def api_record_stop():
    return JSONResponse(_record_stop())

@app.get("/api/record/status")
def api_record_status():
    return JSONResponse(_record_status())

@app.post("/api/guidance/predictive")
def api_set_predictive(payload: Dict[str, Any]):
    val = bool(payload.get("predictive", False))
    with guidance_lock:
        guidance_state["predictive"] = val
    return JSONResponse({"ok": True, "predictive": val, "predictT": PREDICT_T_SEC})

@app.get("/api/status")
def api_status():
    with latest_lock:
        if not latest:
            return JSONResponse({"ok": False, "msg": "no data yet"})
        save_hz, push_hz = _hz_snapshot()
        with guidance_lock:
            predictive = bool(guidance_state["predictive"])
        d_for_audio = latest.get("pdmin") if predictive else latest.get("dmin")
        return JSONResponse({
            "ok": True,
            "data": {**latest, "saveHz": round(save_hz, 2), "pushHz": round(push_hz, 2),
                     "predictive": predictive, "predictT": PREDICT_T_SEC, "dForAudio": d_for_audio}
        })

@app.get("/api/geo/viewport")
def api_geo_viewport(
    minLat: float = Query(...),
    minLon: float = Query(...),
    maxLat: float = Query(...),
    maxLon: float = Query(...)
):
    if not GEO_API_ENABLE:
        return JSONResponse({"ok": True, "lines": []})

    lat_lo, lat_hi = (min(minLat, maxLat), max(minLat, maxLat))
    lon_lo, lon_hi = (min(minLon, maxLon), max(minLon, maxLon))

    out: List[List[List[float]]] = []
    stride = max(1, int(GEO_SIMPLIFY_STRIDE))
    cap = max(1, int(GEO_API_MAX_LINES))

    for g in GEO_GEOMS:
        if len(out) >= cap:
            break
        minx, miny, maxx, maxy = g["bbox"]
        if maxx < lon_lo or minx > lon_hi or maxy < lat_lo or miny > lat_hi:
            continue
        t = g["type"]
        c = g["coordinates"]

        if t == "LineString":
            line = _to_latlon_line(c, stride)
            if line: out.append(line)
        elif t == "MultiLineString":
            for ls in c:
                if len(out) >= cap: break
                line = _to_latlon_line(ls, stride)
                if line: out.append(line)
        elif t == "Polygon":
            if c and isinstance(c[0], list):
                line = _to_latlon_line(c[0], stride)
                if line: out.append(line)
        elif t == "MultiPolygon":
            for poly in c:
                if len(out) >= cap: break
                if poly and isinstance(poly[0], list):
                    line = _to_latlon_line(poly[0], stride)
                    if line: out.append(line)

    return JSONResponse({"ok": True, "lines": out})

@app.get("/api/geo/all")
def api_geo_all():
    return ALL_GEO_FC

@app.websocket("/ws")
async def ws(ws: WebSocket):
    await ws.accept()
    ws.app_state = type("S", (), {})()
    import asyncio
    ws.app_state.loop = asyncio.get_running_loop()
    clients.add(ws)
    try:
        while True:
            await asyncio.sleep(30)
    finally:
        clients.discard(ws)

# =======================
# Boot threads
# =======================
threading.Thread(target=_writer_thread, daemon=True).start()
threading.Thread(target=gga_reader_thread_fn, daemon=True).start()
threading.Thread(target=hdt_reader_thread_fn, daemon=True).start()
threading.Thread(target=nearest_worker_thread, daemon=True).start()
threading.Thread(target=recorder_timer_thread, daemon=True).start()
threading.Thread(target=broadcaster_timer_thread, daemon=True).start()
if HW_AUDIO_ENABLED:
    threading.Thread(target=hw_audio_thread, daemon=True).start()