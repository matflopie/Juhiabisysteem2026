# app_replay.py
# function to watch back, replay, combine recordings/data
# runs on same requirements and source venv

## run
## python -m uvicorn app_replay:app --host 0.0.0.0 --port 8001

import os, json, time, math
from typing import Optional, Dict, Any, List, Tuple
from math import cos, radians
from pathlib import Path
from string import Template

from fastapi import FastAPI, Query
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pyproj import Transformer

# --------- CONFIG (env or defaults) ----------
# Keep map & polygons identical to live app
GEO_PATH = Path("static/25cm3301.geojson")
GEO_EPSG = int(os.getenv("GEO_EPSG", "3301"))  # default to Estonia L-EST97

# Where recordings are saved
RECORD_DIR = Path(os.getenv("RECORD_DIR", "recordings"))

# ===== Audio-related numbers only appear in HUD text hint; no audio here =====
ALERT_SILENT_BEYOND_M = float(os.getenv("ALERT_SILENT_BEYOND_M", "2"))
ALERT_SILENT_RESUME_M = float(os.getenv("ALERT_SILENT_RESUME_M", "1.5"))

# default search radius for /api/geo_near (meters)
DEFAULT_R_METERS = 50.0


# ---------------------------------------------
app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

# ============ GEO CACHE (same as live app) ============

GEO_GEOMS: List[Dict[str, Any]] = []  # entries in WGS84 lon/lat with bboxes
_TRANSFORMER: Optional[Transformer] = None

def _get_transformer() -> Optional[Transformer]:
    global _TRANSFORMER
    if _TRANSFORMER is None and GEO_EPSG != 4326:
        _TRANSFORMER = Transformer.from_crs(GEO_EPSG, 4326, always_xy=True)
    return _TRANSFORMER

def _transform_coords_to_wgs84(geom_type: str, coords):
    """Transform from GEO_EPSG -> WGS84 lon/lat. If already 4326, return as-is."""
    tr = _get_transformer()
    if tr is None:
        return coords  # already WGS84

    def tx_xy(x, y):
        lon, lat = tr.transform(x, y)
        return [lon, lat]

    def walk(t, c):
        if t == "Point":
            x, y = c
            return tx_xy(x, y)
        if t in ("MultiPoint", "LineString"):
            return [tx_xy(x, y) for x, y in c]
        if t == "MultiLineString":
            return [[tx_xy(x, y) for x, y in part] for part in c]
        if t == "Polygon":
            return [[tx_xy(x, y) for x, y in ring] for ring in c]
        if t == "MultiPolygon":
            return [[[tx_xy(x, y) for x, y in ring] for ring in poly] for poly in c]
        return c

    return walk(geom_type, coords)

def _explode_geometry(geom):
    """Yield simple per-part geometries (split MultiPolygon/MultiLineString)."""
    t = geom.get("type")
    c = geom.get("coordinates", [])
    if t == "MultiPolygon":
        for poly in c:
            yield {"type": "Polygon", "coordinates": poly}
    elif t == "MultiLineString":
        for ls in c:
            yield {"type": "LineString", "coordinates": ls}
    else:
        yield geom

def _bbox_of_coords(coords) -> Tuple[float, float, float, float]:
    mnx = mny = float("inf")
    mxx = mxy = float("-inf")

    def walk(c):
        nonlocal mnx, mny, mxx, mxy
        if isinstance(c, list):
            if c and isinstance(c[0], (int, float)):
                x, y = c[0], c[1]
                mnx = min(mnx, x); mny = min(mny, y)
                mxx = max(mxx, x); mxy = max(mxy, y)
            else:
                for k in c:
                    walk(k)

    walk(coords)
    if mnx == float("inf"):
        return (0, 0, 0, 0)
    return (mnx, mny, mxx, mxy)

def _preload_geoms():
    """Load GeoJSON, reproject to WGS84, explode parts, cache with bboxes."""
    global GEO_GEOMS
    if not GEO_PATH.exists():
        print(f"[GEO] file not found: {GEO_PATH}")
        GEO_GEOMS = []
        return

    with GEO_PATH.open("r", encoding="utf-8") as f:
        data = json.load(f)

    raw_geoms: List[Dict[str, Any]] = []

    def push_geom(g):
        for part in _explode_geometry(g):
            t = part["type"]
            coords_wgs84 = _transform_coords_to_wgs84(t, part["coordinates"])
            b = _bbox_of_coords(coords_wgs84)
            raw_geoms.append({"type": t, "coordinates": coords_wgs84, "bbox": b})

    t = data.get("type")
    if t == "FeatureCollection":
        for feat in data.get("features", []):
            g = feat.get("geometry")
            if g:
                push_geom(g)
    elif t == "GeometryCollection":
        for g in data.get("geometries", []):
            push_geom(g)
    elif t in ("Polygon", "MultiPolygon", "LineString", "MultiLineString", "Point", "MultiPoint"):
        push_geom({"type": t, "coordinates": data.get("coordinates", [])})
    elif t == "Feature":
        g = data.get("geometry")
        if g:
            push_geom(g)

    GEO_GEOMS = raw_geoms
    print(f"[GEO] loaded {len(GEO_GEOMS)} parts from {GEO_PATH} (EPSG:{GEO_EPSG} → 4326)")

_preload_geoms()

# ---- precise distance helpers (WGS84 lon/lat) ----
def _meters_per_deg(lat_deg: float) -> Tuple[float, float]:
    lat = math.radians(lat_deg)
    m_per_deg_lat = 111132.92 - 559.82*math.cos(2*lat) + 1.175*math.cos(4*lat) - 0.0023*math.cos(6*lat)
    m_per_deg_lon = 111412.84*math.cos(lat) - 93.5*math.cos(3*lat) + 0.118*math.cos(5*lat)
    return m_per_deg_lat, max(1e-6, m_per_deg_lon)

def _point_point_dist_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    mlat, mlon = _meters_per_deg((lat1 + lat2) * 0.5)
    dx = (lon2 - lon1) * mlon
    dy = (lat2 - lat1) * mlat
    return math.hypot(dx, dy)

def _point_seg_dist_m(lat: float, lon: float, a: Tuple[float,float], b: Tuple[float,float]) -> float:
    mlat, mlon = _meters_per_deg(lat)
    ax, ay = ( (a[0]-lon)*mlon, (a[1]-lat)*mlat )
    bx, by = ( (b[0]-lon)*mlon, (b[1]-lat)*mlat )
    abx, aby = (bx-ax, by-ay)
    ab2 = abx*abx + aby*aby
    t = 0.0 if ab2 == 0 else max(0.0, min(1.0, (-(ax)*abx + (-(ay))*aby) / ab2))
    px, py = (ax + t*abx, ay + t*aby)
    return math.hypot(px, py)

def _geom_min_dist_m(lat: float, lon: float, g: Dict[str,Any]) -> float:
    t = g["type"]; c = g["coordinates"]

    def ring_min(coords):
        m = float("inf")
        if not coords or len(coords) < 2:
            return m
        for i in range(1, len(coords)):
            m = min(m, _point_seg_dist_m(lat, lon, coords[i-1], coords[i]))
        return m

    if t == "Point":
        x, y = c
        return _point_point_dist_m(lat, lon, y, x)

    if t == "MultiPoint":
        return min((_point_point_dist_m(lat, lon, y, x) for x, y in c), default=float("inf"))

    if t == "LineString":
        return ring_min(c)

    if t == "MultiLineString":
        return min((ring_min(ls) for ls in c), default=float("inf"))

    if t == "Polygon":
        return min((ring_min(ring) for ring in c if ring and len(ring) >= 2), default=float("inf"))

    if t == "MultiPolygon":
        return min(
            (ring_min(ring)
             for poly in c if poly
             for ring in poly if ring and len(ring) >= 2),
            default=float("inf")
        )

    return float("inf")


# ===== COMBINER: CSV I/O helpers =====
def _load_points_csv(path: Path) -> List[Dict[str, Any]]:
    pts: List[Dict[str, Any]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        _ = f.readline()  # header
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) < 4:
                parts += [""] * (4 - len(parts))
            try:
                ts = float(parts[0]) if parts[0] else float('nan')
            except Exception:
                continue
            try:
                lat = float(parts[1]); lon = float(parts[2])
            except Exception:
                continue
            try:
                dmin = float(parts[3]) if parts[3] != "" else None
            except Exception:
                dmin = None
            pts.append({"ts": ts, "lat": lat, "lon": lon, "dmin": dmin})
    return pts

def _save_points_csv(path: Path, points: List[Dict[str, Any]]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("ts,lat,lon,dmin\n")
        for p in points:
            ts = p.get("ts"); lat = p.get("lat"); lon = p.get("lon"); dmin = p.get("dmin")
            if ts is None or lat is None or lon is None:
                continue
            d = "" if dmin is None or not (isinstance(dmin, (int, float)) and math.isfinite(dmin)) else f"{float(dmin):.3f}"
            f.write(f"{float(ts):.3f},{float(lat):.8f},{float(lon):.8f},{d}\n")

def _combine_points(files: List[str], drop_dup: bool, rebase_ts: bool) -> List[Dict[str, Any]]:
    merged: List[Dict[str, Any]] = []
    for f in files:
        safe = Path(f).name
        path = RECORD_DIR / safe
        if not path.exists() or not path.is_file():
            continue
        merged.extend(_load_points_csv(path))
    merged.sort(key=lambda p: (p["ts"], p["lat"], p["lon"]))
    if drop_dup:
        ded: List[Dict[str, Any]] = []
        seen = set()
        for p in merged:
            key = (round(p["ts"], 3), round(p["lat"], 8), round(p["lon"], 8))
            if key in seen:
                continue
            seen.add(key)
            ded.append(p)
        merged = ded
    if rebase_ts and merged:
        t0 = merged[0]["ts"]
        for p in merged:
            try:
                p["ts"] = float(p["ts"]) - float(t0)
            except Exception:
                pass
    return merged


# ============ HTML (Map + picker + playback) ============

MAP_HTML_TPL = Template(r"""<!doctype html>
<meta charset="utf-8" />
<title>Replay: GNSS trail</title>
<meta name="viewport" content="width=device-width, initial-scale=1" />
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>

<style>
  html, body { height: 100%; margin: 0; }
  #map { height: 100%; }
  .leaflet-container { background: #f5f7fb; }
  #hud {
    position: absolute;
    top: 10px; left: 10px;
    z-index: 1000;
    background: rgba(255,255,255,0.92);
    border: 1px solid #d0d7de;
    border-radius: 8px;
    padding: 8px 10px;
    font: 14px/1.2 system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
    box-shadow: 0 1px 2px rgba(0,0,0,0.05);
    min-width: 360px;
    max-width: 520px;
  }
  #hud b { font-weight: 600; }
  #hud .ok   { color: #0a7f2e; }
  #hud .warn { color: #a15c00; }
  #hud .bad  { color: #b00020; }
  #hud button, #hud select, #hud input[type=range], #hud input[type=number], #hud input[type=text]{
    padding:4px 8px;border:1px solid #d0d7de;border-radius:6px;background:#fff;cursor:pointer
  }
  #row { display:flex; gap:6px; margin-top:6px; align-items:center; flex-wrap: wrap; }
  #row label { font-size: 13px; color:#444; }
  #meta { color:#555; font-size:12px; margin-top:4px; }
  #followBtn.active{background:#e6f4ff;border-color:#8ac1ff}
  #scrub { width: 100%; }
</style>

<div id="map"></div>
<div id="hud">
  <div><b>Replay viewer</b></div>

  <!-- Combiner UI -->
  <div id="row" style="width:100%">
    <select id="multi" multiple style="width:100%;height:130px"></select>
    <div style="font-size:12px;color:#666">Ctrl/Cmd-click to select multiple files. We sort by timestamp.</div>
  </div>
  <div id="row" style="width:100%">
    <button id="refreshBtn">Refresh</button>
    <button id="combinePreviewBtn">Preview combine</button>
    <input id="outName" type="text" placeholder="combined_YYYYMMDD.csv" style="flex:1" />
    <button id="combineSaveBtn">Save combined</button>
  </div>
  <div id="row">
    <label><input type="checkbox" id="dropDup" checked /> Drop duplicates</label>
    <label><input type="checkbox" id="rebaseTs" /> Rebase ts to 0</label>
  </div>

  <!-- Single-file load + playback -->
  <div id="row">
    <select id="fileSel"></select>
    <button id="loadBtn">Load</button>
    <button id="fitBtn">Fit</button>
    <button id="followBtn" class="active">Follow: ON</button>
  </div>

  <div id="row">
    <button id="playBtn">Play</button>
    <button id="pauseBtn">Pause</button>
    <label>Speed</label>
    <input id="speed" type="range" min="0.05" max="4" value="1" step="0.05" />
    <span id="speedVal">1×</span>
    <input id="speedNum" type="number" min="0.05" max="4" step="0.05" value="1" style="width:70px" />
    <span style="font-size:12px;color:#666">×</span>
  </div>

  <div id="row" style="width:100%">
    <button id="back10">⟲ 10</button>
    <button id="back1">⟲ 1</button>
    <input id="scrub" type="range" min="0" max="0" value="0" step="1" />
    <button id="fwd1">1 ⟳</button>
    <button id="fwd10">10 ⟳</button>
  </div>

  <div id="meta"></div>
  <div><b>Nearest:</b> <span id="dmin">–</span> <span style="color:#666">(mute &gt; ${SILENT_BEYOND_M_ROUND} m)</span></div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
  const map = L.map('map', { preferCanvas: true });
  // 3Di Tartu orthophoto WMS
  const wmsUrl = 'http://wms.3di.ee/mapproxy/tartu2024/service';
  const ortho = L.tileLayer.wms(wmsUrl, {
    layers: 'Ortofoto', format: 'image/jpeg', transparent: false, version: '1.3.0', styles: '', crs: L.CRS.EPSG3857
  }).addTo(map);
  ortho.bringToBack();
  map.setView([58.38, 26.73], 18);

  function makeGeoLayer(fc) {
    return L.geoJSON(fc, {
      style: () => ({ color: '#0066ff', weight: 1, opacity: 0.9, fill: false }),
      onEachFeature: (feat, layer) => {
        if (layer && layer.options) { layer.options.smoothFactor = 0.3; layer.options.renderer = L.canvas(); }
      }
    });
  }

  let geoLayer = null;

  async function loadPolysAroundCenterOnce(){
    try {
      const c = map.getCenter();
      const r = 300; // meters
      const url = "/api/geo_near?lat=" + encodeURIComponent(c.lat)
                + "&lon=" + encodeURIComponent(c.lng)
                + "&r=" + encodeURIComponent(r)
                + "&limit=5000&precise=0";
      const res = await fetch(url);
      if (!res.ok) throw new Error("HTTP " + res.status);
      const data = await res.json();
      let fc;
      if (data.type === "GeometryCollection") {
        fc = {
          type: "FeatureCollection",
          features: (data.geometries || []).map(g => ({ type:"Feature", properties:{}, geometry:g }))
        };
      } else if (data.type === "FeatureCollection") {
        fc = data;
      } else {
        fc = { type:"FeatureCollection", features:[{ type:"Feature", properties:{}, geometry:data }] };
      }
      if (geoLayer) { try { map.removeLayer(geoLayer); } catch(e){} }
      geoLayer = makeGeoLayer(fc).addTo(map);
      geoLayer.bringToBack();
      ortho.bringToBack();
    } catch(e) {
      console.error(e);
    }
  }
  loadPolysAroundCenterOnce();

  // ---- UI elements ----
  const fileSel   = document.getElementById('fileSel');
  const loadBtn   = document.getElementById('loadBtn');
  const fitBtn    = document.getElementById('fitBtn');
  const playBtn   = document.getElementById('playBtn');
  const pauseBtn  = document.getElementById('pauseBtn');
  const speedIn   = document.getElementById('speed');
  const speedVal  = document.getElementById('speedVal');
  const speedNum  = document.getElementById('speedNum');
  const followBtn = document.getElementById('followBtn');
  const metaEl    = document.getElementById('meta');
  const dminEl    = document.getElementById('dmin');

  const scrub     = document.getElementById('scrub');
  const back1     = document.getElementById('back1');
  const back10    = document.getElementById('back10');
  const fwd1      = document.getElementById('fwd1');
  const fwd10     = document.getElementById('fwd10');

  // Combiner
  const multi   = document.getElementById('multi');
  const refreshBtn = document.getElementById('refreshBtn');
  const combinePreviewBtn = document.getElementById('combinePreviewBtn');
  const combineSaveBtn = document.getElementById('combineSaveBtn');
  const outName  = document.getElementById('outName');
  const dropDup  = document.getElementById('dropDup');
  const rebaseTs = document.getElementById('rebaseTs');

  // Playback state
  let points = []; // [{ts,lat,lon,dmin}]
  let poly = L.polyline([], { color: '#ff2a6d', weight: 2, opacity: 0.9 }).addTo(map);
  let marker = null;
  let idx = 0;

  // Real-time playback loop via requestAnimationFrame
  let playing = false;
  let rafId = null;
  let lastTick = null;
  let acc = 0; // step accumulator
  const BASE_FPS = 30; // steps/second at 1× (one point per "frame")

  // ---- Speed helpers (instant effect) ----
  function currentSpeed(){
    const v = parseFloat(speedIn.value || '1');
    const min = parseFloat(speedIn.min), max = parseFloat(speedIn.max);
    const val = (isNaN(v) ? 1 : Math.max(min, Math.min(max, v)));
    return val;
  }
  function setSpeedUI(v){
    const min = parseFloat(speedIn.min), max = parseFloat(speedIn.max);
    const val = Math.max(min, Math.min(max, v));
    speedIn.value = String(val);
    speedNum.value = String(val);
    speedVal.textContent = (Math.round(val*100)/100) + "×";
  }
  speedIn.addEventListener('input', ()=> setSpeedUI(parseFloat(speedIn.value || "1")));  // loop reads currentSpeed() every frame
  speedNum.addEventListener('input', ()=> setSpeedUI(parseFloat(speedNum.value || "1")));
  setSpeedUI(1);

  // Follow toggle
  function setFollow(on){
    const yes = !!on;
    followBtn.textContent = "Follow: " + (yes ? "ON" : "OFF");
    followBtn.classList.toggle('active', yes);
  }
  followBtn.onclick = () => setFollow(!followBtn.classList.contains('active'));
  setFollow(true); // default ON

  async function refreshList(){
    try{
      const r = await fetch("/api/recordings");
      if(!r.ok) return;
      const j = await r.json();

      // single-file list
      fileSel.innerHTML = "";
      // multi-select list
      multi.innerHTML = "";

      (j.files || []).forEach(f=>{
        const opt1 = document.createElement('option');
        opt1.value = f.name;
        opt1.textContent = f.name + " ("+f.count+" pts, "+(f.size_kb.toFixed(1))+" KB)";
        fileSel.appendChild(opt1);

        const opt2 = document.createElement('option');
        opt2.value = f.name;
        opt2.textContent = f.name + " ("+f.count+" pts, "+(f.size_kb.toFixed(1))+" KB)";
        multi.appendChild(opt2);
      });
    }catch(e){}
  }
  refreshList();
  refreshBtn.onclick = refreshList;

  function clearTrack(){
    points = [];
    poly.setLatLngs([]);
    if (marker) { try { map.removeLayer(marker); } catch(e){} marker = null; }
    idx = 0;
    metaEl.textContent = "";
    dminEl.textContent = "–";
    scrub.min = "0"; scrub.max = "0"; scrub.value = "0";
  }

  async function loadSelected(){
    clearTrack();
    const f = fileSel.value;
    if (!f) return;
    try{
      const r = await fetch("/api/recording?file=" + encodeURIComponent(f));
      if(!r.ok) throw new Error("HTTP "+r.status);
      const j = await r.json();
      points = j.points || [];
      if (points.length){
        poly.setLatLngs(points.map(p=>[p.lat, p.lon]));
        if (!marker){
          marker = L.circleMarker([points[0].lat, points[0].lon], {
            radius: 6, color: "#111", fillColor: "#00d084", fillOpacity: 0.9, weight: 1
          }).addTo(map);
        }
        const b = L.latLngBounds(points.map(p=>[p.lat, p.lon]));
        map.fitBounds(b.pad(0.15));
        const t0 = points[0].ts, t1 = points[points.length-1].ts;
        metaEl.textContent = "Points: "+points.length+" | Duration: "+((t1 - t0).toFixed(1))+" s";
        scrub.max = String(points.length - 1);
        scrub.value = "0";
        idx = 0;
        renderAtIndex(idx);
      }
    }catch(e){
      console.error(e);
    }
  }

  loadBtn.onclick = loadSelected;
  fitBtn.onclick = () => {
    if (points.length){
      const b = L.latLngBounds(points.map(p=>[p.lat, p.lon]));
      map.fitBounds(b.pad(0.15));
    }
  };

  // Distance (meters) between two Leaflet LatLngs (haversine)
  function metersBetween(a, b){
    const toRad = d=>d*Math.PI/180, R=6371000;
    const dLat=toRad(b.lat-a.lat), dLon=toRad(b.lng-a.lng);
    const la1=toRad(a.lat), la2=toRad(b.lat);
    const s=Math.sin(dLat/2)**2 + Math.cos(la1)*Math.cos(la2)*Math.sin(dLon/2)**2;
    return 2*R*Math.asin(Math.min(1, Math.sqrt(s)));
  }

  function renderAtIndex(i){
    if (!points.length) return;
    const k = Math.max(0, Math.min(points.length-1, i));
    idx = k;
    const p = points[k];
    const latlng = [p.lat, p.lon];

    if (marker) {
      const old = marker.getLatLng();
      marker.setLatLng(latlng);
      if (followBtn.classList.contains('active')) {
        const moved = metersBetween({lat:old.lat, lng:old.lng}, {lat:p.lat, lng:p.lon});
        if (moved > 1.5) map.panTo(latlng, { animate: true });
      }
    } else {
      marker = L.circleMarker(latlng, {
        radius: 6, color: "#111", fillColor: "#00d084", fillOpacity: 0.9, weight: 1
      }).addTo(map);
      if (followBtn.classList.contains('active')) map.panTo(latlng);
    }

    dminEl.textContent = (typeof p.dmin === "number" && isFinite(p.dmin))
      ? (p.dmin >= 1000 ? (p.dmin/1000).toFixed(2)+" km" : p.dmin.toFixed(2)+" m")
      : "–";

    // keep scrub UI in sync if it wasn't the driver
    if (parseInt(scrub.value, 10) !== k) scrub.value = String(k);
  }

  function loop(ts){
    if (!playing) return;
    if (lastTick == null) lastTick = ts;
    const dt = ts - lastTick; // ms
    lastTick = ts;

    // Advance accumulator by speed-scaled "frames"
    const spd = currentSpeed();                    // reads slider every frame → real-time adjust
    acc += dt * spd * (BASE_FPS / 1000);

    if (acc >= 1){
      const adv = Math.floor(acc);
      acc -= adv;
      const next = Math.min(points.length - 1, idx + adv);
      if (next !== idx){
        renderAtIndex(next);
      }
      if (idx >= points.length - 1){
        pause();
        return;
      }
    }
    rafId = requestAnimationFrame(loop);
  }

  function play(){
    if (!points.length) return;
    if (playing) return;
    playing = true;
    lastTick = null;
    rafId = requestAnimationFrame(loop);
  }
  function pause(){
    playing = false;
    if (rafId) { cancelAnimationFrame(rafId); rafId = null; }
  }

  playBtn.onclick = play;
  pauseBtn.onclick = pause;

  // ---- Scrubbing & small-step controls ----
  function stepBy(n){
    if (!points.length) return;
    const next = Math.max(0, Math.min(points.length - 1, idx + n));
    renderAtIndex(next);
  }
  back1.onclick  = ()=> stepBy(-1);
  back10.onclick = ()=> stepBy(-10);
  fwd1.onclick   = ()=> stepBy(1);
  fwd10.onclick  = ()=> stepBy(10);

  // Dragging scrubber updates position immediately (and playback continues from there if playing)
  let scrubbing = false;
  scrub.addEventListener('input', ()=>{
    scrubbing = true;
    renderAtIndex(parseInt(scrub.value || "0", 10));
  });
  scrub.addEventListener('change', ()=>{ scrubbing = false; });

  // ---- Combiner wiring ----
  async function combinePreview(){
    const files = Array.from(multi.selectedOptions).map(o=>o.value);
    if (files.length < 2){ alert("Select at least 2 files."); return; }
    const url = "/api/combine_points?"
              + files.map(f=>"files="+encodeURIComponent(f)).join("&")
              + "&drop_dup="+(dropDup.checked?1:0)
              + "&rebase_ts="+(rebaseTs.checked?1:0);
    const r = await fetch(url);
    if(!r.ok){ alert("Combine preview failed"); return; }
    const j = await r.json();

    // Replace current points and redraw
    clearTrack();
    points = j.points || [];
    if (points.length){
      poly.setLatLngs(points.map(p=>[p.lat,p.lon]));
      const b = L.latLngBounds(points.map(p=>[p.lat,p.lon]));
      map.fitBounds(b.pad(0.15));
      const t0=points[0].ts, t1=points[points.length-1].ts;
      metaEl.textContent = "Preview: "+files.length+" files → "+points.length+" pts | Duration: "+((t1-t0).toFixed(1))+" s";
      scrub.max = String(points.length - 1);
      renderAtIndex(0);
    } else {
      metaEl.textContent = "Preview: 0 points";
    }
  }
  combinePreviewBtn.onclick = combinePreview;

  async function combineSave(){
    const files = Array.from(multi.selectedOptions).map(o=>o.value);
    if (files.length < 2){ alert("Select at least 2 files."); return; }
    const out = (outName.value.trim() || ("combined_"+Date.now()+".csv"));
    const r = await fetch("/api/combine_save", {
      method: "POST",
      headers: {"Content-Type":"application/json"},
      body: JSON.stringify({ files, out, drop_dup: !!dropDup.checked, rebase_ts: !!rebaseTs.checked })
    });
    const j = await r.json();
    if (!j.ok){ alert("Save failed: " + (j.msg||"")); return; }
    metaEl.textContent = "Saved "+j.out+" | points="+j.points+" | files="+j.files;

    // refresh lists and auto-load the new file for replay
    await refreshList();
    const found = Array.from((fileSel.options||[])).find(o=>o.value === j.out);
    if (found){ fileSel.value = j.out; loadSelected && loadSelected(); }
  }
  combineSaveBtn.onclick = combineSave;

</script>
""")

MAP_HTML = MAP_HTML_TPL.safe_substitute(
    SILENT_BEYOND_M=int(ALERT_SILENT_BEYOND_M),
    SILENT_RESUME_M=int(ALERT_SILENT_RESUME_M),
    SILENT_BEYOND_M_ROUND=int(round(ALERT_SILENT_BEYOND_M)),
)

# ============ ROUTES ============

@app.get("/")
def index():
    return HTMLResponse(MAP_HTML)

# List available recording files in recordings/ (ts,lat,lon,dmin CSV)
@app.get("/api/recordings")
def list_recordings():
    RECORD_DIR.mkdir(parents=True, exist_ok=True)
    files = []
    for p in sorted(RECORD_DIR.glob("*.csv")):
        try:
            size_kb = p.stat().st_size / 1024.0
            # quick count (lines-1 for header)
            count = 0
            with p.open("r", encoding="utf-8", errors="ignore") as f:
                for i, _ in enumerate(f, 1):
                    pass
            count = max(0, i - 1) if 'i' in locals() else 0
            files.append({"name": p.name, "size_kb": size_kb, "count": count})
        except Exception:
            pass
    return {"files": files}

# Load a specific recording as JSON list of points
@app.get("/api/recording")
def get_recording(file: str = Query(..., description="CSV filename under recordings/")):
    # security: prevent path traversal
    safe = Path(file).name
    path = RECORD_DIR / safe
    if not path.exists() or not path.is_file():
        return JSONResponse({"ok": False, "msg": "file not found"}, status_code=404)

    pts: List[Dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as f:
            _ = f.readline()  # header
            for line in f:
                line = line.strip()
                if not line:
                    continue
                # expected: ts,lat,lon,dmin
                parts = line.split(",")
                if len(parts) < 4:
                    # tolerate partial lines
                    parts += [""] * (4 - len(parts))
                try:
                    ts = float(parts[0]) if parts[0] else float('nan')
                except Exception:
                    continue
                try:
                    lat = float(parts[1]); lon = float(parts[2])
                except Exception:
                    continue
                dmin: Optional[float]
                try:
                    dmin = float(parts[3]) if parts[3] != "" else None
                except Exception:
                    dmin = None
                pts.append({"ts": ts, "lat": lat, "lon": lon, "dmin": dmin})
    except Exception as e:
        return JSONResponse({"ok": False, "msg": str(e)}, status_code=500)

    # normalize timestamps: ensure monotonic increasing if needed
    pts.sort(key=lambda p: (p["ts"], p["lat"], p["lon"]))
    return {"ok": True, "points": pts}

# ---- Combiner endpoints ----
@app.get("/api/combine_points")
def api_combine_points(
    files: List[str] = Query(..., description="repeat ?files=a.csv&files=b.csv"),
    drop_dup: int = Query(1),
    rebase_ts: int = Query(0),
):
    pts = _combine_points(files, bool(drop_dup), bool(rebase_ts))
    return {"ok": True, "points": pts, "count": len(pts)}

@app.post("/api/combine_save")
def api_combine_save(payload: Dict[str, Any]):
    files = payload.get("files") or []
    out = (payload.get("out") or "").strip()
    drop_dup = bool(payload.get("drop_dup", True))
    rebase_ts = bool(payload.get("rebase_ts", False))
    if not files or len(files) < 2:
        return {"ok": False, "msg": "need at least two files"}
    if not out.endswith(".csv"):
        out += ".csv"
    pts = _combine_points(files, drop_dup, rebase_ts)
    try:
        out_path = RECORD_DIR / Path(out).name
        _save_points_csv(out_path, pts)
        return {"ok": True, "out": out_path.name, "points": len(pts), "files": len(files)}
    except Exception as e:
        return {"ok": False, "msg": str(e)}

# ---- Optional: same polygon helpers as live app ----
@app.get("/api/geo_count")
def geo_count():
    return {"count": len(GEO_GEOMS), "epsg_in": GEO_EPSG}

@app.get("/api/geo_debug")
def geo_debug():
    if not GEO_GEOMS:
        return {"count": 0}
    mnx = mny = float("inf")
    mxx = mxy = float("-inf")
    type_counts = {}
    for g in GEO_GEOMS:
        t = g["type"]
        type_counts[t] = type_counts.get(t, 0) + 1
        x0,y0,x1,y1 = g["bbox"]
        mnx = min(mnx,x0); mny = min(mny,y0)
        mxx = max(mxx,x1); mxy = max(mxy,y1)
    return {
        "count": len(GEO_GEOMS),
        "types": type_counts,
        "bbox_wgs84": [mnx, mny, mxx, mxy],
    }

@app.get("/api/geo_sample")
def geo_sample(n: int = 3):
    return {"sample": GEO_GEOMS[:max(0, min(n, len(GEO_GEOMS)))]}

@app.get("/api/geo_nearest")
def geo_nearest(lat: float, lon: float):
    if not GEO_GEOMS:
        return {"ok": False, "msg": "no geoms"}
    best = {"d": float("inf"), "idx": -1, "type": None}
    for i, g in enumerate(GEO_GEOMS):
        d = _geom_min_dist_m(lat, lon, {"type": g["type"], "coordinates": g["coordinates"]})
        if d < best["d"]:
            best = {"d": d, "idx": i, "type": g["type"]}
    return {"ok": True, "nearest_m": best["d"], "geom_idx": best["idx"], "type": best["type"]}

@app.get("/api/geo_near")
def geo_near(
    lat: float,
    lon: float,
    r: float = DEFAULT_R_METERS,
    limit: int = 500,
    precise: int = 0,
    with_d: int = 0,
):
    if not GEO_GEOMS:
        return {"type": "GeometryCollection", "geometries": []}

    dlat = r / 111_320.0
    dlon = r / (111_320.0 * max(0.000001, cos(radians(lat))))
    xmin, xmax = lon - dlon, lon + dlon
    ymin, ymax = lat - dlat, lat + dlat

    out = []
    for g in GEO_GEOMS:
        minx, miny, maxx, maxy = g["bbox"]
        # FIX: correct bbox reject test (was "maxy > ymax")
        if maxx < xmin or minx > xmax or maxy < ymin or miny > ymax:
            continue

        if precise:
            dmin = _geom_min_dist_m(lat, lon, {"type": g["type"], "coordinates": g["coordinates"]})
            if dmin > r:
                continue
            if with_d:
                out.append({"type": g["type"], "coordinates": g["coordinates"], "d": dmin})
            else:
                out.append({"type": g["type"], "coordinates": g["coordinates"]})
        else:
            out.append({"type": g["type"], "coordinates": g["coordinates"]})

        if len(out) >= limit:
            break

    return {"type": "GeometryCollection", "geometries": out}
