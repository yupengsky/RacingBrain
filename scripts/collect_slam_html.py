#!/usr/bin/env python3
from __future__ import annotations

import argparse
import html
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from drd25_msgs.msg import Map
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from test_cone_segmentation.msg import ThreeDConeArray
from visualization_msgs.msg import Marker, MarkerArray


def point_xy(x: float, y: float) -> Dict[str, float]:
    return {"x": round(float(x), 4), "y": round(float(y), 4)}


def distance(a: Dict[str, float], b: Dict[str, float]) -> float:
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def color_hex(r: float, g: float, b: float) -> str:
    values = [max(0, min(255, int(round(v * 255.0)))) for v in (r, g, b)]
    if values == [0, 0, 0]:
        values = [20, 90, 220]
    return "#{:02x}{:02x}{:02x}".format(*values)


class SlamHtmlCollector(Node):
    def __init__(self, *, frame_period_sec: float, max_lidar_points: int) -> None:
        super().__init__("slam_html_collector")
        self.started_at = time.time()
        self.frame_period_sec = max(0.1, frame_period_sec)
        self.max_lidar_points = max(50, max_lidar_points)

        self.lidar_messages = 0
        self.local_detection_messages = 0
        self.fusion_map_messages = 0
        self.global_map_messages = 0
        self.odom_messages = 0

        self.latest_lidar: List[List[float]] = []
        self.latest_local_cones: List[Dict[str, float]] = []
        self.latest_fusion_cones: List[Dict[str, Any]] = []
        self.latest_markers: List[Dict[str, Any]] = []
        self.confirmed_markers_by_key: Dict[str, Dict[str, Any]] = {}
        self.latest_vehicle: Optional[Dict[str, float]] = None
        self.odom_path: List[Dict[str, float]] = []
        self.frames: List[Dict[str, Any]] = []
        self.last_lidar_sample_wall = 0.0

        self.create_subscription(PointCloud2, "/lidar_points", self.cb_lidar, 10)
        self.create_subscription(ThreeDConeArray, "/cone_detection_custom", self.cb_local_cones, 10)
        self.create_subscription(Map, "/perception/fusion/map", self.cb_fusion_map, 10)
        self.create_subscription(MarkerArray, "/global_map", self.cb_global_map, 10)
        self.create_subscription(Odometry, "/vehicle_odom", self.cb_odom, 10)
        self.create_timer(self.frame_period_sec, self.capture_frame)

    def elapsed(self) -> float:
        return time.time() - self.started_at

    def cb_lidar(self, msg: PointCloud2) -> None:
        self.lidar_messages += 1
        now = time.time()
        if now - self.last_lidar_sample_wall < min(0.45, self.frame_period_sec):
            return
        self.last_lidar_sample_wall = now

        total = max(1, int(msg.width) * int(msg.height))
        stride = max(1, total // self.max_lidar_points)
        points: List[List[float]] = []
        for index, point in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if index % stride != 0:
                continue
            try:
                x, y, z = float(point[0]), float(point[1]), float(point[2])
            except (IndexError, TypeError, ValueError):
                x, y, z = float(point["x"]), float(point["y"]), float(point["z"])
            if -2.0 <= x <= 35.0 and abs(y) <= 18.0 and -3.0 <= z <= 4.0:
                points.append([round(x, 3), round(y, 3), round(z, 3)])
            if len(points) >= self.max_lidar_points:
                break
        self.latest_lidar = points

    def cb_local_cones(self, msg: ThreeDConeArray) -> None:
        self.local_detection_messages += 1
        cones: List[Dict[str, float]] = []
        for cone in msg.cones:
            cones.append(
                {
                    "x": round(float(cone.center.x), 4),
                    "y": round(float(cone.center.y), 4),
                    "z": round(float(cone.center.z), 4),
                    "h": round(float(cone.size.z), 4),
                }
            )
        self.latest_local_cones = cones

    def cb_fusion_map(self, msg: Map) -> None:
        self.fusion_map_messages += 1
        self.latest_fusion_cones = [
            {
                "x": round(float(cone.x), 4),
                "y": round(float(cone.y), 4),
                "color": int(cone.color),
            }
            for cone in msg.track
        ]

    def cb_global_map(self, msg: MarkerArray) -> None:
        self.global_map_messages += 1
        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            if marker.points:
                for point_index, point in enumerate(marker.points):
                    key = f"{marker.ns}:{int(marker.id)}:{point_index}"
                    self.confirm_marker(
                        key,
                        {
                            "x": round(float(point.x), 4),
                            "y": round(float(point.y), 4),
                            "id": int(marker.id),
                            "ns": marker.ns,
                            "color": color_hex(marker.color.r, marker.color.g, marker.color.b),
                        },
                    )
            else:
                key = f"{marker.ns}:{int(marker.id)}:pose"
                self.confirm_marker(
                    key,
                    {
                        "x": round(float(marker.pose.position.x), 4),
                        "y": round(float(marker.pose.position.y), 4),
                        "id": int(marker.id),
                        "ns": marker.ns,
                        "color": color_hex(marker.color.r, marker.color.g, marker.color.b),
                    },
                )
        self.latest_markers = self.sorted_confirmed_markers()

    def confirm_marker(self, key: str, marker: Dict[str, Any]) -> None:
        existing = self.confirmed_markers_by_key.get(key)
        if existing is not None:
            marker["firstSeen"] = existing.get("firstSeen")
        else:
            marker["firstSeen"] = round(self.elapsed(), 2)
        self.confirmed_markers_by_key[key] = marker

    def sorted_confirmed_markers(self) -> List[Dict[str, Any]]:
        return sorted(
            self.confirmed_markers_by_key.values(),
            key=lambda marker: (int(marker.get("id", 0)), float(marker.get("x", 0.0)), float(marker.get("y", 0.0))),
        )

    def cb_odom(self, msg: Odometry) -> None:
        self.odom_messages += 1
        point = {
            "t": round(self.elapsed(), 3),
            "x": round(float(msg.pose.pose.position.x), 4),
            "y": round(float(msg.pose.pose.position.y), 4),
            "yaw": round(yaw_from_odom(msg), 5),
        }
        self.latest_vehicle = point
        if not self.odom_path or distance(self.odom_path[-1], point) >= 0.2:
            self.odom_path.append(point)

    def capture_frame(self) -> None:
        if self.latest_vehicle is None and not self.latest_markers and not self.latest_lidar:
            return
        frame = {
            "t": round(self.elapsed(), 2),
            "frame": len(self.frames),
            "lidar": self.latest_lidar,
            "local": self.latest_local_cones,
            "fusion": self.latest_fusion_cones,
            "markers": self.latest_markers,
            "vehicle": self.latest_vehicle,
            "pathCount": len(self.odom_path),
            "counts": {
                "lidarMessages": self.lidar_messages,
                "localDetectionMessages": self.local_detection_messages,
                "fusionMapMessages": self.fusion_map_messages,
                "globalMapMessages": self.global_map_messages,
                "odomMessages": self.odom_messages,
            },
        }
        self.frames.append(frame)

    def snapshot(self) -> Dict[str, Any]:
        if not self.frames:
            self.capture_frame()
        return {
            "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_sec": round(self.elapsed(), 2),
            "frame_period_sec": self.frame_period_sec,
            "frame_count": len(self.frames),
            "lidar_messages": self.lidar_messages,
            "local_detection_messages": self.local_detection_messages,
            "fusion_map_messages": self.fusion_map_messages,
            "global_map_messages": self.global_map_messages,
            "odom_messages": self.odom_messages,
            "marker_count": len(self.confirmed_markers_by_key),
            "path_count": len(self.odom_path),
            "odom_path": self.odom_path,
            "frames": self.frames,
        }


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def write_html(output_path: Path, data: Dict[str, Any]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    output_path.write_text(
        HTML_TEMPLATE.replace("__SLAM_DATA__", html.escape(payload, quote=False)),
        encoding="utf-8",
    )


HTML_TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RacingBrain Mapping Demo</title>
<style>
html,body{margin:0;height:100%;overflow:hidden;background:#f3f5f2;color:#252a2e;font:13px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#stage{display:block;width:100vw;height:100vh;background:#f3f5f2;cursor:default}
#controls{position:fixed;left:0;right:0;bottom:0;height:44px;display:flex;align-items:center;gap:12px;padding:0 16px;background:rgba(243,245,242,.94);border-top:1px solid #d7ddd6}
#play,#reset{height:28px;border:1px solid #aeb8b0;background:#fff;color:#1f2428;border-radius:6px;padding:0 12px;font:12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
#timeline{flex:1}
#speed{height:28px;border:1px solid #aeb8b0;background:#fff;border-radius:6px}
#readout{min-width:168px;text-align:right;color:#59635e}
</style>
</head>
<body>
<canvas id="stage"></canvas>
<div id="controls">
  <button id="play">Pause</button>
  <input id="timeline" type="range" min="0" max="0" value="0">
  <select id="speed"><option value="0.5">0.5x</option><option value="1" selected>1x</option><option value="2">2x</option><option value="4">4x</option></select>
  <button id="reset">Reset View</button>
  <span id="readout"></span>
</div>
<script id="slam-data" type="application/json">__SLAM_DATA__</script>
<script>
const data = JSON.parse(document.getElementById("slam-data").textContent);
const frames = data.frames || [];
const path = data.odom_path || [];
const canvas = document.getElementById("stage");
const ctx = canvas.getContext("2d");
const playButton = document.getElementById("play");
const resetButton = document.getElementById("reset");
const timeline = document.getElementById("timeline");
const speedSelect = document.getElementById("speed");
const readout = document.getElementById("readout");
let dpr = 1, width = 0, height = 0;
const defaultIndex = frames.findIndex(f => (f.t || 0) >= 80 && (f.markers || []).length >= 5 && (f.local || []).length >= 3);
let index = defaultIndex >= 0 ? defaultIndex : 0, playing = true, lastStep = 0;
let globalZoom = 1, globalPan = {x:0,y:0}, dragging = false, dragLast = null;
timeline.max = Math.max(0, frames.length - 1);

function resize(){
  dpr = window.devicePixelRatio || 1;
  width = innerWidth;
  height = innerHeight - 44;
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  canvas.style.width = width + "px";
  canvas.style.height = height + "px";
  ctx.setTransform(dpr,0,0,dpr,0,0);
  draw();
}

function panels(){
  const gap = 14;
  const header = 58;
  const w = (width - gap) / 2;
  const h = (height - header - gap) / 2;
  return {
    lidar:{x:0,y:header,w,h,title:"LiDAR point cloud"},
    global:{x:w+gap,y:header,w,h,title:"Global map view (camera follows vehicle)"},
    local:{x:0,y:header+h+gap,w,h,title:"Local detections in LiDAR frame"},
    status:{x:w+gap,y:header+h+gap,w,h,title:"Status dashboard"}
  };
}

function current(){ return frames[Math.max(0, Math.min(index, frames.length - 1))] || {}; }

function drawTitle(f){
  ctx.fillStyle = "#f3f5f2";
  ctx.fillRect(0,0,width,58);
  ctx.fillStyle = "#111";
  ctx.font = "700 28px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  ctx.fillText("RacingBrain Mapping Demo", 0, 29);
  ctx.font = "700 13px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  const status = [
    ["elapsed", fmt(f.t || 0,1)+"s"],
    ["frame", String(f.frame || 0)],
    ["mode", "full bag playback"],
    ["map msgs", String(data.global_map_messages || 0)],
    ["path pts", String(data.path_count || 0)]
  ];
  let x = 0;
  for(const [k,v] of status){
    ctx.fillStyle = "#252a2e"; ctx.fillText(k, x, 49);
    x += ctx.measureText(k).width + 12;
    ctx.fillStyle = "#111"; ctx.fillText(v, x, 49);
    x += ctx.measureText(v).width + 26;
  }
}

function drawPanel(rect){
  ctx.fillStyle = "#fbfcfa";
  ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
  ctx.strokeStyle = "#e2e7e1";
  ctx.strokeRect(rect.x+.5, rect.y+.5, rect.w-1, rect.h-1);
  ctx.fillStyle = "#252a2e";
  ctx.font = "700 19px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  ctx.fillText(rect.title, rect.x + 10, rect.y + 25);
}

function drawGrid(rect, stepPx=56){
  ctx.save();
  ctx.beginPath(); ctx.rect(rect.x,rect.y,rect.w,rect.h); ctx.clip();
  ctx.strokeStyle = "#e6ebe5";
  ctx.lineWidth = 1;
  for(let x=rect.x; x<=rect.x+rect.w; x+=stepPx){ ctx.beginPath(); ctx.moveTo(x,rect.y+34); ctx.lineTo(x,rect.y+rect.h); ctx.stroke(); }
  for(let y=rect.y+34; y<=rect.y+rect.h; y+=stepPx){ ctx.beginPath(); ctx.moveTo(rect.x,y); ctx.lineTo(rect.x+rect.w,y); ctx.stroke(); }
  ctx.restore();
}

function drawLidar(rect, f){
  drawPanel(rect); drawGrid(rect);
  const origin = {x: rect.x + rect.w/2, y: rect.y + rect.h - 38};
  const scale = Math.min(rect.w / 38, rect.h / 35);
  ctx.strokeStyle = "#4a514d"; ctx.lineWidth = 2;
  arrow(origin.x, origin.y, origin.x, origin.y - 90, "#4a514d");
  ctx.fillStyle = "#59635e"; ctx.font = "12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  ctx.fillText("forward", origin.x + 8, origin.y - 92);
  for(const p of f.lidar || []){
    const x = origin.x - p[1] * scale;
    const y = origin.y - p[0] * scale;
    if(x < rect.x || x > rect.x+rect.w || y < rect.y+34 || y > rect.y+rect.h) continue;
    const shade = Math.max(45, Math.min(190, 116 + p[2] * 28));
    ctx.fillStyle = `rgb(${shade},${shade},${shade})`;
    ctx.fillRect(x, y, 1.3, 1.3);
  }
  ctx.fillStyle = "#7b847f"; ctx.font = "12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  [0,5,10,15,20,25,30].forEach(m => ctx.fillText(m+"m", rect.x+2, origin.y - m*scale + 4));
}

function drawLocal(rect, f){
  drawPanel(rect); drawGrid(rect);
  const origin = {x: rect.x + 110, y: rect.y + rect.h/2 + 20};
  const scale = Math.min((rect.w-150)/35, (rect.h-70)/28);
  ctx.strokeStyle = "#bbc3bd"; ctx.lineWidth = 1.2;
  [5,10,15,20,25,30].forEach(r => { ctx.beginPath(); ctx.arc(origin.x, origin.y, r*scale, 0, Math.PI*2); ctx.stroke(); });
  arrow(origin.x, origin.y, origin.x + 95, origin.y, "#4a514d");
  arrow(origin.x, origin.y, origin.x, origin.y - 90, "#4a514d");
  ctx.fillStyle = "#59635e"; ctx.fillText("x forward", origin.x+100, origin.y+4); ctx.fillText("y left", origin.x+8, origin.y-96);
  for(const c of f.local || []){
    const x = origin.x + c.x * scale;
    const y = origin.y - c.y * scale;
    square(x,y,8,"#4358d8");
  }
  for(const c of f.fusion || []){
    const x = origin.x + c.x * scale;
    const y = origin.y - c.y * scale;
    square(x,y,8,"#ffcc1a");
  }
  legend(rect.x+rect.w-235, rect.y+45, [["#4358d8","LiDAR clusters"],["#ffcc1a","Fused cones"]]);
}

function drawGlobal(rect, f){
  drawPanel(rect);
  const vehicle = f.vehicle || {x:0,y:0,yaw:0};
  const baseScale = Math.min(rect.w/70, rect.h/42) * globalZoom;
  const cx = rect.x + rect.w/2 + globalPan.x;
  const cy = rect.y + rect.h/2 + globalPan.y;
  const sx = x => cx + (x - vehicle.x) * baseScale;
  const sy = y => cy - (y - vehicle.y) * baseScale;
  ctx.save(); ctx.beginPath(); ctx.rect(rect.x, rect.y+34, rect.w, rect.h-34); ctx.clip();
  ctx.strokeStyle = "#e1e7df"; ctx.lineWidth = 1;
  const grid = 5;
  for(let gx=Math.floor((vehicle.x-60)/grid)*grid; gx<vehicle.x+60; gx+=grid){ ctx.beginPath(); ctx.moveTo(sx(gx), rect.y+34); ctx.lineTo(sx(gx), rect.y+rect.h); ctx.stroke(); }
  for(let gy=Math.floor((vehicle.y-40)/grid)*grid; gy<vehicle.y+40; gy+=grid){ ctx.beginPath(); ctx.moveTo(rect.x, sy(gy)); ctx.lineTo(rect.x+rect.w, sy(gy)); ctx.stroke(); }
  drawPathUntil(f.t, sx, sy);
  for(const m of f.markers || []){
    const x = sx(m.x), y = sy(m.y);
    if(x < rect.x || x > rect.x+rect.w || y < rect.y+34 || y > rect.y+rect.h) continue;
    ctx.beginPath(); ctx.fillStyle = m.color || "#174bdb"; ctx.arc(x,y,7,0,Math.PI*2); ctx.fill();
    ctx.fillStyle = "#4b524e"; ctx.font = "12px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
    ctx.fillText("id " + m.id, x+10, y-7);
  }
  ctx.fillStyle = "#151817"; ctx.beginPath(); ctx.arc(sx(vehicle.x), sy(vehicle.y), 8, 0, Math.PI*2); ctx.fill();
  ctx.fillText("vehicle", sx(vehicle.x)+11, sy(vehicle.y)+4);
  arrow(sx(vehicle.x), sy(vehicle.y), sx(vehicle.x)+Math.cos(vehicle.yaw||0)*48, sy(vehicle.y)-Math.sin(vehicle.yaw||0)*48, "#151817");
  ctx.restore();
}

function drawPathUntil(t, sx, sy){
  const pts = path.filter(p => p.t <= t);
  if(pts.length < 2) return;
  ctx.strokeStyle = "#2d61b8"; ctx.lineWidth = 2.2; ctx.beginPath();
  pts.forEach((p,i) => i ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y)));
  ctx.stroke();
}

function drawStatus(rect, f){
  drawPanel(rect);
  const rows = [
    ["LiDAR raw points", String((f.lidar||[]).length)],
    ["LiDAR 3D detections", String((f.local||[]).length)],
    ["Fusion map cones", String((f.fusion||[]).length)],
    ["Confirmed map cones", String((f.markers||[]).length)],
    ["Elapsed", fmt(f.t||0,1)+" s"],
    ["Frame", String(f.frame||0)],
    ["Recorded frames", String(data.frame_count||frames.length)],
    ["Global map msgs", String((f.counts||{}).globalMapMessages||0)],
    ["LiDAR msgs", String((f.counts||{}).lidarMessages||0)],
    ["Mapping odom x/y", f.vehicle ? `${fmt(f.vehicle.x,3)}, ${fmt(f.vehicle.y,3)}` : "waiting"],
    ["Output file", shortPath(data.output_file || "results/slam_mapping_2026_05_10.html")]
  ];
  ctx.font = "16px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace";
  let y = rect.y + 62;
  for(const [k,v] of rows){
    ctx.fillStyle = "#4f5a54"; ctx.fillText(k.padEnd(22," "), rect.x+20, y);
    ctx.fillStyle = "#1b1f22"; ctx.fillText(": " + v, rect.x+250, y);
    y += 27;
  }
  const barX = rect.x+20, barY = rect.y+rect.h-42, barW = rect.w-40;
  ctx.fillStyle = "#dde5dd"; ctx.fillRect(barX, barY, barW, 10);
  ctx.fillStyle = "#2d61b8"; ctx.fillRect(barX, barY, barW * (index/Math.max(1,frames.length-1)), 10);
}

function draw(){
  const f = current();
  ctx.clearRect(0,0,width,height);
  drawTitle(f);
  const p = panels();
  drawLidar(p.lidar, f);
  drawGlobal(p.global, f);
  drawLocal(p.local, f);
  drawStatus(p.status, f);
  timeline.value = index;
  readout.textContent = `${index+1}/${frames.length}  ${fmt(f.t||0,1)}s`;
}

function fmt(v,d){ return Number(v||0).toFixed(d); }
function shortPath(value){
  const path = String(value || "");
  const marker = "/RacingBrain/";
  const at = path.indexOf(marker);
  if(at >= 0) return path.slice(at + marker.length);
  const parts = path.split("/").filter(Boolean);
  return parts.slice(-2).join("/") || path;
}
function square(x,y,s,color){ ctx.fillStyle=color; ctx.fillRect(x-s/2,y-s/2,s,s); }
function legend(x,y,items){ ctx.font = "14px ui-monospace,SFMono-Regular,Menlo,Consolas,monospace"; items.forEach((it,i)=>{ square(x,y+i*24,10,it[0]); ctx.fillStyle="#66706a"; ctx.fillText(it[1],x+18,y+5+i*24); });}
function arrow(x1,y1,x2,y2,color){ const a=Math.atan2(y2-y1,x2-x1); ctx.strokeStyle=color; ctx.fillStyle=color; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke(); ctx.beginPath(); ctx.moveTo(x2,y2); ctx.lineTo(x2-9*Math.cos(a-.45),y2-9*Math.sin(a-.45)); ctx.lineTo(x2-9*Math.cos(a+.45),y2-9*Math.sin(a+.45)); ctx.closePath(); ctx.fill(); }

function inGlobalPanel(e){ const r = panels().global; return e.clientX>=r.x && e.clientX<=r.x+r.w && e.clientY>=r.y && e.clientY<=r.y+r.h; }
canvas.addEventListener("pointerdown", e => { if(!inGlobalPanel(e)) return; dragging=true; dragLast={x:e.clientX,y:e.clientY}; canvas.setPointerCapture(e.pointerId); });
canvas.addEventListener("pointerup", () => { dragging=false; dragLast=null; });
canvas.addEventListener("pointermove", e => { if(!dragging||!dragLast) return; globalPan.x += e.clientX-dragLast.x; globalPan.y += e.clientY-dragLast.y; dragLast={x:e.clientX,y:e.clientY}; draw(); });
canvas.addEventListener("wheel", e => { if(!inGlobalPanel(e)) return; e.preventDefault(); globalZoom *= Math.exp(-e.deltaY*0.001); globalZoom = Math.max(.35, Math.min(8, globalZoom)); draw(); }, {passive:false});
playButton.addEventListener("click", () => { playing=!playing; playButton.textContent = playing ? "Pause" : "Play"; });
resetButton.addEventListener("click", () => { globalZoom=1; globalPan={x:0,y:0}; draw(); });
timeline.addEventListener("input", () => { index = Number(timeline.value); playing=false; playButton.textContent="Play"; draw(); });
window.addEventListener("resize", resize);

function tick(ts){
  const speed = Number(speedSelect.value || 1);
  if(playing && frames.length && ts-lastStep > 1000/(10*speed)){
    index = Math.min(frames.length-1, index+1);
    if(index >= frames.length-1){ playing=false; playButton.textContent="Play"; }
    lastStep = ts; draw();
  }
  requestAnimationFrame(tick);
}
resize(); requestAnimationFrame(tick);
</script>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-sec", type=float, default=90.0)
    parser.add_argument("--frame-period-sec", type=float, default=0.5)
    parser.add_argument("--max-lidar-points", type=int, default=700)
    args = parser.parse_args()

    rclpy.init()
    node = SlamHtmlCollector(frame_period_sec=args.frame_period_sec, max_lidar_points=args.max_lidar_points)
    deadline = time.time() + max(1.0, args.duration_sec)
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        data = node.snapshot()
        data["output_file"] = str(Path(args.output).resolve())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    write_html(Path(args.output), data)
    print(
        json.dumps(
            {
                "frames": data["frame_count"],
                "global_map_messages": data["global_map_messages"],
                "fusion_map_messages": data["fusion_map_messages"],
                "local_detection_messages": data["local_detection_messages"],
                "lidar_messages": data["lidar_messages"],
                "marker_count": data["marker_count"],
                "path_count": data["path_count"],
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
