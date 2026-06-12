#!/usr/bin/env python3
"""
Jupiter IMU 상태 Web GUI (브라우저로 접속)
- /imu/data_raw (sensor_msgs/Imu)
- /imu/mag      (sensor_msgs/MagneticField)

실행:
    source /opt/ros/humble/setup.bash
    python3 imu_state_web.py            # 기본 0.0.0.0:5000
    python3 imu_state_web.py --port 8080

접속:
    http://<jetson-ip>:5000/
"""
import argparse
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, MagneticField


# -------------------- ROS node --------------------

class ImuStateNode(Node):
    def __init__(self):
        super().__init__('imu_state_web')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)
        self.create_subscription(MagneticField, '/imu/mag', self.cb_mag, qos)

        self.lock = threading.Lock()
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.mx = self.my = self.mz = 0.0
        self.imu_count = 0
        self.mag_count = 0

    def cb_imu(self, msg: Imu):
        with self.lock:
            self.ax = msg.linear_acceleration.x
            self.ay = msg.linear_acceleration.y
            self.az = msg.linear_acceleration.z
            self.gx = msg.angular_velocity.x
            self.gy = msg.angular_velocity.y
            self.gz = msg.angular_velocity.z
            self.imu_count += 1

    def cb_mag(self, msg: MagneticField):
        with self.lock:
            self.mx = msg.magnetic_field.x
            self.my = msg.magnetic_field.y
            self.mz = msg.magnetic_field.z
            self.mag_count += 1

    def snapshot(self):
        with self.lock:
            return dict(
                ax=self.ax, ay=self.ay, az=self.az,
                gx=self.gx, gy=self.gy, gz=self.gz,
                mx=self.mx, my=self.my, mz=self.mz,
                imu_count=self.imu_count,
                mag_count=self.mag_count,
            )


def compute_attitude(ax, ay, az, mx, my, mz):
    roll = math.atan2(ay, math.copysign(math.sqrt(az * az + 0.01 * ax * ax), az))
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    mx_h = mx * cp + mz * sp
    my_h = mx * sr * sp + my * cr - mz * sr * cp
    yaw = math.atan2(-my_h, mx_h)
    return roll, pitch, yaw


# -------------------- HTTP server --------------------

INDEX_HTML = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<title>Jupiter IMU State</title>
<style>
  :root { color-scheme: dark; }
  body { font-family: system-ui, sans-serif; background:#111; color:#eee;
         margin:0; padding:14px; }
  h1 { font-size:18px; margin:0 0 12px; color:#bbb; font-weight:500; }
  .grid { display:grid; grid-template-columns: 1fr 1fr 1fr; gap:14px;
          margin-bottom:14px; }
  .card { background:#1c1c1c; border:1px solid #333; border-radius:8px;
          padding:12px; }
  .card h2 { margin:0 0 6px; font-size:12px; color:#888;
             font-weight:500; text-transform:uppercase; letter-spacing:1px; }
  .big { font-size:38px; font-weight:600; font-variant-numeric: tabular-nums; }
  .roll  .big { color:#5fb3ff; }
  .pitch .big { color:#5fff9b; }
  .yaw   .big { color:#ffc15f; }
  .canvases { display:flex; gap:14px; flex-wrap:wrap; margin-bottom:14px; }
  canvas { background:#000; border:1px solid #333; border-radius:8px; }
  .raw { display:grid; grid-template-columns: 1fr 1fr 1fr; gap:14px; }
  .raw .card pre { margin:0; font-family:ui-monospace,Menlo,Consolas,monospace;
                   font-size:13px; color:#ddd; white-space:pre; }
  .status { margin-top:14px; color:#888; font-size:13px;
            font-family:ui-monospace,Menlo,Consolas,monospace; }
  .ok { color:#5fff9b; } .bad { color:#ff6060; }
</style>
</head>
<body>
<h1>Jupiter IMU State <span id="hint" style="color:#555">/imu/data_raw + /imu/mag</span></h1>

<div class="grid">
  <div class="card roll">  <h2>Roll</h2>  <div class="big" id="roll">--</div></div>
  <div class="card pitch"> <h2>Pitch</h2> <div class="big" id="pitch">--</div></div>
  <div class="card yaw">   <h2>Yaw / Heading</h2> <div class="big" id="yaw">--</div></div>
</div>

<div class="canvases">
  <canvas id="att" width="260" height="260"></canvas>
  <canvas id="cmp" width="260" height="260"></canvas>
</div>

<div class="raw">
  <div class="card"><h2>Angular velocity [rad/s]</h2><pre id="gyro">--</pre></div>
  <div class="card"><h2>Linear acceleration [m/s²]</h2><pre id="acc">--</pre></div>
  <div class="card"><h2>Magnetic field [µT]</h2><pre id="mag">--</pre></div>
</div>

<div class="status" id="status">connecting...</div>

<script>
const att = document.getElementById('att');
const cmp = document.getElementById('cmp');
const aCtx = att.getContext('2d');
const cCtx = cmp.getContext('2d');

let sRoll=null, sPitch=null, sYaw=null;

function blend(prev, next, a) {
  return prev === null ? next : (1-a)*prev + a*next;
}
function blendAngle(prev, next, a) {
  if (prev === null) return next;
  let d = Math.atan2(Math.sin(next-prev), Math.cos(next-prev));
  let v = prev + a*d;
  return Math.atan2(Math.sin(v), Math.cos(v));
}

function drawAttitude(roll, pitch) {
  const w = att.width, h = att.height;
  const cx = w/2, cy = h/2;
  const r  = Math.min(w,h)/2 - 8;
  const pitchPx = Math.max(-1, Math.min(1, (pitch*180/Math.PI)/45)) * r * 0.6;

  aCtx.save();
  aCtx.clearRect(0,0,w,h);
  aCtx.beginPath(); aCtx.arc(cx,cy,r,0,Math.PI*2); aCtx.clip();

  aCtx.translate(cx, cy);
  aCtx.rotate(-roll);
  const big = r*2.5;
  aCtx.fillStyle = '#1e6fd1';
  aCtx.fillRect(-big, -big, big*2, big + pitchPx);
  aCtx.fillStyle = '#8a5a2b';
  aCtx.fillRect(-big, pitchPx, big*2, big);
  aCtx.strokeStyle = 'white'; aCtx.lineWidth = 2;
  aCtx.beginPath(); aCtx.moveTo(-r, pitchPx); aCtx.lineTo(r, pitchPx); aCtx.stroke();

  // pitch ladder
  aCtx.strokeStyle = 'rgba(255,255,255,0.7)'; aCtx.lineWidth = 1;
  aCtx.fillStyle = 'white'; aCtx.font = '10px sans-serif';
  aCtx.textAlign = 'center';
  for (let p=-60; p<=60; p+=10) {
    if (p===0) continue;
    const y = pitchPx - (p/45)*r*0.6;
    const len = (p%30===0) ? 40 : 20;
    aCtx.beginPath(); aCtx.moveTo(-len, y); aCtx.lineTo(len, y); aCtx.stroke();
    if (p%30===0) aCtx.fillText(p, -len-10, y+3);
  }
  aCtx.restore();

  // fixed aircraft symbol
  aCtx.strokeStyle = 'yellow'; aCtx.lineWidth = 3;
  aCtx.beginPath(); aCtx.moveTo(cx-50,cy); aCtx.lineTo(cx-15,cy);
  aCtx.moveTo(cx+15,cy); aCtx.lineTo(cx+50,cy); aCtx.stroke();
  aCtx.fillStyle = 'yellow';
  aCtx.beginPath(); aCtx.arc(cx,cy,3,0,Math.PI*2); aCtx.fill();

  aCtx.strokeStyle = '#666'; aCtx.lineWidth = 2;
  aCtx.beginPath(); aCtx.arc(cx,cy,r,0,Math.PI*2); aCtx.stroke();

  aCtx.fillStyle = '#888'; aCtx.font = '11px sans-serif';
  aCtx.textAlign = 'center';
  aCtx.fillText('Attitude', cx, 14);
}

function drawCompass(yaw) {
  const w = cmp.width, h = cmp.height;
  const cx = w/2, cy = h/2;
  const r  = Math.min(w,h)/2 - 12;

  cCtx.clearRect(0,0,w,h);
  cCtx.strokeStyle = '#666'; cCtx.lineWidth = 2;
  cCtx.beginPath(); cCtx.arc(cx,cy,r,0,Math.PI*2); cCtx.stroke();

  cCtx.strokeStyle = '#555';
  for (let deg=0; deg<360; deg+=30) {
    const ang = deg*Math.PI/180 - yaw - Math.PI/2;
    cCtx.beginPath();
    cCtx.moveTo(cx + (r-4)*Math.cos(ang), cy + (r-4)*Math.sin(ang));
    cCtx.lineTo(cx + r    *Math.cos(ang), cy + r    *Math.sin(ang));
    cCtx.stroke();
  }

  const labels = [['N',0,'#ff5050'],['E',90,'white'],['S',180,'white'],['W',270,'white']];
  cCtx.font = 'bold 14px sans-serif'; cCtx.textAlign = 'center'; cCtx.textBaseline = 'middle';
  for (const [lab,deg,col] of labels) {
    const ang = deg*Math.PI/180 - yaw - Math.PI/2;
    cCtx.fillStyle = col;
    cCtx.fillText(lab, cx + (r-18)*Math.cos(ang), cy + (r-18)*Math.sin(ang));
  }

  cCtx.fillStyle = 'yellow';
  cCtx.beginPath();
  cCtx.moveTo(cx, cy-r+2);
  cCtx.lineTo(cx-8, cy-r+18);
  cCtx.lineTo(cx+8, cy-r+18);
  cCtx.closePath(); cCtx.fill();

  let deg = yaw*180/Math.PI; if (deg<0) deg+=360;
  cCtx.fillStyle = 'white'; cCtx.font = 'bold 18px sans-serif';
  cCtx.fillText(deg.toFixed(1)+'°', cx, cy);

  cCtx.fillStyle = '#888'; cCtx.font = '11px sans-serif';
  cCtx.fillText('Heading (mag)', cx, 14);
}

function fmt(v, w=8, d=3) {
  let s = (v>=0?'+':'') + v.toFixed(d);
  while (s.length < w) s = ' ' + s;
  return s;
}

async function tick() {
  try {
    const r = await fetch('/state', {cache:'no-store'});
    if (!r.ok) throw new Error('http '+r.status);
    const s = await r.json();

    sRoll  = blend(sRoll,  s.roll,  0.25);
    sPitch = blend(sPitch, s.pitch, 0.25);
    sYaw   = blendAngle(sYaw, s.yaw, 0.25);

    let yd = sYaw*180/Math.PI; if (yd<0) yd+=360;
    document.getElementById('roll').textContent  = (sRoll*180/Math.PI).toFixed(1)+'°';
    document.getElementById('pitch').textContent = (sPitch*180/Math.PI).toFixed(1)+'°';
    document.getElementById('yaw').textContent   = yd.toFixed(1)+'°';

    document.getElementById('gyro').textContent =
      'x='+fmt(s.gx,9,4)+'   y='+fmt(s.gy,9,4)+'   z='+fmt(s.gz,9,4);
    document.getElementById('acc').textContent =
      'x='+fmt(s.ax,8,3)+'   y='+fmt(s.ay,8,3)+'   z='+fmt(s.az,8,3);
    const bmag = Math.sqrt(s.mx*s.mx+s.my*s.my+s.mz*s.mz)*1e6;
    document.getElementById('mag').textContent =
      'x='+fmt(s.mx*1e6,8,2)+'   y='+fmt(s.my*1e6,8,2)+'   z='+fmt(s.mz*1e6,8,2)+
      '\n|B|='+fmt(bmag,7,2)+'  µT';

    drawAttitude(sRoll, sPitch);
    drawCompass(sYaw);

    const okI = s.imu_count > 0;
    const okM = s.mag_count > 0;
    document.getElementById('status').innerHTML =
      '/imu/data_raw : <span class="'+(okI?'ok':'bad')+'">'+(okI?'OK':'NO DATA')+'</span>'+
      '  ('+s.imu_hz.toFixed(1)+' Hz, n='+s.imu_count+')   '+
      '/imu/mag : <span class="'+(okM?'ok':'bad')+'">'+(okM?'OK':'NO DATA')+'</span>'+
      '  ('+s.mag_hz.toFixed(1)+' Hz, n='+s.mag_count+')';
  } catch(e) {
    document.getElementById('status').textContent = 'fetch error: '+e;
  }
}
setInterval(tick, 100);
tick();
</script>
</body>
</html>
"""


class StateServer:
    def __init__(self, node: ImuStateNode):
        self.node = node
        self.last_imu = 0
        self.last_mag = 0
        self.last_t = 0.0
        self.imu_hz = 0.0
        self.mag_hz = 0.0

    def state_json(self):
        import time
        s = self.node.snapshot()
        roll, pitch, yaw = compute_attitude(
            s['ax'], s['ay'], s['az'],
            s['mx'], s['my'], s['mz'],
        )
        t = time.monotonic()
        if self.last_t == 0.0:
            self.last_t = t
            self.last_imu = s['imu_count']
            self.last_mag = s['mag_count']
        dt = t - self.last_t
        if dt >= 1.0:
            self.imu_hz = (s['imu_count'] - self.last_imu) / dt
            self.mag_hz = (s['mag_count'] - self.last_mag) / dt
            self.last_imu = s['imu_count']
            self.last_mag = s['mag_count']
            self.last_t = t

        return {
            **s,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'imu_hz': self.imu_hz, 'mag_hz': self.mag_hz,
        }


def make_handler(server: StateServer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a, **kw):
            pass  # silence access log

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/index'):
                body = INDEX_HTML.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == '/state':
                body = json.dumps(server.state_json()).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)
    return Handler


def spin_ros(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=5000)
    args = p.parse_args()

    rclpy.init()
    node = ImuStateNode()
    t = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    t.start()

    state = StateServer(node)
    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    print(f"[imu_state_web] listening on http://{args.host}:{args.port}/  (Ctrl+C to stop)")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down...")
    finally:
        httpd.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
