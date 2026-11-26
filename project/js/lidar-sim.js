/* 線雷射洗車系統：點雲模擬器
   座標：X 車長（車頭 +X）、Y 車寬（左側 +Y）、Z 高度，單位公尺。
   流程：2D LiDAR 射線求交 -> 加雜訊與遺漏 -> 逐層堆疊 -> 轉世界座標 -> 融合 -> 5cm 整併 -> 兩點求法線 -> 噴塗路徑 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;
const MOBILE = matchMedia('(max-width: 760px)').matches;
const REDUCED = matchMedia('(prefers-reduced-motion: reduce)').matches || new URLSearchParams(location.search).has('instant');
const CAP = MOBILE ? 70000 : 150000;          // 每顆雷射的點數上限
const TOOL_LEN = 1.0;                          // 噴塗工具長度，TCP 在法蘭前 1000mm
const BUFFER = 0.5;                            // 預退 buffer 500mm

/* ---------- 亂數（固定種子，結果可重現） ---------- */
let seed = 20250806;
function rnd() { seed |= 0; seed = (seed + 0x6D2B79F5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }
function gauss() { let u = 0; while (u === 0) u = rnd(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rnd()); }

/* ---------- 參數化車體（七人座 MPV 輪廓，隱函數） ---------- */
const CAR = { L: 5.0, W: 1.9, H: 1.9, clr: 0.18 };
const TOP = [[-2.5, 0.72], [-2.46, 1.45], [-2.3, 1.84], [-1.2, 1.9], [0.2, 1.88], [0.75, 1.78], [1.55, 1.12], [2.25, 0.98], [2.45, 0.8], [2.5, 0.55]];
function topLin(x) {
  if (x <= TOP[0][0]) return TOP[0][1];
  for (let i = 1; i < TOP.length; i++) if (x <= TOP[i][0]) { const a = TOP[i - 1], b = TOP[i]; return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]); }
  return TOP[TOP.length - 1][1];
}
function topZ(x) { return (topLin(x - 0.05) + 2 * topLin(x) + topLin(x + 0.05)) / 4; }
function halfW(x) { const a = Math.abs(x); if (a <= 1.9) return CAR.W / 2; const t = (a - 1.9) / 0.6; return CAR.W / 2 - 0.24 * Math.pow(t, 2.5); }
function inCar(x, y, z) {
  if (x < -2.5 || x > 2.5 || z < 0 || z > 1.95) return false;
  const ay = Math.abs(y);
  if (ay > 0.96) return false;
  const h = halfW(x);
  if (z >= CAR.clr && ay <= h) {
    const u = ay / h, zt = topZ(x) * (1 - 0.05 * u * u * u * u);
    if (z <= zt) {
      let hz = h;
      if (z > 1.0) hz = h - 0.16 * Math.pow((z - 1.0) / 0.9, 1.5);
      else if (z < 0.45) hz = h - 0.05 * (0.45 - z) / 0.27;
      if (ay <= hz) return true;
    }
  }
  if (z <= 0.72 && ay >= 0.68 && ay <= 0.93) {           // 四個輪胎
    const dz = z - 0.36, d1 = x - 1.5, d2 = x + 1.5;
    if (d1 * d1 + dz * dz <= 0.1296 || d2 * d2 + dz * dz <= 0.1296) return true;
  }
  return false;
}
/* 距離校正標靶：位置已知的四根圓柱 */
const POST_R = 0.06, POST_H = 2.1;
const POSTS = [[3.3, 1.25], [3.3, -1.25], [-3.3, 1.25], [-3.3, -1.25]];

/* ---------- 射線求交 ---------- */
let hitT = 0, hitKind = 0;                     // kind：1 車體、2 標靶、3 地面
const BB = [-2.5, 2.5, -0.96, 0.96, 0, 1.95];
function cast(ox, oy, oz, dx, dy, dz, rmax) {
  let best = rmax, kind = 0;
  if (dz < -1e-6) { const t = -oz / dz; if (t < best) { best = t; kind = 3; } }
  const a = dx * dx + dy * dy;
  if (a > 1e-9) for (let i = 0; i < 4; i++) {
    const px = ox - POSTS[i][0], py = oy - POSTS[i][1];
    const b = px * dx + py * dy, c = px * px + py * py - POST_R * POST_R, disc = b * b - a * c;
    if (disc > 0) { const t = (-b - Math.sqrt(disc)) / a; if (t > 0.05 && t < best) { const z = oz + dz * t; if (z >= 0 && z <= POST_H) { best = t; kind = 2; } } }
  }
  let t0 = 0, t1 = best, o = [ox, oy, oz], d = [dx, dy, dz], ok = true;
  for (let k = 0; k < 3 && ok; k++) {
    if (Math.abs(d[k]) < 1e-9) { if (o[k] < BB[2 * k] || o[k] > BB[2 * k + 1]) ok = false; }
    else { let ta = (BB[2 * k] - o[k]) / d[k], tb = (BB[2 * k + 1] - o[k]) / d[k]; if (ta > tb) { const s = ta; ta = tb; tb = s; } if (ta > t0) t0 = ta; if (tb < t1) t1 = tb; if (t0 > t1) ok = false; }
  }
  if (ok) {
    const STEP = 0.015;
    for (let t = t0 + 1e-4; t <= t1; t += STEP) {
      if (inCar(ox + dx * t, oy + dy * t, oz + dz * t)) {
        let lo = Math.max(t0, t - STEP), hi = t;
        for (let k = 0; k < 10; k++) { const m = (lo + hi) / 2; if (inCar(ox + dx * m, oy + dy * m, oz + dz * m)) hi = m; else lo = m; }
        best = hi; kind = 1; break;
      }
    }
  }
  hitT = best; hitKind = kind;
  return kind !== 0;
}

/* ---------- Kabsch（平面剛體）：求 R、t 使 dst ≈ R·src + t ----------
   高度由線性馬達決定，所以對齊問題是繞 Z 軸旋轉加 XY 平移。
   2x2 共變異矩陣的 SVD 解可以化簡成一個 atan2。 */
export function kabsch2D(src, dst) {
  const n = src.length; let ax = 0, ay = 0, bx = 0, by = 0;
  for (let i = 0; i < n; i++) { ax += src[i][0]; ay += src[i][1]; bx += dst[i][0]; by += dst[i][1]; }
  ax /= n; ay /= n; bx /= n; by /= n;
  let sdot = 0, scross = 0;
  for (let i = 0; i < n; i++) { const px = src[i][0] - ax, py = src[i][1] - ay, qx = dst[i][0] - bx, qy = dst[i][1] - by; sdot += px * qx + py * qy; scross += px * qy - py * qx; }
  const th = Math.atan2(scross, sdot), c = Math.cos(th), s = Math.sin(th);
  const tx = bx - (c * ax - s * ay), ty = by - (s * ax + c * ay);
  let e = 0;
  for (let i = 0; i < n; i++) { const x = c * src[i][0] - s * src[i][1] + tx - dst[i][0], y = s * src[i][0] + c * src[i][1] + ty - dst[i][1]; e += x * x + y * y; }
  return { rot: th, tx, ty, rms: Math.sqrt(e / n) };
}

/* ---------- Three.js 場景 ---------- */
const host = $('simCanvas');
const statusEl = $('simStatus');
function fail(msg) { statusEl.textContent = msg; host.classList.add('failed'); }
window.addEventListener('error', (e) => { if (String(e.filename || '').includes('lidar-sim')) fail('模擬器發生錯誤：' + e.message); });

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
let renderer;
try { renderer = new THREE.WebGLRenderer({ antialias: !MOBILE, alpha: true, preserveDrawingBuffer: true }); }
catch (err) { fail('這台裝置無法啟用 WebGL，模擬器沒辦法顯示。'); throw err; }
const DPR = Math.min(window.devicePixelRatio || 1, 2);
renderer.setPixelRatio(DPR);
host.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 200);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = false;
controls.maxPolarAngle = Math.PI * 0.495;
let dirty = true, visible = true;
const invalidate = () => { dirty = true; };
controls.addEventListener('change', invalidate);

const world = new THREE.Group(); scene.add(world);     // 依模式重建的東西都放這裡
const overlay = new THREE.Group(); scene.add(overlay); // 點選標記、量測線
let grid = null;
function setGrid(size, div) {
  if (grid) { scene.remove(grid); grid.geometry.dispose(); }
  grid = new THREE.GridHelper(size, div, 0x3a3d44, 0x1e2024); grid.rotation.x = Math.PI / 2; scene.add(grid);
}
function resize() {
  const w = host.clientWidth, h = host.clientHeight; if (!w || !h) return;
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); invalidate();
}
new ResizeObserver(resize).observe(host);
new IntersectionObserver((es) => { visible = es[0].isIntersecting; }, { threshold: 0.02 }).observe(host);

function lineSeg(arr, color, opacity) {
  const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.Float32BufferAttribute(arr, 3));
  const l = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity })); l.frustumCulled = false; return l;
}
function dot(color, r) { return new THREE.Mesh(new THREE.SphereGeometry(r, 14, 10), new THREE.MeshBasicMaterial({ color })); }

/* ---------- 點雲容器 ---------- */
function makeCloud(key, label, hex) {
  const geo = new THREE.BufferGeometry();
  const pos = new Float32Array(CAP * 3), col = new Float32Array(CAP * 3);
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
  geo.setDrawRange(0, 0);
  geo.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1000);
  const obj = new THREE.Points(geo, new THREE.PointsMaterial({ size: (MOBILE ? 1.6 : 1.9) * DPR, sizeAttenuation: false, vertexColors: true, transparent: true }));
  obj.frustumCulled = false;
  return { key, label, hex, color: new THREE.Color(hex), pos, col, n: 0, loc: new Float32Array(CAP * 3), kind: new Uint8Array(CAP), geo, obj };
}
let clouds = [];
let mode = 'sim';
let zRange = [0, 2];
const cLow = new THREE.Color('#3b5bdb'), cMid = new THREE.Color('#b5d63f'), cHigh = new THREE.Color('#f0a948'), cTarget = new THREE.Color('#ff7a7a'), tmpC = new THREE.Color();
function paint(cl, i) {
  const byH = $('colorMode').value === 'height';
  if (cl.kind[i] === 2 && !byH) tmpC.copy(cTarget);
  else if (byH) { const t = Math.min(1, Math.max(0, (cl.pos[i * 3 + 2] - zRange[0]) / (zRange[1] - zRange[0]))); if (t < 0.5) tmpC.copy(cLow).lerp(cMid, t * 2); else tmpC.copy(cMid).lerp(cHigh, (t - 0.5) * 2); }
  else tmpC.copy(cl.color);
  cl.col[i * 3] = tmpC.r; cl.col[i * 3 + 1] = tmpC.g; cl.col[i * 3 + 2] = tmpC.b;
}
function repaint() { for (const cl of clouds) { for (let i = 0; i < cl.n; i++) paint(cl, i); cl.geo.attributes.color.needsUpdate = true; } invalidate(); }
function flush(cl) { cl.geo.setDrawRange(0, cl.n); cl.geo.attributes.position.needsUpdate = true; cl.geo.attributes.color.needsUpdate = true; }

/* ---------- 模擬參數與感測器 ---------- */
const P = {};
let sensors = [];
function readParams() {
  P.step = +$('pStep').value / 1000; P.res = +$('pRes').value; P.noise = +$('pNoise').value / 1000; P.drop = +$('pDrop').value / 100;
  P.sideD = +$('pSideD').value; P.sideX = +$('pSideX').value; P.topH = +$('pTopH').value;
  P.err = $('cErr').checked; P.gf = $('cGround').checked;
  P.z0 = 0.10; P.z1 = 2.0; P.x0 = -2.7; P.x1 = 2.7; P.rmax = 8; P.rmin = 0.12;
}
/* 側面雷射：水平掃描面，沿垂直線性軸逐層抬升。區域座標沿用原本 C# 的定義 x0 = r·sin(a)、y0 = -r·cos(a) */
function sideWorld(s, lx, ly, k, out, o) {
  const c = Math.cos(s.rot), sn = Math.sin(s.rot);
  out[o] = s.pos[0] + lx * c - ly * sn; out[o + 1] = s.pos[1] + lx * sn + ly * c; out[o + 2] = P.z0 + k * P.step;
}
/* 天花板雷射：掃描面轉 90 度成為 YZ 切面，沿天花板軸往車長方向逐層移動 */
function topWorld(s, lx, ly, k, out, o) { out[o] = P.x0 + k * P.step; out[o + 1] = lx; out[o + 2] = P.topH + ly; }

function buildSensors() {
  const nomL = { pos: [P.sideX, P.sideD], rot: 0 }, nomR = { pos: [-P.sideX, -P.sideD], rot: Math.PI };
  const eR = P.err ? { dx: 0.07, dy: -0.05, dr: 3 * DEG } : { dx: 0, dy: 0, dr: 0 };
  sensors = [
    { key: 'L', type: 'side', label: '左側雷射', nom: nomL, pos: nomL.pos.slice(), rot: nomL.rot, truePos: nomL.pos.slice(), trueRot: nomL.rot, layers: Math.floor((P.z1 - P.z0) / P.step) + 1, cloud: clouds[0] },
    { key: 'R', type: 'side', label: '右側雷射', nom: nomR, pos: nomR.pos.slice(), rot: nomR.rot, truePos: [nomR.pos[0] + eR.dx, nomR.pos[1] + eR.dy], trueRot: nomR.rot + eR.dr, layers: Math.floor((P.z1 - P.z0) / P.step) + 1, cloud: clouds[1] },
    { key: 'T', type: 'top', label: '天花板雷射', layers: Math.floor((P.x1 - P.x0) / P.step) + 1, cloud: clouds[2] },
  ];
}
let keep = 1, fanGeo = null, fanCount = 0;
function scanLayer(s, k, drawFan) {
  const cl = s.cloud, nr = Math.round(360 / P.res), phase = rnd() * P.res;
  let ox, oy, oz, c = 1, sn = 0;
  if (s.type === 'side') { c = Math.cos(s.trueRot); sn = Math.sin(s.trueRot); ox = s.truePos[0]; oy = s.truePos[1]; oz = P.z0 + k * P.step; }
  else { ox = P.x0 + k * P.step; oy = 0; oz = P.topH; }
  for (let i = 0; i < nr; i++) {
    const a = (i * P.res + phase) * DEG, ux = Math.sin(a), uy = -Math.cos(a);
    let dx, dy, dz;
    if (s.type === 'side') { dx = ux * c - uy * sn; dy = ux * sn + uy * c; dz = 0; } else { dx = 0; dy = ux; dz = uy; }
    if (!cast(ox, oy, oz, dx, dy, dz, P.rmax)) continue;
    if (rnd() < P.drop || rnd() > keep) continue;
    const r = hitT + gauss() * P.noise;
    if (r < P.rmin || r > P.rmax || cl.n >= CAP) continue;
    const lx = r * ux, ly = r * uy, j = cl.n * 3;
    if (s.type === 'side') sideWorld(s, lx, ly, k, cl.pos, j); else topWorld(s, lx, ly, k, cl.pos, j);
    if (s.type === 'top' && P.gf && cl.pos[j + 2] < 0.05) continue;       // 地面濾除
    cl.loc[j] = lx; cl.loc[j + 1] = ly; cl.loc[j + 2] = k; cl.kind[cl.n] = hitKind;
    paint(cl, cl.n); cl.n++;
    if (drawFan && fanCount < 600 && (i & 3) === 0) {
      const f = fanGeo.attributes.position.array, q = fanCount * 6;
      f[q] = ox; f[q + 1] = oy; f[q + 2] = oz; f[q + 3] = ox + dx * hitT; f[q + 4] = oy + dy * hitT; f[q + 5] = oz + dz * hitT; fanCount++;
    }
  }
  if (s.marker) s.marker.position.set(ox, oy, oz);
}
function reproject(s) {
  const cl = s.cloud;
  for (let i = 0; i < cl.n; i++) sideWorld(s, cl.loc[i * 3], cl.loc[i * 3 + 1], cl.loc[i * 3 + 2], cl.pos, i * 3);
  flush(cl);
}

/* ---------- 掃描排程（逐層動畫） ---------- */
let jobs = [], jobIdx = 0, scanning = false, scanStart = 0, perFrame = 1, onDone = null;
function clearGroup(g) { while (g.children.length) { const c = g.children.pop(); c.traverse((o) => { if (o.geometry) o.geometry.dispose(); if (o.material) o.material.dispose(); }); } }
function resetScene() {
  clearGroup(world); clearGroup(overlay); pathGroup = null; pathData = null; picks = []; toolAnim = null;
  $('pickInfo').textContent = '點一下點雲上的任一點，可以看座標與到各雷射的距離。';
  $('pathInfo').textContent = ''; $('calibInfo').textContent = '';
}
function startSim() {
  mode = 'sim'; seed = 20250806; readParams(); resetScene();
  clouds = [makeCloud('L', '左側雷射', '#b5d63f'), makeCloud('R', '右側雷射', '#4fc3d9'), makeCloud('T', '天花板雷射', '#f0a948')];
  clouds.forEach((c) => world.add(c.obj));
  buildSensors(); zRange = [0, 2]; setGrid(12, 24);
  // 標靶、線性軸、雷射本體
  const rails = [];
  for (const p of POSTS) { rails.push(p[0], p[1], 0, p[0], p[1], POST_H); }
  world.add(lineSeg(rails, 0xff7a7a, 0.55));
  const ax = [];
  for (const s of sensors) {
    s.marker = dot(s.cloud.hex, 0.07); world.add(s.marker);
    if (s.type === 'side') { ax.push(s.truePos[0], s.truePos[1], 0, s.truePos[0], s.truePos[1], P.z1 + 0.15); s.marker.position.set(s.truePos[0], s.truePos[1], P.z0); }
    else { ax.push(P.x0, 0, P.topH, P.x1, 0, P.topH); s.marker.position.set(P.x0, 0, P.topH); }
  }
  world.add(lineSeg(ax, 0x8a8d94, 0.7));
  fanGeo = new THREE.BufferGeometry(); fanGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(600 * 6), 3)); fanGeo.setDrawRange(0, 0);
  const fan = new THREE.LineSegments(fanGeo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.13 })); fan.frustumCulled = false; world.add(fan);
  // 點數預估，超過上限就等比例下取樣
  const est = (sensors[0].layers * 150 / P.res), estT = sensors[2].layers * 75 / P.res;
  keep = Math.min(1, CAP * 0.95 / Math.max(est, estT));
  jobs = [];
  for (const s of sensors) for (let k = 0; k < s.layers; k++) jobs.push({ s, k, f: k / s.layers });
  jobs.sort((a, b) => a.f - b.f);
  jobIdx = 0; scanning = true; scanStart = performance.now(); perFrame = Math.max(1, Math.ceil(jobs.length / 300));
  onDone = () => { if ($('cPath').checked) planAndDraw(); };
  applyVisibility(); setModeUI();
  if (REDUCED) finishNow();
}
function finishNow() { while (jobIdx < jobs.length) { const j = jobs[jobIdx++]; j.run ? j.run() : scanLayer(j.s, j.k, false); } endScan(); }
function endScan() {
  scanning = false; if (fanGeo) fanGeo.setDrawRange(0, 0);
  clouds.forEach(flush);
  const total = clouds.reduce((a, c) => a + c.n, 0);
  const ms = Math.round(performance.now() - scanStart);
  if (mode === 'sim') statusEl.textContent = `掃描完成：側面 ${sensors[0].layers} 層、天花板 ${sensors[2].layers} 層，共 ${total.toLocaleString()} 點` + (keep < 1 ? `（超過顯示上限，已下取樣到 ${Math.round(keep * 100)}%）` : '') + `，耗時 ${(ms / 1000).toFixed(1)} 秒。`;
  else statusEl.textContent = `實測資料載入完成：兩次掃描共 ${jobs.length} 層、${total.toLocaleString()} 點，層距 2 mm。`;
  if (onDone) onDone(); invalidate();
}
function stepScan() {
  const t0 = performance.now(); let done = 0; fanCount = 0;
  while (jobIdx < jobs.length && (done < perFrame || performance.now() - t0 < 6) && performance.now() - t0 < 28) {
    const j = jobs[jobIdx++]; j.run ? j.run() : scanLayer(j.s, j.k, true); done++;
  }
  if (fanGeo) { fanGeo.setDrawRange(0, fanCount * 2); fanGeo.attributes.position.needsUpdate = true; }
  clouds.forEach(flush);
  const total = clouds.reduce((a, c) => a + c.n, 0);
  statusEl.textContent = `掃描中 ${Math.round(jobIdx / jobs.length * 100)}%，已累積 ${total.toLocaleString()} 點`;
  if (jobIdx >= jobs.length) endScan();
}

/* ---------- 實測資料模式 ---------- */
let realData = null;
async function startReal() {
  mode = 'real'; resetScene(); setModeUI();
  statusEl.textContent = '載入實測資料中';
  try {
    if (!realData) { const r = await fetch('data/lidar-scan.json'); if (!r.ok) throw new Error(r.status); realData = await r.json(); }
  } catch (e) { fail('實測資料載入失敗，請重新整理頁面再試一次。'); return; }
  if (mode !== 'real') return;
  clouds = [makeCloud('A', '第一次掃描（右側雷射位）', '#4fc3d9'), makeCloud('B', '第二次掃描（頂部雷射位）', '#f0a948')];
  clouds.forEach((c) => world.add(c.obj));
  const step = realData.stepMm / 1000; zRange = [0.12, 0.12 + 35 * step]; setGrid(1.2, 24);
  sensors = []; jobs = [];
  [['right', clouds[0]], ['top', clouds[1]]].forEach(([name, cl]) => {
    const d = realData[name], s = { key: cl.key, type: 'real', label: cl.label, pos: [d.pos[0] / 1000, d.pos[1] / 1000], z: d.pos[2] / 1000, rot: d.rotDeg * DEG, cloud: cl };
    s.marker = dot(cl.hex, 0.008); s.marker.position.set(s.pos[0], s.pos[1], s.z); world.add(s.marker);
    world.add(lineSeg([0, 0, 0, s.pos[0], s.pos[1], s.z], cl.hex, 0.5));
    sensors.push(s);
    const c = Math.cos(s.rot), sn = Math.sin(s.rot);
    d.layers.forEach((flat, k) => jobs.push({ f: k / d.layers.length, run() {
      for (let i = 0; i < flat.length && cl.n < CAP; i += 2) {
        // 與原本 C# 程式相同的換算
        const a = flat[i] / 100 * DEG, r = flat[i + 1] / 1000, x0 = r * Math.sin(a), y0 = -r * Math.cos(a), j = cl.n * 3;
        cl.pos[j] = s.pos[0] + x0 * c - y0 * sn; cl.pos[j + 1] = s.pos[1] + x0 * sn + y0 * c; cl.pos[j + 2] = s.z + k * step;
        cl.loc[j] = x0; cl.loc[j + 1] = y0; cl.loc[j + 2] = k; cl.kind[cl.n] = 1; paint(cl, cl.n); cl.n++;
      }
      s.marker.position.z = s.z + k * step;
    } }));
  });
  jobs.sort((a, b) => a.f - b.f);
  fanGeo = null; jobIdx = 0; scanning = true; scanStart = performance.now(); perFrame = 1; onDone = null; keep = 1;
  applyVisibility();
  if (REDUCED) finishNow();
}

/* ---------- 標靶校正 ---------- */
function z0(v, d) { const t = v.toFixed(d); return +t === 0 ? (0).toFixed(d) : t; }
function calibrate() {
  if (mode !== 'sim' || scanning) return;
  const out = [];
  for (const s of sensors) {
    if (s.type !== 'side') continue;
    const cl = s.cloud, acc = POSTS.map(() => [0, 0, 0]), w = [0, 0, 0];
    for (let i = 0; i < cl.n; i++) {
      if (cl.kind[i] !== 2) continue;
      // 用名義姿態換算後找最近的標靶，做資料關聯
      const lx = cl.loc[i * 3], ly = cl.loc[i * 3 + 1], c = Math.cos(s.nom.rot), sn = Math.sin(s.nom.rot);
      w[0] = s.nom.pos[0] + lx * c - ly * sn; w[1] = s.nom.pos[1] + lx * sn + ly * c;
      let bi = -1, bd = 0.6 * 0.6;
      for (let p = 0; p < 4; p++) { const d = (w[0] - POSTS[p][0]) ** 2 + (w[1] - POSTS[p][1]) ** 2; if (d < bd) { bd = d; bi = p; } }
      if (bi >= 0) { acc[bi][0] += lx; acc[bi][1] += ly; acc[bi][2]++; }
    }
    const src = [], dst = [];
    acc.forEach((a, p) => {
      if (a[2] < 8) return;
      let cx = a[0] / a[2], cy = a[1] / a[2]; const d = Math.hypot(cx, cy);
      // 可見弧面的重心比圓心靠近雷射約 0.785r，往外補回去
      cx += cx / d * POST_R * 0.785; cy += cy / d * POST_R * 0.785;
      src.push([cx, cy]); dst.push(POSTS[p]);
    });
    if (src.length < 2) { out.push(`${s.label}：看得到的標靶不足，無法校正`); continue; }
    const k = kabsch2D(src, dst);
    s.pos = [k.tx, k.ty]; s.rot = k.rot; reproject(s);
    let dr = (k.rot - s.nom.rot) / DEG; dr = ((dr + 540) % 360) - 180;
    out.push(`${s.label}：用 ${src.length} 根標靶解出安裝偏差 ΔX ${z0((k.tx - s.nom.pos[0]) * 1000, 0)} mm、ΔY ${z0((k.ty - s.nom.pos[1]) * 1000, 0)} mm、Δθ ${z0(dr, 2)}°，殘差 ${(k.rms * 1000).toFixed(1)} mm`);
  }
  $('calibInfo').textContent = out.join('\n');
  if (pathGroup) planAndDraw(); invalidate();
}

/* ---------- 後處理：5cm 整併、兩點求法線、路徑 ---------- */
let pathGroup = null, pathData = null, toolAnim = null;
function quant(a, q) { a.sort((x, y) => x - y); return a[Math.min(a.length - 1, Math.floor(a.length * q))]; }
/* 沿輪廓每 5cm 整併成一個點，間隙太大就斷開成不同線段 */
function resample(chain, ds, gap) {
  const segs = []; let cur = [], acc = [0, 0, 0, 0], run = 0;
  const emit = () => { if (acc[3]) cur.push([acc[0] / acc[3], acc[1] / acc[3], acc[2] / acc[3]]); acc = [0, 0, 0, 0]; run = 0; };
  for (let i = 0; i < chain.length; i++) {
    if (i > 0) {
      const d = Math.hypot(chain[i][0] - chain[i - 1][0], chain[i][1] - chain[i - 1][1], chain[i][2] - chain[i - 1][2]);
      if (d > gap) { emit(); if (cur.length > 1) segs.push(cur); cur = []; }
      else { run += d; if (run >= ds) emit(); }
    }
    acc[0] += chain[i][0]; acc[1] += chain[i][1]; acc[2] += chain[i][2]; acc[3]++;
  }
  emit(); if (cur.length > 1) segs.push(cur);
  return segs;
}
/* 兩點決定姿態：切面內相鄰兩點給切線，切線在切面內轉 90 度就是工具方向 */
function poses(seg, plane, outward, off) {
  const res = [];
  for (let i = 0; i < seg.length; i++) {
    const a = seg[Math.max(0, i - 1)], b = seg[Math.min(seg.length - 1, i + 1)], p = seg[i];
    let n;
    if (plane === 'xz') { const tx = b[0] - a[0], tz = b[2] - a[2], l = Math.hypot(tx, tz) || 1; n = [-tz / l, 0, tx / l]; }
    else { const tx = b[0] - a[0], ty = b[1] - a[1], l = Math.hypot(tx, ty) || 1; n = [-ty / l, tx / l, 0]; }
    const o = outward(p); if (n[0] * o[0] + n[1] * o[1] + n[2] * o[2] < 0) n = [-n[0], -n[1], -n[2]];
    res.push({ s: p, n, tcp: [p[0] + n[0] * off, p[1] + n[1] * off, p[2] + n[2] * off] });
  }
  return res;
}
function planPaths() {
  const pitch = +$('pPitch').value / 1000, off = +$('pOff').value / 1000, zmin = +$('pZmin').value / 1000, SLAB = 0.025;
  const X = [], Y = [], Z = [];
  for (const cl of clouds) for (let i = 0; i < cl.n; i++) {
    const x = cl.pos[i * 3], y = cl.pos[i * 3 + 1], z = cl.pos[i * 3 + 2];
    if (z < 0.05) continue;
    let near = false; for (const p of POSTS) if ((x - p[0]) ** 2 + (y - p[1]) ** 2 < 0.09) { near = true; break; }   // 已知位置的標靶不算車體
    if (near) continue; X.push(x); Y.push(y); Z.push(z);
  }
  const n = X.length; if (n < 2000) return null;
  const xmin = quant(X.slice(), 0.003), xmax = quant(X.slice(), 0.997), zmax = quant(Z.slice(), 0.997), ymax = quant(Y.map(Math.abs), 0.997);
  const xc = (xmin + xmax) / 2, zc = 0.45;
  const arms = [{ name: '左手臂', s: 1, passes: [] }, { name: '右手臂', s: -1, passes: [] }];
  for (const arm of arms) {
    // 縱向車道：沿車長方向取 XZ 切面，一趟走完車頭 -> 車頂 -> 車尾
    let flip = false;
    for (let yy = pitch / 2; yy <= ymax - 0.32; yy += pitch) {
      const yl = arm.s * yy, NB = 440, bins = new Array(NB);
      for (let i = 0; i < n; i++) {
        if (Math.abs(Y[i] - yl) > SLAB || Z[i] < zmin) continue;
        let th = Math.atan2(Z[i] - zc, X[i] - xc) / DEG; if (th < -90) th += 360;
        const b = Math.floor((th + 20) * 2); if (b < 0 || b >= NB) continue;
        (bins[b] || (bins[b] = [])).push(Math.hypot(X[i] - xc, Z[i] - zc));
      }
      const chain = [];
      for (let b = 0; b < NB; b++) if (bins[b]) { const r = quant(bins[b], 0.8), th = (-20 + (b + 0.5) / 2) * DEG; chain.push([xc + r * Math.cos(th), yl, zc + r * Math.sin(th)]); }
      for (const seg of resample(chain, 0.05, 0.3)) {
        if (seg.length < 6) continue;
        const ps = poses(seg, 'xz', (p) => [p[0] - xc, 0, p[2] - zc], off); if (flip) ps.reverse(); flip = !flip;
        arm.passes.push({ kind: '車頭、車頂、車尾', ps });
      }
    }
    // 側面車道：依高度取 XY 切面，由上往下
    for (let zl = zmax - 0.3; zl >= zmin; zl -= pitch) {
      const x0 = xmin + 0.1, NB = Math.ceil((xmax - 0.1 - x0) / 0.025), bins = new Array(NB);
      for (let i = 0; i < n; i++) {
        if (Math.abs(Z[i] - zl) > SLAB || Y[i] * arm.s < 0.35) continue;
        const b = Math.floor((X[i] - x0) / 0.025); if (b < 0 || b >= NB) continue;
        (bins[b] || (bins[b] = [])).push(Math.abs(Y[i]));
      }
      const chain = [];
      for (let b = NB - 1; b >= 0; b--) if (bins[b]) chain.push([x0 + (b + 0.5) * 0.025, arm.s * quant(bins[b], 0.8), zl]);
      for (const seg of resample(chain, 0.05, 0.3)) {
        if (seg.length < 6) continue;
        const ps = poses(seg, 'xy', () => [0, arm.s, 0], off); if (flip) ps.reverse(); flip = !flip;
        arm.passes.push({ kind: '側面', ps });
      }
    }
  }
  return { arms, off };
}
function planAndDraw() {
  if (mode !== 'sim' || scanning) return;
  if (pathGroup) { world.remove(pathGroup); clearGroup(pathGroup); }
  const t0 = performance.now(); pathData = planPaths(); toolAnim = null;
  if (!pathData) { $('pathInfo').textContent = '點數太少，無法規劃路徑。'; return; }
  pathGroup = new THREE.Group(); world.add(pathGroup);
  const colors = [0xeaff8f, 0x8fe6ff], info = []; toolAnim = [];
  pathData.arms.forEach((arm, ai) => {
    const path = [], ticks = [], link = [], seq = []; let len = 0, count = 0, last = null;
    for (const pass of arm.passes) {
      const ps = pass.ps, a = ps[0], z = ps[ps.length - 1];
      const ap = [a.tcp[0] + a.n[0] * BUFFER, a.tcp[1] + a.n[1] * BUFFER, a.tcp[2] + a.n[2] * BUFFER];
      const rt = [z.tcp[0] + z.n[0] * BUFFER, z.tcp[1] + z.n[1] * BUFFER, z.tcp[2] + z.n[2] * BUFFER];
      if (last) link.push(...last, ...ap);
      link.push(...ap, ...a.tcp, ...z.tcp, ...rt); last = rt;
      seq.push({ tcp: ap, n: a.n });
      for (let i = 0; i < ps.length; i++) {
        if (i) { path.push(...ps[i - 1].tcp, ...ps[i].tcp); len += Math.hypot(ps[i].tcp[0] - ps[i - 1].tcp[0], ps[i].tcp[1] - ps[i - 1].tcp[1], ps[i].tcp[2] - ps[i - 1].tcp[2]); }
        if (i % 4 === 0) { const p = ps[i]; ticks.push(...p.tcp, p.tcp[0] + p.n[0] * 0.18, p.tcp[1] + p.n[1] * 0.18, p.tcp[2] + p.n[2] * 0.18); }
        seq.push(ps[i]); count++;
      }
      seq.push({ tcp: rt, n: z.n });
    }
    pathGroup.add(lineSeg(path, colors[ai], 1), lineSeg(ticks, colors[ai], 0.45), lineSeg(link, 0x9a9da4, 0.35));
    const g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const tool = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xffffff })); tool.frustumCulled = false;
    const tip = dot(colors[ai], 0.045), fl = dot(0xffffff, 0.06);
    pathGroup.add(tool, tip, fl);
    toolAnim.push({ seq, g, tip, fl, t: 0 });
    info.push(`${arm.name}：${arm.passes.length} 趟、${count} 個姿態、噴塗長度 ${len.toFixed(1)} m`);
  });
  $('pathInfo').textContent = info.join('\n') + `\n規劃耗時 ${Math.round(performance.now() - t0)} ms。白線是 1000 mm 噴塗工具，白球是法蘭，灰線是 500 mm 預退與趟間移動。`;
  clouds.forEach((c) => { c.obj.material.opacity = 0.5; });
  poseTool(REDUCED ? 0.35 : 0); invalidate();
}
function poseTool(frac) {
  if (!toolAnim) return;
  for (const a of toolAnim) {
    if (!a.seq.length) continue;
    if (frac !== undefined) a.t = frac * (a.seq.length - 1);
    const i = Math.floor(a.t) % a.seq.length, j = Math.min(a.seq.length - 1, i + 1), f = a.t - Math.floor(a.t), A = a.seq[i], B = a.seq[j];
    const arr = a.g.attributes.position.array;
    for (let k = 0; k < 3; k++) { const tcp = A.tcp[k] + (B.tcp[k] - A.tcp[k]) * f, nn = A.n[k] + (B.n[k] - A.n[k]) * f; arr[k] = tcp; arr[k + 3] = tcp + nn * TOOL_LEN; }
    a.g.attributes.position.needsUpdate = true;
    a.tip.position.set(arr[0], arr[1], arr[2]); a.fl.position.set(arr[3], arr[4], arr[5]);
  }
}

/* ---------- 點選與量測 ---------- */
const ray = new THREE.Raycaster(); let picks = [], downAt = null;
renderer.domElement.addEventListener('pointerdown', (e) => { downAt = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 6 || scanning) return;
  const rc = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2((e.clientX - rc.left) / rc.width * 2 - 1, -(e.clientY - rc.top) / rc.height * 2 + 1);
  ray.setFromCamera(ndc, camera);
  const dist = camera.position.distanceTo(controls.target);
  ray.params.Points.threshold = dist * (MOBILE ? 0.02 : 0.009);
  const hits = ray.intersectObjects(clouds.filter((c) => c.obj.visible).map((c) => c.obj), false);
  if (!hits.length) return;
  hits.sort((a, b) => a.distanceToRay - b.distanceToRay);
  const h = hits[0], cl = clouds.find((c) => c.obj === h.object), i = h.index;
  const p = [cl.pos[i * 3], cl.pos[i * 3 + 1], cl.pos[i * 3 + 2]];
  const measuring = $('cMeasure').checked;
  if (!measuring || picks.length >= 2) { clearGroup(overlay); picks = []; }
  picks.push(p);
  const m = dot(0x7dff6b, dist * 0.008); m.position.set(...p); overlay.add(m);
  const mm = (v) => (v * 1000).toFixed(1);
  
  let txt = `座標 (X, Y, Z)：${mm(p[0])}, ${mm(p[1])}, ${mm(p[2])} mm\n來源：${cl.label}` + (cl.kind[i] === 2 ? '（校正標靶）' : '');
  for (const s of sensors) {
    let d;
    if (s.type === 'top') d = Math.hypot(p[1], p[2] - P.topH); else d = Math.hypot(p[0] - s.pos[0], p[1] - s.pos[1]);
    txt += `\n離${s.label.replace(/（.*/, '')}：${mm(d)} mm` + (s.cloud === cl ? '（量到這一點的雷射）' : '');
  }
  if (measuring && picks.length === 2) {
    const [a, b] = picks; overlay.add(lineSeg([...a, ...b], 0xff5555, 1));
    txt = `兩點距離：${mm(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]))} mm\nΔX ${mm(b[0] - a[0])}、ΔY ${mm(b[1] - a[1])}、ΔZ ${mm(b[2] - a[2])} mm\n拿這個數字去對實物上同一段的捲尺量測值，就能驗證點雲比例對不對。`;
  } else if (measuring) txt += '\n量測模式：再點第二個點。';
  $('pickInfo').textContent = txt; invalidate();
});

/* ---------- 匯出 ---------- */
function download(name, text) { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type: 'text/plain' })); a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000); }
function exportCloud(fmt) {
  const rows = []; let n = 0;
  for (const cl of clouds) { if (!cl.obj.visible) continue; for (let i = 0; i < cl.n; i++) { const j = i * 3; n++;
    if (fmt === 'csv') rows.push(`${cl.pos[j].toFixed(4)},${cl.pos[j + 1].toFixed(4)},${cl.pos[j + 2].toFixed(4)},${cl.key}`);
    else rows.push(`${cl.pos[j].toFixed(4)} ${cl.pos[j + 1].toFixed(4)} ${cl.pos[j + 2].toFixed(4)} ${Math.round(cl.col[j] * 255)} ${Math.round(cl.col[j + 1] * 255)} ${Math.round(cl.col[j + 2] * 255)}`); } }
  if (fmt === 'csv') download(`pointcloud_${mode}.csv`, 'X,Y,Z,sensor\n' + rows.join('\n'));
  else download(`pointcloud_${mode}.ply`, `ply\nformat ascii 1.0\ncomment unit meter\nelement vertex ${n}\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n` + rows.join('\n') + '\n');
}
function exportPath() {
  if (!pathData) { $('pathInfo').textContent = '請先規劃路徑。'; return; }
  const rows = ['arm,pass,region,tcp_x,tcp_y,tcp_z,tool_x,tool_y,tool_z,flange_x,flange_y,flange_z'];
  pathData.arms.forEach((arm) => arm.passes.forEach((pass, pi) => pass.ps.forEach((p) => {
    rows.push([arm.s > 0 ? 'L' : 'R', pi + 1, pass.kind === '側面' ? 'side' : 'head-top-back', ...p.tcp.map((v) => v.toFixed(4)), ...p.n.map((v) => (-v).toFixed(4)), ...p.tcp.map((v, k) => (v + p.n[k] * TOOL_LEN).toFixed(4))].join(','));
  })));
  download('spray_path.csv', rows.join('\n'));
}

/* ---------- 介面 ---------- */
function applyVisibility() {
  const map = mode === 'sim' ? ['vA', 'vB', 'vC'] : ['vA', 'vB'];
  clouds.forEach((c, i) => { c.obj.visible = $(map[i]).checked; });
  if (pathGroup) pathGroup.visible = $('cPath').checked;
  invalidate();
}
function setModeUI() {
  document.querySelectorAll('[data-mode]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.mode === mode)));
  document.querySelectorAll('.only-sim').forEach((el) => { el.hidden = mode !== 'sim'; });
  $('lA').textContent = mode === 'sim' ? '左側雷射' : '第一次掃描';
  $('lB').textContent = mode === 'sim' ? '右側雷射' : '第二次掃描';
  $('dA').style.background = mode === 'sim' ? '#b5d63f' : '#4fc3d9'; $('dB').style.background = mode === 'sim' ? '#4fc3d9' : '#f0a948';
  resetView();
}
function resetView() {
  if (mode === 'sim') { camera.position.set(6.4, -7.2, 4.6); controls.target.set(0, 0, 0.8); controls.minDistance = 1; controls.maxDistance = 40; }
  else { camera.position.set(-0.40, -0.10, 0.40); controls.target.set(-0.02, 0.29, 0.15); controls.minDistance = 0.08; controls.maxDistance = 4; }
  if (host.clientWidth < 560) camera.position.sub(controls.target).multiplyScalar(1.45).add(controls.target);   // 窄螢幕退遠一點
  camera.near = mode === 'sim' ? 0.05 : 0.005; camera.updateProjectionMatrix(); controls.update(); invalidate();
}
const fmt = { pStep: (v) => `${v} mm`, pRes: (v) => `${(+v).toFixed(2)}°`, pNoise: (v) => `±${v} mm`, pDrop: (v) => `${v}%`, pSideD: (v) => `${(+v).toFixed(1)} m`, pSideX: (v) => `${(+v).toFixed(1)} m`, pTopH: (v) => `${(+v).toFixed(1)} m`, pPitch: (v) => `${v} mm`, pOff: (v) => `${v} mm`, pZmin: (v) => `${v} mm` };
Object.keys(fmt).forEach((id) => {
  const el = $(id), out = $(id + 'V'), show = () => { out.textContent = fmt[id](el.value); };
  el.addEventListener('input', show); show();
  el.addEventListener('change', () => { if (['pPitch', 'pOff', 'pZmin'].includes(id)) { if (pathGroup || $('cPath').checked) planAndDraw(); } else if (mode === 'sim') startSim(); });
});
['cErr', 'cGround'].forEach((id) => $(id).addEventListener('change', () => { if (mode === 'sim') startSim(); }));
['vA', 'vB', 'vC'].forEach((id) => $(id).addEventListener('change', applyVisibility));
$('cPath').addEventListener('change', () => { if ($('cPath').checked && !pathGroup) planAndDraw(); else { clouds.forEach((c) => { c.obj.material.opacity = $('cPath').checked ? 0.5 : 1; }); applyVisibility(); } });
$('cMeasure').addEventListener('change', () => { clearGroup(overlay); picks = []; $('pickInfo').textContent = $('cMeasure').checked ? '量測模式：依序點兩個點，會畫出紅線並算出距離。' : '點一下點雲上的任一點，可以看座標與到各雷射的距離。'; invalidate(); });
$('colorMode').addEventListener('change', repaint);
$('bRescan').addEventListener('click', () => (mode === 'sim' ? startSim() : startReal()));
$('bCalib').addEventListener('click', calibrate);
$('bView').addEventListener('click', resetView);
$('bCsv').addEventListener('click', () => exportCloud('csv'));
$('bPly').addEventListener('click', () => exportCloud('ply'));
$('bPath').addEventListener('click', exportPath);
document.querySelectorAll('[data-mode]').forEach((b) => b.addEventListener('click', () => { if (b.dataset.mode === mode) return; b.dataset.mode === 'sim' ? startSim() : startReal(); }));

/* ---------- 主迴圈：只在看得到而且有變化時才畫 ---------- */
let lastT = 0;
function frame(t) {
  requestAnimationFrame(frame);
  if (!visible || document.hidden) { lastT = t; return; }
  if (scanning) { stepScan(); dirty = true; }
  if (toolAnim && !REDUCED && pathGroup && pathGroup.visible && !scanning) {
    const dt = Math.min(0.05, (t - lastT) / 1000);
    for (const a of toolAnim) { a.t += dt * 14; if (a.t >= a.seq.length - 1) a.t = 0; }
    poseTool(); dirty = true;
  }
  lastT = t;
  if (dirty) { dirty = false; renderer.render(scene, camera); }
}
resize();
startSim();
requestAnimationFrame(frame);
window.__lidarSim = { get clouds() { return clouds; }, get scanning() { return scanning; }, startReal, startSim, planAndDraw, calibrate };
