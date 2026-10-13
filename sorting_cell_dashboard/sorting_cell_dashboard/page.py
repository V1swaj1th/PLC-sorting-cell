"""The dashboard page: one self-contained HTML file, no external resources."""

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sorting Cell Scene</title>
<style>
:root {
  --bg: #f4f3ef; --card: #fcfcfb; --ink: #1d2126; --muted: #5d646d; --line: #d9d7d0;
  --table: #8a8d91; --accent: #2a78d6; --ok: #1f8a4c; --bad: #c4372d; --warn: #b7791f;
  --reach-no: rgba(196, 55, 45, 0.30); --chip: #ebe9e3;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #15181c; --card: #1e2227; --ink: #e8e6e1; --muted: #9aa1a9; --line: #343a41;
    --table: #4d5257; --accent: #5b9ce8; --ok: #4cc27f; --bad: #ef6b5f; --warn: #e0a84c;
    --reach-no: rgba(239, 107, 95, 0.35); --chip: #2a2f35;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink);
       font: 14px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
header { display: flex; align-items: center; gap: 14px; flex-wrap: wrap; padding: 12px 18px;
         border-bottom: 1px solid var(--line); background: var(--card); }
header h1 { font-size: 16px; margin: 0; font-weight: 650; letter-spacing: .01em; }
.pill { display: inline-flex; align-items: center; gap: 6px; padding: 2px 10px; border-radius: 99px;
        background: var(--chip); font-weight: 600; font-size: 12.5px; text-transform: uppercase; letter-spacing: .04em; }
.pill::before { content: ""; width: 8px; height: 8px; border-radius: 50%; background: var(--muted); }
.pill.ready::before, .pill.done::before { background: var(--ok); }
.pill.running::before { background: var(--accent); animation: blink 1s infinite; }
.pill.fault::before { background: var(--bad); }
.pill.waiting::before, .pill.stopping::before, .pill.enabling::before { background: var(--warn); }
.pill.disabled::before { background: var(--muted); }
#dstate { text-transform: none; letter-spacing: 0; }
.bar { display: inline-block; height: 8px; border-radius: 4px; background: var(--accent); vertical-align: middle; }
.bar.hot { background: var(--bad); }
@keyframes blink { 50% { opacity: .3; } }
.meta { color: var(--muted); font-size: 13px; }
main { display: grid; grid-template-columns: minmax(0, 1fr) 360px; align-items: start; gap: 16px; padding: 16px; max-width: 1400px; margin: 0 auto; }
@media (max-width: 900px) { main { grid-template-columns: 1fr; padding: 12px; } }
.card { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 14px; }
.card h2 { font-size: 12px; text-transform: uppercase; letter-spacing: .06em; color: var(--muted); margin: 0 0 10px; font-weight: 650; }
.side { display: flex; flex-direction: column; gap: 12px; }
#cv { width: 100%; display: block; border-radius: 6px; touch-action: none; cursor: crosshair; }
.hint { color: var(--muted); font-size: 12.5px; margin: 8px 0 0; }
.row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
button { font: inherit; padding: 6px 12px; border-radius: 7px; border: 1px solid var(--line);
         background: var(--card); color: var(--ink); cursor: pointer; }
button:hover:not(:disabled) { border-color: var(--accent); }
button:disabled { opacity: .45; cursor: not-allowed; }
button.primary { background: var(--accent); border-color: var(--accent); color: #fff; font-weight: 600; }
button.small { padding: 2px 8px; font-size: 12.5px; }
select, input { font: inherit; padding: 4px 6px; border-radius: 6px; border: 1px solid var(--line);
                background: var(--card); color: var(--ink); }
input[type=number] { width: 78px; }
input.path { flex: 1; min-width: 140px; }
#msg { margin-top: 10px; padding: 8px 10px; border-radius: 7px; background: var(--chip); min-height: 2.2em; word-break: break-word; }
#msg.ok { border-left: 3px solid var(--ok); }
#msg.bad { border-left: 3px solid var(--bad); }
table { width: 100%; border-collapse: collapse; }
td { padding: 4px 4px; border-top: 1px solid var(--line); vertical-align: middle; }
tr.sel td { background: var(--chip); }
tr { cursor: pointer; }
.sw { display: inline-block; width: 14px; height: 14px; border-radius: 3px; vertical-align: -2px; border: 1px solid rgba(0,0,0,.25); }
.arrow { color: var(--muted); }
.lock { display: none; margin-top: 8px; color: var(--warn); font-weight: 600; }
.locked .lock { display: block; }
label.chk { display: inline-flex; gap: 6px; align-items: center; color: var(--muted); font-size: 13px; }
.mono { font-variant-numeric: tabular-nums; }
</style>
</head>
<body>
<header>
  <h1>Sorting cell &middot; scene</h1>
  <span id="seq" class="pill">connecting</span>
  <span id="meta" class="meta"></span>
  <span style="flex:1"></span>
  <button id="start" class="primary">Start sort</button>
</header>
<main id="main">
  <section class="card">
    <canvas id="cv" aria-label="Top view of the table"></canvas>
    <div class="row" style="margin-top:8px">
      <label class="chk"><input type="checkbox" id="reach" checked> Show IK reachability</label>
      <label class="chk"><input type="checkbox" id="lines" checked> Show destinations</label>
    </div>
    <p class="hint">Top view, robot base at the bottom, table in the middle. Drag an item, or select it and click a spot on the table.
      Arrow keys nudge the selected item 10 mm (Shift: 2 mm), Delete removes it. A position is only accepted if the
      own IK can plan the whole sort; red cells are out of the arm's reach.</p>
  </section>
  <aside class="side">
    <div class="card">
      <h2>Status</h2>
      <div id="msg">Waiting for scene_server...</div>
      <div class="lock">The arm is moving: editing is locked until the cycle ends.</div>
    </div>
    <div class="card" id="drivecard">
      <h2>Servo drive</h2>
      <div class="row"><span id="dstate" class="pill">-</span><span id="dpeak" class="meta"></span></div>
      <table id="fe" class="mono" style="margin-top:8px"></table>
      <div class="row" style="margin-top:8px">
        <select id="jamj"><option value="1">J1</option><option value="2" selected>J2</option><option value="3">J3</option>
          <option value="4">J4</option><option value="5">J5</option><option value="6">J6</option></select>
        <button id="jam" class="small">Jam joint</button>
        <button id="clear" class="small">Clear jam</button>
        <button id="dreset" class="small">Reset drive</button>
      </div>
      <p class="hint">Fault injection: a jammed joint cannot follow, the following error passes its limit and the
        drive switches the torque off. With the PLC the alarm is latched there and needs a PLC reset.</p>
    </div>
    <div class="card">
      <h2>Random placement</h2>
      <div class="row">
        <button data-cmd="random objects">Objects</button>
        <button data-cmd="random containers">Containers</button>
        <button data-cmd="random all">Everything</button>
        <button data-cmd="reset" title="Reload the scene file the cell was started with">Reset</button>
      </div>
    </div>
    <div class="card">
      <h2>Objects</h2>
      <table id="objs"></table>
      <div class="row" style="margin-top:8px">
        <select id="newObj"></select><button id="addObj" class="small">Add object</button>
      </div>
    </div>
    <div class="card">
      <h2>Containers</h2>
      <table id="bins"></table>
      <div class="row" style="margin-top:8px">
        <select id="newBin"></select><button id="addBin" class="small">Add container</button>
      </div>
    </div>
    <div class="card">
      <h2>Selected</h2>
      <div id="selbox" class="meta">Nothing selected.</div>
    </div>
    <div class="card">
      <h2>Scene file</h2>
      <div class="row"><input class="path" id="path" value="~/sorting_cell_scene.yaml" spellcheck="false">
        <button id="save" class="small">Save</button><button id="load" class="small">Load</button></div>
    </div>
  </aside>
</main>
<script>
"use strict";
const RGB = { red: [219,41,41], orange: [245,140,31], yellow: [230,219,41], green: [61,191,61],
              cyan: [41,199,219], blue: [41,89,219], purple: [150,51,189], gray: [199,199,199] };
const VIEW = { xTop: -1.00, xBot: 0.08, yL: -0.80, yR: 0.80 };
const $ = (id) => document.getElementById(id);
const cv = $("cv"), ctx = cv.getContext("2d");
let snap = null, S = null, version = -1, sel = null, drag = null, ghost = null;
let checkBusy = false, lastCheck = 0, k = 1, dpr = 1;

function rgb(name, a) { const c = RGB[name] || RGB.gray; return a === undefined ? `rgb(${c})` : `rgba(${c},${a})`; }
function css(v) { return getComputedStyle(document.documentElement).getPropertyValue(v).trim(); }
function el(tag, props, ...kids) {
  const e = document.createElement(tag);
  for (const [key, v] of Object.entries(props || {})) {
    if (key === "class") e.className = v; else if (key.startsWith("on")) e.addEventListener(key.slice(2), v);
    else if (key === "text") e.textContent = v; else e.setAttribute(key, v);
  }
  for (const c of kids) if (c != null) e.append(c);
  return e;
}
const locked = () => !S || S.seq === "running";

// ---------------------------------------------------------------- server
async function api(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" },
                                  body: JSON.stringify(body || {}) });
    return await r.json();
  } catch (e) { return { ok: false, message: "dashboard not reachable: " + e }; }
}
function say(text, ok) { const m = $("msg"); m.textContent = text; m.className = ok ? "ok" : "bad"; }
async function run(cmd) {
  if (locked()) { say("The arm is moving: editing is locked.", false); return { ok: false }; }
  const r = await api("/api/command", { cmd });
  say(r.message || "", r.ok);
  await poll();
  return r;
}
async function poll() {
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    const s = await r.json();
    snap = s;
    if (s.version !== version) {
      const first = !S;
      version = s.version; S = s.state;
      if (S) { fixSel(); renderPanel(); if (first) say(S.message || "Scene loaded", S.ok !== false); }
    }
    renderHeader();
    renderDrive();
    draw();
  } catch (e) { $("seq").textContent = "offline"; $("seq").className = "pill fault"; }
}

// ---------------------------------------------------------------- geometry
function resize() {
  dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = Math.round(w * (VIEW.xBot - VIEW.xTop) / (VIEW.yR - VIEW.yL));
  cv.style.height = h + "px";
  cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  k = w / (VIEW.yR - VIEW.yL);
  draw();
}
const w2c = (x, y) => [(y - VIEW.yL) * k, (x - VIEW.xTop) * k];
const c2w = (px, py) => [VIEW.xTop + py / k, VIEW.yL + px / k];
function itemPos(it) {
  if (it.kind === "object") return S.objects[it.i];
  if (it.kind === "container") return S.containers[it.i];
  return S.gray;
}
function hit(px, py) {
  const [x, y] = c2w(px, py), cube = S.sizes.cube, bin = S.sizes.container, pad = 4 / k;
  for (let i = S.objects.length - 1; i >= 0; i--) {
    const o = S.objects[i];
    if (Math.abs(o.x - x) < cube * 0.75 + pad && Math.abs(o.y - y) < cube * 0.75 + pad) return { kind: "object", i };
  }
  for (let i = 0; i < S.containers.length; i++) {
    const c = S.containers[i];
    if (Math.abs(c.x - x) < bin / 2 && Math.abs(c.y - y) < bin / 2) return { kind: "container", i };
  }
  if (Math.abs(S.gray.x - x) < bin / 2 && Math.abs(S.gray.y - y) < bin / 2) return { kind: "gray", i: 0 };
  return null;
}
function fixSel() {
  if (!sel) return;
  if ((sel.kind === "object" && sel.i >= S.objects.length) || (sel.kind === "container" && sel.i >= S.containers.length)) sel = null;
}
const same = (a, b) => a && b && a.kind === b.kind && a.i === b.i;
const f4 = (v) => (+v).toFixed(4);
function moveCmd(it, x, y, check) {
  const verb = check ? "check_" : "move_";
  if (it.kind === "gray") return `${verb}gray ${f4(x)} ${f4(y)}`;
  return `${verb}${it.kind} ${it.i} ${f4(x)} ${f4(y)}`;
}

// ---------------------------------------------------------------- drawing
function rect(x0, x1, y0, y1) { const [a, b] = w2c(x0, y0), [c, d] = w2c(x1, y1); return [Math.min(a, c), Math.min(b, d), Math.abs(c - a), Math.abs(d - b)]; }
function draw() {
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const W = cv.width / dpr, H = cv.height / dpr;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = css("--chip"); ctx.fillRect(0, 0, W, H);
  if (!S) return;
  const R = S.regions;
  ctx.fillStyle = css("--table"); ctx.fillRect(...rect(R.table.xmin, R.table.xmax, R.table.ymin, R.table.ymax));
  // reachability: red where the own IK cannot plan home -> hover -> grasp
  if ($("reach").checked && S.reach) {
    const g = S.reach;
    ctx.fillStyle = css("--reach-no");
    ctx.beginPath();  // one path, one fill: overlapping cells do not darken each other
    for (let i = 0; i < g.nx; i++) for (let j = 0; j < g.ny; j++) {
      if (String(g.rows[i])[j] === "1") continue;
      const x = g.x0 - i * g.step, y = g.y0 + j * g.step;
      const [a, b, w, h] = rect(x - g.step / 2, x + g.step / 2, y - g.step / 2, y + g.step / 2);
      ctx.rect(Math.floor(a), Math.floor(b), Math.ceil(w) + 1, Math.ceil(h) + 1);
    }
    ctx.fill();
  }
  ctx.setLineDash([6, 5]); ctx.lineWidth = 1.2; ctx.strokeStyle = "rgba(255,255,255,.55)";
  ctx.strokeRect(...rect(R.object.xmin, R.object.xmax, R.object.ymin, R.object.ymax));
  ctx.setLineDash([]);
  ctx.fillStyle = "rgba(255,255,255,.75)"; ctx.font = "11px system-ui, sans-serif";
  const [lx, ly] = w2c(R.object.xmin, R.object.ymin); ctx.fillText("camera view", lx + 4, ly + 13);
  // robot base (UR5e base is about 0.15 m across)
  const [bx, by] = w2c(0, 0);
  ctx.fillStyle = css("--muted"); ctx.beginPath(); ctx.arc(bx, by, 0.075 * k, 0, 2 * Math.PI); ctx.fill();
  ctx.fillStyle = css("--card"); ctx.textAlign = "center"; ctx.font = "600 11px system-ui, sans-serif";
  ctx.fillText("base", bx, by + 4); ctx.textAlign = "left";
  if ($("reach").checked) {
    ctx.fillStyle = css("--muted"); ctx.font = "11px system-ui, sans-serif";
    const [rx, ry] = w2c(-0.97, -0.78); ctx.fillText("red: out of reach (own IK)", rx, ry + 10);
  }
  // destinations
  if ($("lines").checked) {
    S.objects.forEach((o) => {
      const d = o.dest === "gray" ? S.gray : S.containers.find((c) => c.color === o.dest);
      if (!d) return;
      const [a, b] = w2c(o.x, o.y), [c, e] = w2c(d.x, d.y);
      ctx.strokeStyle = rgb(o.color, .9); ctx.lineWidth = 1.5; ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, e); ctx.stroke(); ctx.setLineDash([]);
    });
  }
  const bin = S.sizes.container, cube = S.sizes.cube;
  const drawBin = (c, name, label, it) => {
    ctx.fillStyle = rgb(name, .55); ctx.fillRect(...rect(c.x - bin / 2, c.x + bin / 2, c.y - bin / 2, c.y + bin / 2));
    ctx.lineWidth = same(sel, it) ? 3 : 1.5; ctx.strokeStyle = same(sel, it) ? css("--accent") : rgb(name);
    ctx.strokeRect(...rect(c.x - bin / 2, c.x + bin / 2, c.y - bin / 2, c.y + bin / 2));
    const [px, py] = w2c(c.x, c.y);
    ctx.fillStyle = "#fff"; ctx.font = "600 11px system-ui, sans-serif"; ctx.textAlign = "center";
    ctx.fillText(label, px, py + bin * k / 2 - 5); ctx.textAlign = "left";
  };
  S.containers.forEach((c, i) => drawBin(c, c.color, c.color, { kind: "container", i }));
  drawBin(S.gray, "gray", "no match", { kind: "gray", i: 0 });
  const drawCube = (o, i, alpha, outline) => {
    const [px, py] = w2c(o.x, o.y), s = cube * k;
    ctx.save(); ctx.translate(px, py);
    // base yaw about +z; on this view x points down and y right, so a positive yaw turns clockwise
    ctx.rotate(-(o.yaw || 0));
    ctx.fillStyle = rgb(o.color, alpha); ctx.fillRect(-s / 2, -s / 2, s, s);
    ctx.lineWidth = outline ? 3 : 1.2; ctx.strokeStyle = outline || "rgba(0,0,0,.6)"; ctx.strokeRect(-s / 2, -s / 2, s, s);
    ctx.restore();
    if (i !== null) { ctx.fillStyle = "#fff"; ctx.font = "700 10px system-ui"; ctx.textAlign = "center"; ctx.fillText(String(i), px, py + 3.5); ctx.textAlign = "left"; }
  };
  S.objects.forEach((o, i) => drawCube(o, i, 1, same(sel, { kind: "object", i }) ? css("--accent") : null));
  if (ghost) {
    const col = ghost.status === "ok" ? css("--ok") : ghost.status === "bad" ? css("--bad") : css("--muted");
    if (ghost.it.kind === "object") {
      drawCube({ x: ghost.x, y: ghost.y, yaw: S.objects[ghost.it.i].yaw, color: S.objects[ghost.it.i].color }, null, .45, col);
    } else {
      ctx.setLineDash([5, 4]); ctx.lineWidth = 2.5; ctx.strokeStyle = col;
      ctx.strokeRect(...rect(ghost.x - bin / 2, ghost.x + bin / 2, ghost.y - bin / 2, ghost.y + bin / 2)); ctx.setLineDash([]);
    }
  }
}

// ---------------------------------------------------------------- panel
function swatch(name) { return el("span", { class: "sw", style: `background:${rgb(name)}` }); }
function colorSelect(value, onchange, extra) {
  const s = el("select", { onchange: (e) => onchange(e.target.value), onclick: (e) => e.stopPropagation() });
  for (const p of S.palette) { const o = el("option", { value: p, text: p }); if (p === value) o.selected = true; s.append(o); }
  if (extra) s.title = extra;
  s.disabled = locked();
  return s;
}
function renderHeader() {
  const seq = (S && S.seq) || "connecting";
  $("seq").textContent = seq; $("seq").className = "pill " + seq;
  const parts = [];
  if (S) {
    parts.push(`scene "${S.name}" rev ${S.revision}`);
    if (S.plan) parts.push(S.plan.ok ? `planned cycle ${S.plan.seconds.toFixed(1)} s` : "not plannable");
  }
  if (snap && snap.phase && snap.phase !== "idle") parts.push(`phase: ${snap.phase}`);
  if (snap && snap.use_plc) parts.push("PLC controls the start");
  $("meta").textContent = parts.join("  ·  ");
  $("start").disabled = !S || (snap && snap.use_plc) || S.seq === "running" || S.seq === "fault";
  $("main").classList.toggle("locked", locked());
  for (const b of document.querySelectorAll("[data-cmd], #addObj, #addBin, #load")) b.disabled = locked();
}
function renderDrive() {
  const card = $("drivecard");
  if (!snap || !snap.has_drive) { card.style.display = "none"; return; }
  card.style.display = "";
  const d = snap.drive || {}, st = d.state || "no data";
  const word = st.split(" ")[0].toLowerCase();
  $("dstate").textContent = st;
  $("dstate").className = "pill " + (word === "running" ? "ready" : word);
  $("dpeak").textContent = d.fe_peak_mrad ? `peak following error ${d.fe_peak_mrad.toFixed(2)} mrad` : "";
  const t = $("fe"); t.replaceChildren();
  (d.fe_mrad || []).forEach((v, j) => {
    const lim = j < 3 ? 30 : 50, w = Math.min(100, 100 * v / lim);
    t.append(el("tr", {}, el("td", { text: "J" + (j + 1) }),
      el("td", { style: "width:60%" }, el("span", { class: "bar" + (v > 0.5 * lim ? " hot" : ""), style: `width:${Math.max(2, w)}%` })),
      el("td", { style: "text-align:right", text: v.toFixed(2) + " mrad" })));
  });
  $("dreset").disabled = !!snap.use_plc;
  $("dreset").title = snap.use_plc ? "With the PLC, reset from the PLC" : "";
}
async function driveAction(action, joint) {
  const r = await api("/api/drive", { action, joint });
  say(r.message || "", r.ok);
  poll();
}

function renderPanel() {
  const objs = $("objs"); objs.replaceChildren();
  S.objects.forEach((o, i) => {
    const it = { kind: "object", i };
    const tr = el("tr", { class: same(sel, it) ? "sel" : "", onclick: () => select(it) },
      el("td", { class: "mono", text: "#" + i }),
      el("td", {}, colorSelect(o.color, (v) => run(`color_object ${i} ${v}`))),
      el("td", { class: "arrow" }, "→ ", swatch(o.dest), " " + (o.dest === "gray" ? "gray (no match)" : o.dest)),
      el("td", { style: "text-align:right" }, el("button", { class: "small", title: "Remove", text: "✕",
        onclick: (e) => { e.stopPropagation(); run(`remove_object ${i}`); } })));
    objs.append(tr);
  });
  const bins = $("bins"); bins.replaceChildren();
  S.containers.forEach((c, i) => {
    const it = { kind: "container", i };
    const n = S.objects.filter((o) => o.dest === c.color).length;
    bins.append(el("tr", { class: same(sel, it) ? "sel" : "", onclick: () => select(it) },
      el("td", { class: "mono", text: "#" + i }),
      el("td", {}, colorSelect(c.color, (v) => run(`color_container ${i} ${v}`))),
      el("td", { class: "arrow", text: `${n} object${n === 1 ? "" : "s"}` }),
      el("td", { style: "text-align:right" }, el("button", { class: "small", title: "Remove", text: "✕",
        onclick: (e) => { e.stopPropagation(); run(`remove_container ${i}`); } }))));
  });
  const ng = S.objects.filter((o) => o.dest === "gray").length;
  bins.append(el("tr", { class: same(sel, { kind: "gray", i: 0 }) ? "sel" : "", onclick: () => select({ kind: "gray", i: 0 }) },
    el("td", {}), el("td", {}, swatch("gray"), " gray"), el("td", { class: "arrow", text: `${ng} unmatched` }), el("td", {})));
  for (const id of ["newObj", "newBin"]) {
    const s = $(id), keep = s.value; s.replaceChildren();
    for (const p of S.palette) s.append(el("option", { value: p, text: p }));
    const used = S.containers.map((c) => c.color);
    s.value = keep && S.palette.includes(keep) ? keep : (id === "newBin" ? (S.palette.find((p) => !used.includes(p)) || S.palette[0]) : S.palette[0]);
  }
  renderSel();
}
function renderSel() {
  const box = $("selbox"); box.replaceChildren();
  if (!sel || !S) { box.textContent = "Nothing selected. Click an item on the table or in a list."; box.className = "meta"; return; }
  box.className = "";
  const p = itemPos(sel);
  const xi = el("input", { type: "number", step: "0.01", value: (+p.x).toFixed(3) });
  const yi = el("input", { type: "number", step: "0.01", value: (+p.y).toFixed(3) });
  const title = sel.kind === "object" ? `Object #${sel.i} (${p.color})` : sel.kind === "container" ? `Container #${sel.i} (${p.color})` : "Gray container";
  box.append(el("div", { style: "font-weight:600;margin-bottom:6px", text: title }));
  box.append(el("div", { class: "row" }, "x ", xi, " y ", yi,
    el("button", { class: "small", text: "Move", onclick: () => run(moveCmd(sel, xi.value, yi.value, false)) })));
  if (sel.kind === "object") {
    const deg = el("input", { type: "number", step: "5", value: Math.round((p.yaw || 0) * 180 / Math.PI) });
    box.append(el("div", { class: "row", style: "margin-top:6px" }, "yaw ", deg, " deg",
      el("button", { class: "small", text: "Rotate", onclick: () => run(`yaw_object ${sel.i} ${(deg.value * Math.PI / 180).toFixed(4)}`) })));
  }
  box.append(el("div", { class: "meta", style: "margin-top:6px",
    text: "x grows toward the robot base, y to the right. Metres, robot base frame." }));
}
function select(it) { sel = it; ghost = null; if (S) renderPanel(); draw(); }

// ---------------------------------------------------------------- pointer
function pt(e) { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
cv.addEventListener("pointerdown", (e) => {
  if (!S) return;
  const [px, py] = pt(e), it = hit(px, py);
  drag = { it, sx: px, sy: py, moved: false };
  if (it) { select(it); cv.setPointerCapture(e.pointerId); }
});
cv.addEventListener("pointermove", async (e) => {
  if (!drag || !drag.it || locked()) return;
  const [px, py] = pt(e);
  if (!drag.moved && Math.hypot(px - drag.sx, py - drag.sy) < 4) return;
  drag.moved = true;
  const [x, y] = c2w(px, py);
  ghost = { it: drag.it, x, y, status: ghost && ghost.status || "checking" };
  draw();
  const now = performance.now();
  if (checkBusy || now - lastCheck < 120) return;
  checkBusy = true; lastCheck = now;
  const g = ghost;
  const r = await api("/api/command", { cmd: moveCmd(g.it, g.x, g.y, true) });
  checkBusy = false;
  if (ghost && drag && drag.moved) { ghost.status = r.ok ? "ok" : "bad"; if (!r.ok) say(r.message, false); else say("Position is valid", true); draw(); }
});
cv.addEventListener("pointerup", async (e) => {
  if (!drag) return;
  const d = drag; drag = null;
  const [px, py] = pt(e);
  const [x, y] = c2w(px, py);
  if (d.it && d.moved) { ghost = null; await run(moveCmd(d.it, x, y, false)); return; }
  if (!d.it && sel && Math.hypot(px - d.sx, py - d.sy) < 4) { await run(moveCmd(sel, x, y, false)); return; }
  if (!d.it) { sel = null; renderPanel(); draw(); }
});
cv.addEventListener("pointercancel", () => { drag = null; ghost = null; draw(); });
document.addEventListener("keydown", (e) => {
  if (!sel || !S || e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
  const step = e.shiftKey ? 0.002 : 0.01, p = itemPos(sel);
  const d = { ArrowUp: [-step, 0], ArrowDown: [step, 0], ArrowLeft: [0, -step], ArrowRight: [0, step] }[e.key];
  if (d) { e.preventDefault(); run(moveCmd(sel, p.x + d[0], p.y + d[1], false)); }
  else if ((e.key === "Delete" || e.key === "Backspace") && sel.kind !== "gray") { e.preventDefault(); run(`remove_${sel.kind} ${sel.i}`); }
  else if (e.key === "Escape") select(null);
});

// ---------------------------------------------------------------- buttons
for (const b of document.querySelectorAll("[data-cmd]")) b.addEventListener("click", () => run(b.dataset.cmd));
$("addObj").addEventListener("click", () => run(`add_object ${$("newObj").value}`));
$("addBin").addEventListener("click", () => run(`add_container ${$("newBin").value}`));
$("save").addEventListener("click", () => run(`save ${$("path").value}`));
$("load").addEventListener("click", () => run(`load ${$("path").value}`));
$("start").addEventListener("click", async () => { const r = await api("/api/start"); say(r.message, r.ok); poll(); });
$("jam").addEventListener("click", () => driveAction("jam", +$("jamj").value));
$("clear").addEventListener("click", () => driveAction("clear"));
$("dreset").addEventListener("click", () => driveAction("reset"));
$("reach").addEventListener("change", draw);
$("lines").addEventListener("change", draw);
window.addEventListener("resize", resize);
resize();
poll();
setInterval(poll, 400);
</script>
</body>
</html>
"""
