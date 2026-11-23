// scan3d 메시 뷰어: WebGL2, 외부 라이브러리 없음. 좌표는 표식 판 기준(m, z 위).
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const canvas = $("view");
  const gl = canvas.getContext("webgl2", { antialias: true });
  const POLL_MS = 3000;
  const FOV = (40 * Math.PI) / 180;
  const BOARD = 0.594; // 표식 판 한 변(m)

  // ---------- 행렬 ----------
  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
  }
  function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
  function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
  function norm(a) { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }
  function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
  function lookAt(eye, target, up) {
    const z = norm(sub(eye, target)), x = norm(cross(up, z)), y = cross(z, x);
    return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
      -dot(x, eye), -dot(y, eye), -dot(z, eye), 1]);
  }
  function mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
    return o;
  }

  // ---------- 셰이더 ----------
  const SURF_VS = `#version 300 es
  in vec3 aPos; in vec3 aNor; in vec3 aCol;
  uniform mat4 uMVP; uniform mat4 uView; uniform float uPointSize;
  out vec3 vNor; out vec3 vCol;
  void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    gl_PointSize = uPointSize;
    vNor = mat3(uView) * aNor;
    vCol = aCol;
  }`;
  const SURF_FS = `#version 300 es
  precision highp float;
  in vec3 vNor; in vec3 vCol;
  uniform bool uLit; uniform bool uUseColor;
  out vec4 frag;
  void main() {
    vec3 base = uUseColor ? vCol : vec3(0.78);
    float k = 1.0;
    if (uLit) {
      vec3 n = normalize(vNor);
      float d = abs(n.z);                      // 카메라 쪽 빛(양면)
      k = uUseColor ? 0.55 + 0.45 * d : 0.25 + 0.75 * d;
    }
    frag = vec4(base * k, 1.0);
  }`;
  const LINE_VS = `#version 300 es
  in vec3 aPos; uniform mat4 uMVP;
  void main() { gl_Position = uMVP * vec4(aPos, 1.0); }`;
  const LINE_FS = `#version 300 es
  precision highp float; uniform vec4 uColor; out vec4 frag;
  void main() { frag = uColor; }`;

  function program(vs, fs) {
    const p = gl.createProgram();
    for (const [type, src] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      gl.attachShader(p, s);
    }
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    return p;
  }

  // ---------- 상태 ----------
  const state = {
    entries: [], current: null, mtime: null, mode: "mesh", model: null,
    cam: { target: [0, 0, 0.08], dist: 1.2, yaw: -Math.PI / 3, pitch: 0.45 }, home: null,
  };
  let surf, line, grid;

  function buffer(target, data) {
    const b = gl.createBuffer();
    gl.bindBuffer(target, b);
    gl.bufferData(target, data, gl.STATIC_DRAW);
    return b;
  }

  function makeGrid() {
    // 판 테두리와 5 cm 격자(바닥 z = 0)
    const pts = [], h = BOARD / 2, step = 0.05;
    for (let v = -0.25; v <= 0.2501; v += step) pts.push(-h, v, 0, h, v, 0, v, -h, 0, v, h, 0);
    const edge = [];
    for (const [a, b] of [[[-h, -h], [h, -h]], [[h, -h], [h, h]], [[h, h], [-h, h]], [[-h, h], [-h, -h]]])
      edge.push(a[0], a[1], 0, b[0], b[1], 0);
    const axes = [0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1];
    const vao = gl.createVertexArray();
    gl.bindVertexArray(vao);
    buffer(gl.ARRAY_BUFFER, new Float32Array([...pts, ...edge, ...axes]));
    const loc = gl.getAttribLocation(line.p, "aPos");
    gl.enableVertexAttribArray(loc);
    gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0);
    gl.bindVertexArray(null);
    return { vao, grid: pts.length / 3, edge: edge.length / 3 };
  }

  function upload(ply) {
    if (state.model) {
      gl.deleteVertexArray(state.model.vao);
      for (const b of state.model.buffers) gl.deleteBuffer(b);
    }
    const vao = gl.createVertexArray(), buffers = [];
    gl.bindVertexArray(vao);
    const attr = (name, data, size, type, normalized) => {
      const loc = gl.getAttribLocation(surf.p, name);
      if (loc < 0) return;
      if (!data) { gl.disableVertexAttribArray(loc); return; }
      buffers.push(buffer(gl.ARRAY_BUFFER, data));
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, size, type, normalized, 0, 0);
    };
    attr("aPos", ply.positions, 3, gl.FLOAT, false);
    attr("aNor", ply.normals, 3, gl.FLOAT, false);
    attr("aCol", ply.colors, 3, gl.UNSIGNED_BYTE, true);
    if (ply.indices) buffers.push(buffer(gl.ELEMENT_ARRAY_BUFFER, ply.indices));
    gl.bindVertexArray(null);

    const p = ply.positions, lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < p.length; i += 3) for (let k = 0; k < 3; k++) {
      lo[k] = Math.min(lo[k], p[i + k]); hi[k] = Math.max(hi[k], p[i + k]);
    }
    state.model = {
      vao, buffers, count: ply.indices ? ply.indices.length : ply.vertexCount, triangles: !!ply.indices,
      hasColor: !!ply.colors, hasNormals: !!ply.normals, lo, hi,
    };
  }

  function fit() {
    const m = state.model;
    if (!m) return;
    const c = [0, 1, 2].map((k) => (m.lo[k] + m.hi[k]) / 2);
    const r = Math.max(0.05, Math.hypot(m.hi[0] - m.lo[0], m.hi[1] - m.lo[1], m.hi[2] - m.lo[2]) / 2);
    // 세로로 긴 화면(휴대폰)에서는 가로 화각이 더 좁으므로 그쪽에 맞춘다
    const aspect = canvas.clientWidth / Math.max(1, canvas.clientHeight);
    const fov = aspect < 1 ? 2 * Math.atan(Math.tan(FOV / 2) * aspect) : FOV;
    if (canvas.clientWidth <= 520) c[2] -= r * 0.6;   // 좁은 화면은 패널이 아래에 있으므로 물체를 위로 올린다
    state.cam = { target: c, dist: (r / Math.sin(fov / 2)) * 1.35, yaw: -Math.PI / 3, pitch: 0.45 };
  }

  // ---------- 그리기 ----------
  let dirty = true;
  function requestDraw() { dirty = true; }

  function draw() {
    requestAnimationFrame(draw);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.round(canvas.clientWidth * dpr), h = Math.round(canvas.clientHeight * dpr);
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; dirty = true; }
    if (!dirty) return;
    dirty = false;

    const cs = getComputedStyle(document.documentElement);
    const bg = cs.getPropertyValue("--canvas").trim().match(/\d+/g).map((v) => +v / 255);
    gl.viewport(0, 0, w, h);
    gl.clearColor(bg[0], bg[1], bg[2], 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enable(gl.DEPTH_TEST);

    const c = state.cam, cp = Math.cos(c.pitch);
    const eye = [c.target[0] + c.dist * cp * Math.cos(c.yaw), c.target[1] + c.dist * cp * Math.sin(c.yaw),
      c.target[2] + c.dist * Math.sin(c.pitch)];
    const view = lookAt(eye, c.target, [0, 0, 1]);
    const proj = perspective(FOV, w / Math.max(1, h), c.dist * 0.01, c.dist * 50);
    const mvp = mul(proj, view);

    if ($("grid").checked) {
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
      gl.depthMask(false);
      gl.useProgram(line.p);
      gl.uniformMatrix4fv(line.uMVP, false, mvp);
      gl.bindVertexArray(grid.vao);
      gl.uniform4fv(line.uColor, cs.getPropertyValue("--grid").trim().match(/[\d.]+/g).map((v, i) => (i < 3 ? +v / 255 : +v)));
      gl.drawArrays(gl.LINES, 0, grid.grid);
      gl.uniform4fv(line.uColor, cs.getPropertyValue("--edge").trim().match(/[\d.]+/g).map((v, i) => (i < 3 ? +v / 255 : +v)));
      gl.drawArrays(gl.LINES, grid.grid, grid.edge);
      const axis = [[0.86, 0.25, 0.25, 1], [0.25, 0.7, 0.3, 1], [0.3, 0.45, 0.95, 1]];
      for (let k = 0; k < 3; k++) {
        gl.uniform4fv(line.uColor, axis[k]);
        gl.drawArrays(gl.LINES, grid.grid + grid.edge + 2 * k, 2);
      }
      gl.depthMask(true);
      gl.disable(gl.BLEND);
    }

    const m = state.model;
    if (m) {
      gl.useProgram(surf.p);
      gl.uniformMatrix4fv(surf.uMVP, false, mvp);
      gl.uniformMatrix4fv(surf.uView, false, view);
      gl.uniform1i(surf.uLit, m.triangles && m.hasNormals ? 1 : 0);
      gl.uniform1i(surf.uUseColor, m.hasColor && $("color").checked ? 1 : 0);
      gl.uniform1f(surf.uPointSize, Math.max(1.5, 2.5 * dpr));
      gl.bindVertexArray(m.vao);
      if (!m.hasColor) gl.vertexAttrib3f(gl.getAttribLocation(surf.p, "aCol"), 0.78, 0.78, 0.78);
      if (!m.hasNormals) gl.vertexAttrib3f(gl.getAttribLocation(surf.p, "aNor"), 0, 0, 1);
      if (m.triangles) gl.drawElements(gl.TRIANGLES, m.count, gl.UNSIGNED_INT, 0);
      else gl.drawArrays(gl.POINTS, 0, m.count);
    }
    gl.bindVertexArray(null);
  }

  // ---------- 조작 ----------
  function rotate(dx, dy) {
    state.cam.yaw -= dx * 0.008;
    state.cam.pitch = Math.max(-1.55, Math.min(1.55, state.cam.pitch + dy * 0.008));
    requestDraw();
  }
  function pan(dx, dy) {
    const c = state.cam, s = (c.dist * 2 * Math.tan(FOV / 2)) / canvas.clientHeight;
    const right = [-Math.sin(c.yaw), Math.cos(c.yaw), 0];
    const up = [-Math.sin(c.pitch) * Math.cos(c.yaw), -Math.sin(c.pitch) * Math.sin(c.yaw), Math.cos(c.pitch)];
    for (let k = 0; k < 3; k++) c.target[k] += (-dx * right[k] + dy * up[k]) * s;
    requestDraw();
  }
  function zoom(f) { state.cam.dist = Math.max(0.05, Math.min(20, state.cam.dist * f)); requestDraw(); }

  const pointers = new Map();
  let pinch = null;
  canvas.addEventListener("pointerdown", (e) => {
    canvas.setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, button: e.button, shift: e.shiftKey });
  });
  canvas.addEventListener("pointermove", (e) => {
    const p = pointers.get(e.pointerId);
    if (!p) return;
    const dx = e.clientX - p.x, dy = e.clientY - p.y;
    p.x = e.clientX; p.y = e.clientY;
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      const d = Math.hypot(a.x - b.x, a.y - b.y), mid = [(a.x + b.x) / 2, (a.y + b.y) / 2];
      if (pinch) { zoom(pinch.d / Math.max(1, d)); pan(mid[0] - pinch.mid[0], mid[1] - pinch.mid[1]); }
      pinch = { d, mid };
      return;
    }
    if (p.button === 2 || p.button === 1 || p.shift) pan(dx, dy);
    else rotate(dx, dy);
  });
  const release = (e) => { pointers.delete(e.pointerId); if (pointers.size < 2) pinch = null; };
  canvas.addEventListener("pointerup", release);
  canvas.addEventListener("pointercancel", release);
  canvas.addEventListener("contextmenu", (e) => e.preventDefault());
  canvas.addEventListener("wheel", (e) => { e.preventDefault(); zoom(Math.exp(e.deltaY * 0.0015)); }, { passive: false });
  canvas.addEventListener("dblclick", () => { fit(); requestDraw(); });

  // ---------- 목록과 불러오기 ----------
  function fmtDims(d) { return d ? `${d[0].toFixed(0)} × ${d[1].toFixed(0)} × ${d[2].toFixed(0)}` : "–"; }
  function fmtTime(t) { return new Date(t * 1000).toLocaleString(); }

  function setStatus(text, isError) {
    const s = $("status");
    s.textContent = text || "";
    s.classList.toggle("error", !!isError);
  }

  function showInfo(entry) {
    $("dims").textContent = fmtDims(entry && entry.dims_mm);
    $("elapsed").textContent = entry && entry.elapsed_s != null ? `${entry.elapsed_s.toFixed(0)} s` : "–";
    $("mtime").textContent = entry ? fmtTime(entry.mtime) : "–";
    const m = state.model;
    $("counts").textContent = m ? (m.triangles ? `삼각형 ${(m.count / 3).toLocaleString()}` : `점 ${m.count.toLocaleString()}`) : "–";
    const rep = $("report");
    if (entry && entry.files.includes("report.txt")) {
      rep.href = `/files?id=${encodeURIComponent(entry.id)}&name=report.txt`;
      rep.hidden = false;
    } else rep.hidden = true;
    const both = entry && entry.files.includes("mesh.ply") && entry.files.includes("cloud.ply");
    $("modeRow").hidden = !both;
  }

  async function load(entry, keepCamera) {
    const name = state.mode === "cloud" && entry.files.includes("cloud.ply") ? "cloud.ply"
      : entry.files.includes("mesh.ply") ? "mesh.ply" : "cloud.ply";
    setStatus("불러오는 중…");
    try {
      const res = await fetch(`/files?id=${encodeURIComponent(entry.id)}&name=${name}`, { cache: "no-store" });
      if (!res.ok) throw new Error(`${res.status} ${await res.text()}`);
      const ply = parsePLY(await res.arrayBuffer());
      upload(ply);
      const sameScan = state.current === entry.id;
      state.current = entry.id;
      state.mtime = entry.mtime;
      if (!keepCamera || !sameScan) fit();
      showInfo(entry);
      setStatus("");
      requestDraw();
    } catch (err) {
      setStatus(`불러오지 못함: ${err.message}`, true);
    }
  }

  function renderList() {
    const sel = $("scan");
    const keep = state.current;
    sel.replaceChildren(...state.entries.map((e) => {
      const o = document.createElement("option");
      o.value = e.id;
      o.textContent = e.label;
      return o;
    }));
    if (keep) sel.value = keep;
    $("empty").hidden = state.entries.length > 0;
  }

  async function poll() {
    try {
      const res = await fetch("/api/scans", { cache: "no-store" });
      const entries = await res.json();
      const changed = JSON.stringify(entries.map((e) => [e.id, e.mtime])) !== JSON.stringify(state.entries.map((e) => [e.id, e.mtime]));
      state.entries = entries;
      if (changed) renderList();
      if (!entries.length) return;
      const latest = entries[0];
      const cur = entries.find((e) => e.id === state.current);
      if (!state.current || ($("follow").checked && latest.id !== state.current)) {
        await load(latest, false);
        $("scan").value = latest.id;
      } else if (cur && cur.mtime !== state.mtime) {
        await load(cur, true);                       // 같은 결과를 다시 만들었으면 시점은 그대로
      } else if (cur) showInfo(cur);
    } catch (err) {
      setStatus("서버에 연결되지 않음", true);
    }
  }

  $("scan").addEventListener("change", () => {
    const e = state.entries.find((x) => x.id === $("scan").value);
    if (!e) return;
    if (e.id !== state.entries[0].id) $("follow").checked = false;
    load(e, false);
  });
  for (const id of ["color", "grid"]) $(id).addEventListener("change", requestDraw);
  $("follow").addEventListener("change", poll);
  $("home").addEventListener("click", () => { fit(); requestDraw(); });
  for (const r of document.querySelectorAll("input[name=mode]")) r.addEventListener("change", () => {
    state.mode = r.value;
    const e = state.entries.find((x) => x.id === state.current);
    if (e) load(e, true);
  });
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", requestDraw);

  // ---------- 시작 ----------
  if (!gl) {
    setStatus("이 브라우저는 WebGL2를 지원하지 않습니다", true);
    return;
  }
  const sp = program(SURF_VS, SURF_FS), lp = program(LINE_VS, LINE_FS);
  surf = { p: sp, uMVP: gl.getUniformLocation(sp, "uMVP"), uView: gl.getUniformLocation(sp, "uView"),
    uLit: gl.getUniformLocation(sp, "uLit"), uUseColor: gl.getUniformLocation(sp, "uUseColor"),
    uPointSize: gl.getUniformLocation(sp, "uPointSize") };
  line = { p: lp, uMVP: gl.getUniformLocation(lp, "uMVP"), uColor: gl.getUniformLocation(lp, "uColor") };
  grid = makeGrid();
  requestAnimationFrame(draw);
  poll();
  setInterval(poll, POLL_MS);
})();
