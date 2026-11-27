// 메시 보기: WebGL2, 외부 라이브러리 없음. 좌표는 표식 판 기준(m, z 위) 그대로(고른 점을 그대로 서버에 보낸다).
// new MeshView(canvas) → load(arrayBuffer), fit(), pick(clientX, clientY), setMarkers([...]), setOptions({color, grid})
(function (root) {
  "use strict";

  const FOV = (40 * Math.PI) / 180;
  const BOARD = 0.594; // 표식 판 한 변(m)

  // ---------- 행렬 ----------
  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
  }
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  function norm(a) { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }
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
  void main() { gl_Position = uMVP * vec4(aPos, 1.0); gl_PointSize = uPointSize; vNor = mat3(uView) * aNor; vCol = aCol; }`;
  const SURF_FS = `#version 300 es
  precision highp float;
  in vec3 vNor; in vec3 vCol; uniform bool uLit; uniform bool uUseColor; out vec4 frag;
  void main() {
    vec3 base = uUseColor ? vCol : vec3(0.78);
    float k = 1.0;
    if (uLit) { float d = abs(normalize(vNor).z); k = uUseColor ? 0.55 + 0.45 * d : 0.25 + 0.75 * d; }
    frag = vec4(base * k, 1.0);
  }`;
  const LINE_VS = `#version 300 es
  in vec3 aPos; uniform mat4 uMVP; void main() { gl_Position = uMVP * vec4(aPos, 1.0); }`;
  const LINE_FS = `#version 300 es
  precision highp float; uniform vec4 uColor; out vec4 frag; void main() { frag = uColor; }`;
  // 마커: 깊이와 상관없이 위에 그리는 둥근 점(테두리 있음)
  const MARK_VS = `#version 300 es
  in vec3 aPos; in vec3 aCol; in float aSize; uniform mat4 uMVP; out vec3 vCol;
  void main() { gl_Position = uMVP * vec4(aPos, 1.0); gl_PointSize = aSize; vCol = aCol; }`;
  const MARK_FS = `#version 300 es
  precision highp float; in vec3 vCol; out vec4 frag;
  void main() {
    float r = length(gl_PointCoord - 0.5) * 2.0;
    if (r > 1.0) discard;
    frag = r > 0.72 ? vec4(0.05, 0.05, 0.05, 1.0) : vec4(vCol, 1.0);
  }`;

  class MeshView {
    constructor(canvas) {
      this.canvas = canvas;
      this.gl = canvas.getContext("webgl2", { antialias: true });
      this.ok = !!this.gl;
      this.cam = { target: [0, 0, 0.08], dist: 1.2, yaw: -Math.PI / 3, pitch: 0.45 };
      this.opts = { color: true, grid: true };
      this.model = null;
      this.markers = [];
      this.dirty = true;
      this.onPick = null;           // (hit, event) → void
      this.pickEnabled = false;
      if (!this.ok) return;
      const gl = this.gl;
      this.surf = this._program(SURF_VS, SURF_FS, ["uMVP", "uView", "uLit", "uUseColor", "uPointSize"]);
      this.line = this._program(LINE_VS, LINE_FS, ["uMVP", "uColor"]);
      this.mark = this._program(MARK_VS, MARK_FS, ["uMVP"]);
      this.grid = this._makeGrid();
      this.markVao = gl.createVertexArray();
      this.markBuf = gl.createBuffer();
      this._bindInput();
      const loop = () => { requestAnimationFrame(loop); this._draw(); };
      requestAnimationFrame(loop);
      window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => this.redraw());
    }

    redraw() { this.dirty = true; }

    _program(vs, fs, uniforms) {
      const gl = this.gl, p = gl.createProgram();
      for (const [type, src] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
        const s = gl.createShader(type);
        gl.shaderSource(s, src);
        gl.compileShader(s);
        if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
        gl.attachShader(p, s);
      }
      gl.linkProgram(p);
      if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
      const u = { p };
      for (const n of uniforms) u[n] = gl.getUniformLocation(p, n);
      return u;
    }

    _buffer(target, data) {
      const gl = this.gl, b = gl.createBuffer();
      gl.bindBuffer(target, b);
      gl.bufferData(target, data, gl.STATIC_DRAW);
      return b;
    }

    _makeGrid() {
      const gl = this.gl, pts = [], h = BOARD / 2;
      for (let v = -0.25; v <= 0.2501; v += 0.05) pts.push(-h, v, 0, h, v, 0, v, -h, 0, v, h, 0);
      const edge = [];
      for (const [a, b] of [[[-h, -h], [h, -h]], [[h, -h], [h, h]], [[h, h], [-h, h]], [[-h, h], [-h, -h]]])
        edge.push(a[0], a[1], 0, b[0], b[1], 0);
      const axes = [0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1];
      const vao = gl.createVertexArray();
      gl.bindVertexArray(vao);
      this._buffer(gl.ARRAY_BUFFER, new Float32Array([...pts, ...edge, ...axes]));
      const loc = gl.getAttribLocation(this.line.p, "aPos");
      gl.enableVertexAttribArray(loc);
      gl.vertexAttribPointer(loc, 3, gl.FLOAT, false, 0, 0);
      gl.bindVertexArray(null);
      return { vao, grid: pts.length / 3, edge: edge.length / 3 };
    }

    clear() {
      const gl = this.gl;
      if (this.model) {
        gl.deleteVertexArray(this.model.vao);
        for (const b of this.model.buffers) gl.deleteBuffer(b);
      }
      this.model = null;
      this.redraw();
    }

    load(buffer) {
      const ply = root.parsePLY(buffer);
      if (!this.ok) return { triangles: ply.triangleCount, vertices: ply.vertexCount, lo: [0, 0, 0], hi: [0, 0, 0] };
      this.clear();
      const gl = this.gl, vao = gl.createVertexArray(), buffers = [];
      gl.bindVertexArray(vao);
      const attr = (name, data, size, type, normalized) => {
        const loc = gl.getAttribLocation(this.surf.p, name);
        if (loc < 0) return;
        if (!data) { gl.disableVertexAttribArray(loc); return null; }
        const b = this._buffer(gl.ARRAY_BUFFER, data);
        buffers.push(b);
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, size, type, normalized, 0, 0);
        return b;
      };
      attr("aPos", ply.positions, 3, gl.FLOAT, false);
      attr("aNor", ply.normals, 3, gl.FLOAT, false);
      const colorBuf = attr("aCol", ply.colors, 3, gl.UNSIGNED_BYTE, true);
      const overlayBuf = gl.createBuffer();
      buffers.push(overlayBuf);
      if (ply.indices) buffers.push(this._buffer(gl.ELEMENT_ARRAY_BUFFER, ply.indices));
      gl.bindVertexArray(null);
      const p = ply.positions, lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
      for (let i = 0; i < p.length; i += 3) for (let k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], p[i + k]); hi[k] = Math.max(hi[k], p[i + k]);
      }
      this.model = { vao, buffers, count: ply.indices ? ply.indices.length : ply.vertexCount,
        triangles: !!ply.indices, hasColor: !!ply.colors, hasNormals: !!ply.normals, lo, hi,
        positions: ply.positions, indices: ply.indices, colors: ply.colors, colorBuf,
        overlayBuf, overlay: false, vertexCount: ply.vertexCount };
      this.redraw();
      return { triangles: ply.triangleCount, vertices: ply.vertexCount, lo, hi };
    }

    fit() {
      const m = this.model;
      if (!m) return;
      // 숨은 탭(크기 0)에서는 화각을 못 구한다. 보일 때 _draw가 다시 부른다
      if (!this.canvas.clientWidth || !this.canvas.clientHeight) { this.needFit = true; return; }
      this.needFit = false;
      const c = [0, 1, 2].map((k) => (m.lo[k] + m.hi[k]) / 2);
      const r = Math.max(0.05, Math.hypot(m.hi[0] - m.lo[0], m.hi[1] - m.lo[1], m.hi[2] - m.lo[2]) / 2);
      const aspect = this.canvas.clientWidth / Math.max(1, this.canvas.clientHeight);
      const fov = aspect < 1 ? 2 * Math.atan(Math.tan(FOV / 2) * aspect) : FOV;
      this.cam = { target: c, dist: (r / Math.sin(fov / 2)) * 1.35, yaw: -Math.PI / 3, pitch: 0.45 };
      this.redraw();
    }

    setOptions(o) { Object.assign(this.opts, o); this.redraw(); }

    // 꼭짓점 색을 바꿔 그린다(덮인 정도, 고른 영역). colors: Uint8Array(3n) 또는 null(원래 색)
    setOverlay(colors) {
      const m = this.model;
      if (!this.ok || !m) return;
      const gl = this.gl, loc = gl.getAttribLocation(this.surf.p, "aCol");
      gl.bindVertexArray(m.vao);
      if (colors) {
        gl.bindBuffer(gl.ARRAY_BUFFER, m.overlayBuf);
        gl.bufferData(gl.ARRAY_BUFFER, colors, gl.DYNAMIC_DRAW);
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, 3, gl.UNSIGNED_BYTE, true, 0, 0);
      } else if (m.colorBuf) {
        gl.bindBuffer(gl.ARRAY_BUFFER, m.colorBuf);
        gl.vertexAttribPointer(loc, 3, gl.UNSIGNED_BYTE, true, 0, 0);
      } else {
        gl.disableVertexAttribArray(loc);
      }
      gl.bindVertexArray(null);
      m.overlay = !!colors;
      this.redraw();
    }

    // 화면 사각형(클라이언트 좌표) 안의 카메라 쪽을 향하고 보이는 삼각형.
    // 삼각형 중심을 깊이 칸(6 px)에 찍어 칸마다 가장 앞(+1 cm) 것만(가린 면 제외).
    // → {tris: Uint32Array, vertices: Set, points: [[x,y,z]…], normals: […]}(points·normals는 최대 maxPts개로 솎음)
    selectRect(x0, y0, x1, y1, maxPts = 1500) {
      const m = this.model;
      if (!m || !m.triangles || !this._mvp) return null;
      const rect = this.canvas.getBoundingClientRect();
      const ax = Math.min(x0, x1) - rect.left, bx = Math.max(x0, x1) - rect.left;
      const ay = Math.min(y0, y1) - rect.top, by = Math.max(y0, y1) - rect.top;
      const P = m.positions, I = m.indices, M = this._mvp, eye = this._eyeAt, W = rect.width, H = rect.height;
      const CELL = 6, cw = Math.ceil(W / CELL) + 1, zbuf = new Float32Array(cw * (Math.ceil(H / CELL) + 1)).fill(Infinity);
      const nTri = I.length / 3, sx = new Float32Array(nTri), sy = new Float32Array(nTri), sd = new Float32Array(nTri);
      const front = new Uint8Array(nTri);
      for (let t = 0; t < nTri; t++) {
        const a = I[3 * t] * 3, b = I[3 * t + 1] * 3, c = I[3 * t + 2] * 3;
        const cx = (P[a] + P[b] + P[c]) / 3, cy = (P[a + 1] + P[b + 1] + P[c + 1]) / 3, cz = (P[a + 2] + P[b + 2] + P[c + 2]) / 3;
        const w = M[3] * cx + M[7] * cy + M[11] * cz + M[15];
        if (w <= 1e-6) continue;
        const px = ((M[0] * cx + M[4] * cy + M[8] * cz + M[12]) / w * 0.5 + 0.5) * W;
        const py = (1 - ((M[1] * cx + M[5] * cy + M[9] * cz + M[13]) / w * 0.5 + 0.5)) * H;
        const d = Math.hypot(cx - eye[0], cy - eye[1], cz - eye[2]);
        sx[t] = px; sy[t] = py; sd[t] = d;
        const k = Math.floor(py / CELL) * cw + Math.floor(px / CELL);
        if (px >= 0 && py >= 0 && px < W && py < H && d < zbuf[k]) zbuf[k] = d;
        front[t] = 1;
      }
      const tris = [];
      for (let t = 0; t < nTri; t++) {
        if (!front[t] || sx[t] < ax || sx[t] > bx || sy[t] < ay || sy[t] > by) continue;
        const k = Math.floor(sy[t] / CELL) * cw + Math.floor(sx[t] / CELL);
        if (sd[t] <= zbuf[k] + 0.01) tris.push(t);
      }
      const vertices = new Set(), points = [], normals = [];
      const step = Math.max(1, Math.ceil(tris.length / maxPts));
      tris.forEach((t, j) => {
        const a = I[3 * t] * 3, b = I[3 * t + 1] * 3, c = I[3 * t + 2] * 3;
        vertices.add(I[3 * t]); vertices.add(I[3 * t + 1]); vertices.add(I[3 * t + 2]);
        if (j % step) return;
        const cen = [(P[a] + P[b] + P[c]) / 3, (P[a + 1] + P[b + 1] + P[c + 1]) / 3, (P[a + 2] + P[b + 2] + P[c + 2]) / 3];
        let n = norm(cross([P[b] - P[a], P[b + 1] - P[a + 1], P[b + 2] - P[a + 2]], [P[c] - P[a], P[c + 1] - P[a + 1], P[c + 2] - P[a + 2]]));
        if (dot(n, sub(cen, eye)) > 0) n = [-n[0], -n[1], -n[2]];        // 보는 쪽을 바깥으로
        points.push(cen.map((v) => Math.round(v * 1e4) / 1e4));
        normals.push(n.map((v) => Math.round(v * 1e3) / 1e3));
      });
      return { tris: Uint32Array.from(tris), vertices, points, normals };
    }

    // markers: [{p:[x,y,z], color:[r,g,b] 0..1, size:px}]
    setMarkers(markers) {
      this.markers = markers || [];
      if (!this.ok) return;
      const gl = this.gl, data = [];
      for (const m of this.markers) data.push(m.p[0], m.p[1], m.p[2], m.color[0], m.color[1], m.color[2], m.size || 16);
      gl.bindVertexArray(this.markVao);
      gl.bindBuffer(gl.ARRAY_BUFFER, this.markBuf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(data), gl.DYNAMIC_DRAW);
      const stride = 7 * 4;
      for (const [name, size, off] of [["aPos", 3, 0], ["aCol", 3, 12], ["aSize", 1, 24]]) {
        const loc = gl.getAttribLocation(this.mark.p, name);
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, size, gl.FLOAT, false, stride, off);
      }
      gl.bindVertexArray(null);
      this.redraw();
    }

    _eye() {
      const c = this.cam, cp = Math.cos(c.pitch);
      return [c.target[0] + c.dist * cp * Math.cos(c.yaw), c.target[1] + c.dist * cp * Math.sin(c.yaw),
        c.target[2] + c.dist * Math.sin(c.pitch)];
    }

    // 화면 좌표 → 메시 위 가장 가까운 점과 카메라 쪽 면 법선. 메시가 없거나 빗나가면 null.
    pick(clientX, clientY) {
      const m = this.model;
      if (!m || !m.triangles) return null;
      const rect = this.canvas.getBoundingClientRect();
      const x = (2 * (clientX - rect.left)) / rect.width - 1, y = 1 - (2 * (clientY - rect.top)) / rect.height;
      const eye = this._eye(), fwd = norm(sub(this.cam.target, eye));
      const right = norm(cross(fwd, [0, 0, 1])), up = cross(right, fwd);
      const t = Math.tan(FOV / 2), aspect = rect.width / Math.max(1, rect.height);
      const dir = norm([0, 1, 2].map((k) => fwd[k] + x * t * aspect * right[k] + y * t * up[k]));
      const P = m.positions, I = m.indices;
      let best = Infinity, bestTri = -1;
      for (let i = 0; i < I.length; i += 3) {     // Möller–Trumbore
        const a = I[i] * 3, b = I[i + 1] * 3, c = I[i + 2] * 3;
        const e1x = P[b] - P[a], e1y = P[b + 1] - P[a + 1], e1z = P[b + 2] - P[a + 2];
        const e2x = P[c] - P[a], e2y = P[c + 1] - P[a + 1], e2z = P[c + 2] - P[a + 2];
        const px = dir[1] * e2z - dir[2] * e2y, py = dir[2] * e2x - dir[0] * e2z, pz = dir[0] * e2y - dir[1] * e2x;
        const det = e1x * px + e1y * py + e1z * pz;
        if (Math.abs(det) < 1e-12) continue;
        const inv = 1 / det;
        const tx = eye[0] - P[a], ty = eye[1] - P[a + 1], tz = eye[2] - P[a + 2];
        const u = (tx * px + ty * py + tz * pz) * inv;
        if (u < 0 || u > 1) continue;
        const qx = ty * e1z - tz * e1y, qy = tz * e1x - tx * e1z, qz = tx * e1y - ty * e1x;
        const v = (dir[0] * qx + dir[1] * qy + dir[2] * qz) * inv;
        if (v < 0 || u + v > 1) continue;
        const d = (e2x * qx + e2y * qy + e2z * qz) * inv;
        if (d > 1e-6 && d < best) { best = d; bestTri = i; }
      }
      if (bestTri < 0) return null;
      const a = I[bestTri] * 3, b = I[bestTri + 1] * 3, c = I[bestTri + 2] * 3;
      let n = norm(cross([P[b] - P[a], P[b + 1] - P[a + 1], P[b + 2] - P[a + 2]],
        [P[c] - P[a], P[c + 1] - P[a + 1], P[c + 2] - P[a + 2]]));
      if (dot(n, dir) > 0) n = [-n[0], -n[1], -n[2]];     // 보이는 쪽(카메라 쪽)을 바깥으로
      const point = [0, 1, 2].map((k) => eye[k] + dir[k] * best);
      return { point, normal: n };
    }

    _draw() {
      if (!this.ok) return;
      const gl = this.gl, canvas = this.canvas;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = Math.round(canvas.clientWidth * dpr), h = Math.round(canvas.clientHeight * dpr);
      if (!w || !h) return;
      if (this.needFit) this.fit();
      if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; this.dirty = true; }
      if (!this.dirty) return;
      this.dirty = false;
      const cs = getComputedStyle(document.documentElement);
      const rgba = (name) => cs.getPropertyValue(name).trim().match(/[\d.]+/g).map((v, i) => (i < 3 ? +v / 255 : +v));
      const bg = rgba("--canvas");
      gl.viewport(0, 0, w, h);
      gl.clearColor(bg[0], bg[1], bg[2], 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);
      const eye = this._eye();
      const view = lookAt(eye, this.cam.target, [0, 0, 1]);
      const proj = perspective(FOV, w / h, this.cam.dist * 0.01, this.cam.dist * 50);
      const mvp = mul(proj, view);
      this._mvp = mvp;                       // 영역 고르기가 같은 투영을 쓴다
      this._eyeAt = eye;

      if (this.opts.grid) {
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        gl.depthMask(false);
        gl.useProgram(this.line.p);
        gl.uniformMatrix4fv(this.line.uMVP, false, mvp);
        gl.bindVertexArray(this.grid.vao);
        gl.uniform4fv(this.line.uColor, rgba("--grid"));
        gl.drawArrays(gl.LINES, 0, this.grid.grid);
        gl.uniform4fv(this.line.uColor, rgba("--edge"));
        gl.drawArrays(gl.LINES, this.grid.grid, this.grid.edge);
        const axis = [[0.86, 0.25, 0.25, 1], [0.25, 0.7, 0.3, 1], [0.3, 0.45, 0.95, 1]];
        for (let k = 0; k < 3; k++) {
          gl.uniform4fv(this.line.uColor, axis[k]);
          gl.drawArrays(gl.LINES, this.grid.grid + this.grid.edge + 2 * k, 2);
        }
        gl.depthMask(true);
        gl.disable(gl.BLEND);
      }
      const m = this.model;
      if (m) {
        const s = this.surf;
        gl.useProgram(s.p);
        gl.uniformMatrix4fv(s.uMVP, false, mvp);
        gl.uniformMatrix4fv(s.uView, false, view);
        gl.uniform1i(s.uLit, m.triangles && m.hasNormals ? 1 : 0);
        gl.uniform1i(s.uUseColor, m.overlay || (m.hasColor && this.opts.color) ? 1 : 0);
        gl.uniform1f(s.uPointSize, Math.max(1.5, 2.5 * dpr));
        gl.bindVertexArray(m.vao);
        if (!m.hasColor) gl.vertexAttrib3f(gl.getAttribLocation(s.p, "aCol"), 0.78, 0.78, 0.78);
        if (!m.hasNormals) gl.vertexAttrib3f(gl.getAttribLocation(s.p, "aNor"), 0, 0, 1);
        if (m.triangles) gl.drawElements(gl.TRIANGLES, m.count, gl.UNSIGNED_INT, 0);
        else gl.drawArrays(gl.POINTS, 0, m.count);
      }
      if (this.markers.length) {
        gl.disable(gl.DEPTH_TEST);
        gl.useProgram(this.mark.p);
        gl.uniformMatrix4fv(this.mark.uMVP, false, mvp);
        gl.bindVertexArray(this.markVao);
        gl.drawArrays(gl.POINTS, 0, this.markers.length);
        gl.enable(gl.DEPTH_TEST);
      }
      gl.bindVertexArray(null);
    }

    // ---------- 조작 ----------
    _bindInput() {
      const c = this.canvas, pointers = new Map();
      let pinch = null, moved = 0;
      const rotate = (dx, dy) => {
        this.cam.yaw -= dx * 0.008;
        this.cam.pitch = Math.max(-1.55, Math.min(1.55, this.cam.pitch + dy * 0.008));
        this.redraw();
      };
      const pan = (dx, dy) => {
        const cam = this.cam, s = (cam.dist * 2 * Math.tan(FOV / 2)) / c.clientHeight;
        const right = [-Math.sin(cam.yaw), Math.cos(cam.yaw), 0];
        const up = [-Math.sin(cam.pitch) * Math.cos(cam.yaw), -Math.sin(cam.pitch) * Math.sin(cam.yaw), Math.cos(cam.pitch)];
        for (let k = 0; k < 3; k++) cam.target[k] += (-dx * right[k] + dy * up[k]) * s;
        this.redraw();
      };
      const zoom = (f) => { this.cam.dist = Math.max(0.05, Math.min(20, this.cam.dist * f)); this.redraw(); };
      let box = null;
      const boxEl = document.createElement("div");
      boxEl.className = "selbox";
      boxEl.hidden = true;
      c.parentElement.appendChild(boxEl);
      const drawBox = () => {
        const r = c.getBoundingClientRect();
        boxEl.style.left = `${Math.min(box.x0, box.x1) - r.left}px`; boxEl.style.top = `${Math.min(box.y0, box.y1) - r.top}px`;
        boxEl.style.width = `${Math.abs(box.x1 - box.x0)}px`; boxEl.style.height = `${Math.abs(box.y1 - box.y0)}px`;
      };
      c.addEventListener("pointerdown", (e) => {
        c.setPointerCapture(e.pointerId);
        pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, button: e.button, shift: e.shiftKey });
        moved = 0;
        if (this.selectEnabled && e.button === 0 && !e.shiftKey && pointers.size === 1) {
          box = { x0: e.clientX, y0: e.clientY, x1: e.clientX, y1: e.clientY };
        }
      });
      c.addEventListener("pointermove", (e) => {
        const p = pointers.get(e.pointerId);
        if (!p) return;
        const dx = e.clientX - p.x, dy = e.clientY - p.y;
        p.x = e.clientX; p.y = e.clientY;
        moved += Math.abs(dx) + Math.abs(dy);
        if (box) {                            // 영역 고르기: 돌리지 않고 사각형을 그린다
          box.x1 = e.clientX; box.y1 = e.clientY;
          boxEl.hidden = moved < 5;
          if (!boxEl.hidden) drawBox();
          return;
        }
        if (pointers.size === 2) {
          const [a, b] = [...pointers.values()];
          const d = Math.hypot(a.x - b.x, a.y - b.y), mid = [(a.x + b.x) / 2, (a.y + b.y) / 2];
          if (pinch) { zoom(pinch.d / Math.max(1, d)); pan(mid[0] - pinch.mid[0], mid[1] - pinch.mid[1]); }
          pinch = { d, mid };
          return;
        }
        if (p.button === 2 || p.button === 1 || p.shift) pan(dx, dy); else rotate(dx, dy);
      });
      const release = (e) => {
        const p = pointers.get(e.pointerId);
        pointers.delete(e.pointerId);
        if (pointers.size < 2) pinch = null;
        if (box) {
          const b = box;
          box = null;
          boxEl.hidden = true;
          if (e.type === "pointerup" && moved >= 5 && this.onSelect) {
            const sel = this.selectRect(b.x0, b.y0, b.x1, b.y1);
            if (sel && sel.tris.length) this.onSelect(sel);
          }
          if (moved >= 5) return;
        }
        // 거의 안 움직인 왼쪽 클릭 = 점 고르기(더블클릭의 두 번째 클릭은 뺀다)
        if (e.type === "pointerup" && p && p.button === 0 && moved < 5 && this.pickEnabled && this.onPick
            && e.timeStamp - (this._lastPick || -1e9) > 400) {
          this._lastPick = e.timeStamp;
          const hit = this.pick(e.clientX, e.clientY);
          if (hit) this.onPick(hit, e);
        }
      };
      c.addEventListener("pointerup", release);
      c.addEventListener("pointercancel", release);
      c.addEventListener("contextmenu", (e) => e.preventDefault());
      c.addEventListener("wheel", (e) => { e.preventDefault(); zoom(Math.exp(e.deltaY * 0.0015)); }, { passive: false });
      c.addEventListener("dblclick", () => { if (!this.pickEnabled) this.fit(); });
    }
  }

  root.MeshView = MeshView;
})(typeof window !== "undefined" ? window : globalThis);
