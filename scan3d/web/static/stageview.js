// 실시간 3D 스테이지 뷰어: WebGL2 + 2D HUD 오버레이(외부 라이브러리 없음).
// 표식 판, 대상 상자, 기준 원·우회 차선, 장애물, 움직이는 Pinky Pro(L·H) + D455 시야각·촬영 플래시·LiDAR·궤적을 그린다.
(function (root) {
  "use strict";

  const FOV = (42 * Math.PI) / 180;
  const ROLE_RGB = { L: [0.12, 0.68, 0.36], H: [0.16, 0.48, 0.90] };
  const ROLE_CSS = { L: "#1f9d55", H: "#1f6fd1" };
  const ACTIVITY_LABEL = {
    idle: "대기",
    moving: "이동 중",
    capturing: "촬영 중",
    waiting: "장애물 대기",
    bypassing: "우회 중",
    holding: "일시정지",
  };
  const BOX_COLORS = {
    px: [0.85, 0.10, 0.10],
    nx: [0.10, 0.65, 0.15],
    py: [0.10, 0.25, 0.85],
    ny: [0.90, 0.80, 0.10],
    pz: [0.88, 0.88, 0.90],
    nz: [0.40, 0.40, 0.42],
  };

  // ---------- 행렬·벡터 ----------
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  function norm(a) { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; }
  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
  }
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
  function wrapAngle(a) {
    let v = (a + Math.PI) % (2 * Math.PI);
    if (v < 0) v += 2 * Math.PI;
    return v - Math.PI;
  }
  function lerpAngle(a, b, t) { return wrapAngle(a + wrapAngle(b - a) * t); }

  // 로컬(로봇 좌표) → 월드(판 좌표) 변환: 회전(yaw, optional pitch) + 평행이동
  function xformPt(p, origin, yaw, pitch = 0) {
    let [x, y, z] = p;
    if (pitch) {
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const nx = cp * x + sp * z;
      const nz = -sp * x + cp * z;
      x = nx; z = nz;
    }
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    return [origin[0] + cy * x - sy * y, origin[1] + sy * x + cy * y, origin[2] + z];
  }
  function xformVec(v, yaw, pitch = 0) {
    let [x, y, z] = v;
    if (pitch) {
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const nx = cp * x + sp * z;
      const nz = -sp * x + cp * z;
      x = nx; z = nz;
    }
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    return [cy * x - sy * y, sy * x + cy * y, z];
  }

  // ---------- 셰이더 ----------
  const TRI_VS = `#version 300 es
  in vec3 aPos; in vec3 aNor; in vec4 aCol;
  uniform mat4 uMVP;
  out vec3 vNor; out vec4 vCol;
  void main() { gl_Position = uMVP * vec4(aPos, 1.0); vNor = aNor; vCol = aCol; }`;
  const TRI_FS = `#version 300 es
  precision highp float;
  in vec3 vNor; in vec4 vCol;
  out vec4 frag;
  void main() {
    vec3 ldir = normalize(vec3(0.42, 0.32, 0.85));
    float ndl = max(dot(normalize(vNor), ldir), 0.0);
    float rim = max(dot(normalize(vNor), normalize(vec3(-0.5, -0.4, 0.5))), 0.0);
    float lit = 0.48 + 0.44 * ndl + 0.14 * rim;
    frag = vec4(vCol.rgb * lit, vCol.a);
  }`;

  const LINE_VS = `#version 300 es
  in vec3 aPos; in vec4 aCol;
  uniform mat4 uMVP;
  out vec4 vCol;
  void main() { gl_Position = uMVP * vec4(aPos, 1.0); vCol = aCol; }`;
  const LINE_FS = `#version 300 es
  precision highp float;
  in vec4 vCol;
  out vec4 frag;
  void main() { frag = vCol; }`;

  const PT_VS = `#version 300 es
  in vec3 aPos; in vec4 aCol; in float aSize;
  uniform mat4 uMVP;
  out vec4 vCol;
  void main() { gl_Position = uMVP * vec4(aPos, 1.0); gl_PointSize = aSize; vCol = aCol; }`;
  const PT_FS = `#version 300 es
  precision highp float;
  in vec4 vCol;
  out vec4 frag;
  void main() {
    float r = length(gl_PointCoord - 0.5) * 2.0;
    if (r > 1.0) discard;
    frag = vec4(vCol.rgb, vCol.a * (1.0 - smoothstep(0.65, 1.0, r)));
  }`;

  // ---------- 도형 빌더 ----------
  class Batch {
    constructor() {
      this.solid = [];   // pos(3), nor(3), rgba(4)
      this.alpha = [];   // 반투명 삼각형
      this.lines = [];   // pos(3), rgba(4)
      this.pts = [];     // pos(3), rgba(4), size(1)
    }
    tri(a, b, c, nor, col, translucent = false) {
      const dst = translucent ? this.alpha : this.solid;
      const alpha = col.length > 3 ? col[3] : 1;
      for (const p of [a, b, c]) {
        dst.push(p[0], p[1], p[2], nor[0], nor[1], nor[2], col[0], col[1], col[2], alpha);
      }
    }
    quad(a, b, c, d, nor, col, translucent = false) {
      this.tri(a, b, c, nor, col, translucent);
      this.tri(a, c, d, nor, col, translucent);
    }
    line(a, b, col) {
      const alpha = col.length > 3 ? col[3] : 1;
      this.lines.push(a[0], a[1], a[2], col[0], col[1], col[2], alpha,
                      b[0], b[1], b[2], col[0], col[1], col[2], alpha);
    }
    pt(p, col, size) {
      const alpha = col.length > 3 ? col[3] : 1;
      this.pts.push(p[0], p[1], p[2], col[0], col[1], col[2], alpha, size);
    }
    // 로컬 중심 (0, 0, 0), 크기 (sx, sy, sz)인 상자를 origin + yaw + pitch로 배치
    box(origin, yaw, size, colors, pitch = 0, translucent = false, edges = null) {
      const [hx, hy, hz] = [size[0] / 2, size[1] / 2, size[2] / 2];
      const v = (x, y, z) => xformPt([x, y, z], origin, yaw, pitch);
      const p000 = v(-hx, -hy, -hz), p100 = v(hx, -hy, -hz), p110 = v(hx, hy, -hz), p010 = v(-hx, hy, -hz);
      const p001 = v(-hx, -hy, hz), p101 = v(hx, -hy, hz), p111 = v(hx, hy, hz), p011 = v(-hx, hy, hz);
      const c = Array.isArray(colors)
        ? { px: colors, nx: colors, py: colors, ny: colors, pz: colors, nz: colors }
        : colors;
      this.quad(p100, p110, p111, p101, xformVec([1, 0, 0], yaw, pitch), c.px, translucent);
      this.quad(p010, p000, p001, p011, xformVec([-1, 0, 0], yaw, pitch), c.nx, translucent);
      this.quad(p110, p010, p011, p111, xformVec([0, 1, 0], yaw, pitch), c.py, translucent);
      this.quad(p000, p100, p101, p001, xformVec([0, -1, 0], yaw, pitch), c.ny, translucent);
      this.quad(p001, p101, p111, p011, xformVec([0, 0, 1], yaw, pitch), c.pz, translucent);
      this.quad(p010, p110, p100, p000, xformVec([0, 0, -1], yaw, pitch), c.nz, translucent);
      if (edges) {
        for (const [a, b] of [
          [p000, p100], [p100, p110], [p110, p010], [p010, p000],
          [p001, p101], [p101, p111], [p111, p011], [p011, p001],
          [p000, p001], [p100, p101], [p110, p111], [p010, p011],
        ]) this.line(a, b, edges);
      }
    }
    circle(cx, cy, z, r, col, segs = 72, dashed = false) {
      for (let i = 0; i < segs; i++) {
        if (dashed && i % 2 === 1) continue;
        const a0 = (i / segs) * Math.PI * 2;
        const a1 = ((i + 1) / segs) * Math.PI * 2;
        this.line([cx + r * Math.cos(a0), cy + r * Math.sin(a0), z],
                  [cx + r * Math.cos(a1), cy + r * Math.sin(a1), z], col);
      }
    }
  }

  function parseBinarySTL(buffer) {
    if (buffer.byteLength < 84) throw new Error("invalid STL");
    const dv = new DataView(buffer);
    const n = dv.getUint32(80, true);
    if (84 + n * 50 !== buffer.byteLength) throw new Error("non-binary STL");
    const positions = new Float32Array(n * 9);
    const normals = new Float32Array(n * 9);
    let off = 84;
    for (let i = 0; i < n; i++) {
      const nx = dv.getFloat32(off, true), ny = dv.getFloat32(off + 4, true), nz = dv.getFloat32(off + 8, true);
      off += 12;
      for (let k = 0; k < 3; k++) {
        const b = i * 9 + k * 3;
        positions[b] = dv.getFloat32(off, true);
        positions[b + 1] = dv.getFloat32(off + 4, true);
        positions[b + 2] = dv.getFloat32(off + 8, true);
        normals[b] = nx; normals[b + 1] = ny; normals[b + 2] = nz;
        off += 12;
      }
      off += 2;
    }
    return { positions, normals, colors: null, indices: null, vertexCount: n * 3 };
  }

  class StageView {
    constructor(canvas, hudCanvas) {
      this.canvas = canvas;
      this.hud = hudCanvas;
      this.gl = canvas.getContext("webgl2", { antialias: true });
      this.ok = !!this.gl;
      this.cam = { target: [0, 0, 0.12], dist: 2.15, yaw: -Math.PI * 0.34, pitch: 0.62 };
      this.opts = { frustum: true, lidar: true, trail: true, shots: true };
      this.stage = null;
      this.interp = {};           // {role: {x, y, yaw}}
      this.flash = {};            // {role: untilMs}
      this.lastShotSeq = {};      // {role: seq}
      this.urdfMesh = null;       // {vao, buf, count, seq}
      this._urdfLoadingSeq = -1;
      this._mvp = null;
      if (!this.ok) return;
      const gl = this.gl;
      this.triProg = this._prog(TRI_VS, TRI_FS, ["uMVP"]);
      this.lineProg = this._prog(LINE_VS, LINE_FS, ["uMVP"]);
      this.ptProg = this._prog(PT_VS, PT_FS, ["uMVP"]);
      this.solidBuf = this._makeVao(this.triProg.p, [["aPos", 3, 0], ["aNor", 3, 12], ["aCol", 4, 24]], 40);
      this.alphaBuf = this._makeVao(this.triProg.p, [["aPos", 3, 0], ["aNor", 3, 12], ["aCol", 4, 24]], 40);
      this.lineBuf = this._makeVao(this.lineProg.p, [["aPos", 3, 0], ["aCol", 4, 12]], 28);
      this.ptBuf = this._makeVao(this.ptProg.p, [["aPos", 3, 0], ["aCol", 4, 12], ["aSize", 1, 28]], 32);
      this._bindInput();
      const loop = (ts) => { requestAnimationFrame(loop); this._frame(ts); };
      requestAnimationFrame(loop);
    }

    setPreset(name) {
      if (name === "top") {
        this.cam = { target: [0, 0, 0.05], dist: 2.25, yaw: -Math.PI / 2, pitch: 1.52 };
      } else if (name === "side") {
        this.cam = { target: [0, 0, 0.22], dist: 2.05, yaw: -Math.PI * 0.22, pitch: 0.22 };
      } else {
        this.cam = { target: [0, 0, 0.12], dist: 2.15, yaw: -Math.PI * 0.34, pitch: 0.62 };
      }
    }

    setOptions(o) { Object.assign(this.opts, o); }

    update(stage, robotsMeta) {
      this.stage = stage;
      const now = performance.now();
      this._syncUrdf(stage && stage.urdf);
      if (!stage || !stage.robots) return;
      for (const [role, r] of Object.entries(stage.robots)) {
        if (r.pose && !this.interp[role]) {
          this.interp[role] = { x: r.pose[0], y: r.pose[1], yaw: r.pose[2] };
        }
        const seq = robotsMeta && robotsMeta[role] ? robotsMeta[role].preview_seq : 0;
        if (seq && this.lastShotSeq[role] != null && seq !== this.lastShotSeq[role]) {
          this.flash[role] = now + 850;
        }
        this.lastShotSeq[role] = seq;
        if (r.activity === "capturing" || (r.shot_age_s != null && r.shot_age_s < 0.7)) {
          this.flash[role] = Math.max(this.flash[role] || 0, now + 500);
        }
      }
    }

    _clearUrdfMesh() {
      if (!this.urdfMesh || !this.ok) return;
      const gl = this.gl;
      gl.deleteVertexArray(this.urdfMesh.vao);
      gl.deleteBuffer(this.urdfMesh.buf);
      this.urdfMesh = null;
    }

    _syncUrdf(u) {
      if (!this.ok) return;
      if (!u || !u.mesh_url) {
        this._clearUrdfMesh();
        this._urdfLoadingSeq = -1;
        return;
      }
      if ((this.urdfMesh && this.urdfMesh.seq === u.seq) || this._urdfLoadingSeq === u.seq) return;
      this._urdfLoadingSeq = u.seq;
      const seq = u.seq;
      fetch(u.mesh_url, { cache: "no-store" })
        .then((res) => { if (!res.ok) throw new Error(res.status); return res.arrayBuffer(); })
        .then((buf) => {
          if (this._urdfLoadingSeq !== seq) return;
          const geom = u.mesh_format === "ply" && root.parsePLY ? root.parsePLY(buf) : parseBinarySTL(buf);
          this._uploadUrdfMesh(geom, u);
        })
        .catch(() => { if (this._urdfLoadingSeq === seq) this._urdfLoadingSeq = -1; });
    }

    _uploadUrdfMesh(geom, u) {
      const gl = this.gl;
      this._clearUrdfMesh();
      const P = geom.positions, N = geom.normals, C = geom.colors, I = geom.indices;
      const off = u.mesh_format === "ply" && u.ply_offset ? u.ply_offset : [0, 0, 0];
      const org = u.origin_xyz || [0, 0, 0];
      const dx = org[0] - off[0], dy = org[1] - off[1], dz = org[2] - off[2];
      const def = u.rgba || [0.76, 0.78, 0.82, 1.0];
      const nTris = I ? I.length / 3 : Math.floor(P.length / 9);
      if (!nTris) return;
      const out = new Float32Array(nTris * 3 * 10);
      let w = 0;
      for (let t = 0; t < nTris; t++) {
        const i0 = I ? I[3 * t] : 3 * t;
        const i1 = I ? I[3 * t + 1] : 3 * t + 1;
        const i2 = I ? I[3 * t + 2] : 3 * t + 2;
        let fnx = 0, fny = 0, fnz = 1;
        if (!N) {
          const ax = P[3 * i1] - P[3 * i0], ay = P[3 * i1 + 1] - P[3 * i0 + 1], az = P[3 * i1 + 2] - P[3 * i0 + 2];
          const bx = P[3 * i2] - P[3 * i0], by = P[3 * i2 + 1] - P[3 * i0 + 1], bz = P[3 * i2 + 2] - P[3 * i0 + 2];
          const cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
          const ln = Math.hypot(cx, cy, cz) || 1;
          fnx = cx / ln; fny = cy / ln; fnz = cz / ln;
        }
        for (const vi of [i0, i1, i2]) {
          const p3 = 3 * vi;
          out[w++] = P[p3] + dx; out[w++] = P[p3 + 1] + dy; out[w++] = P[p3 + 2] + dz;
          out[w++] = N ? N[p3] : fnx; out[w++] = N ? N[p3 + 1] : fny; out[w++] = N ? N[p3 + 2] : fnz;
          if (C) {
            out[w++] = C[p3] / 255; out[w++] = C[p3 + 1] / 255; out[w++] = C[p3 + 2] / 255; out[w++] = 1.0;
          } else {
            out[w++] = def[0]; out[w++] = def[1]; out[w++] = def[2]; out[w++] = def[3] ?? 1.0;
          }
        }
      }
      const vaoBuf = this._makeVao(this.triProg.p, [["aPos", 3, 0], ["aNor", 3, 12], ["aCol", 4, 24]], 40);
      gl.bindBuffer(gl.ARRAY_BUFFER, vaoBuf.buf);
      gl.bufferData(gl.ARRAY_BUFFER, out, gl.STATIC_DRAW);
      this.urdfMesh = { vao: vaoBuf.vao, buf: vaoBuf.buf, count: nTris * 3, seq: u.seq };
    }

    _prog(vs, fs, uniforms) {
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

    _makeVao(prog, attrs, stride) {
      const gl = this.gl;
      const vao = gl.createVertexArray();
      const buf = gl.createBuffer();
      gl.bindVertexArray(vao);
      gl.bindBuffer(gl.ARRAY_BUFFER, buf);
      for (const [name, size, off] of attrs) {
        const loc = gl.getAttribLocation(prog, name);
        if (loc >= 0) {
          gl.enableVertexAttribArray(loc);
          gl.vertexAttribPointer(loc, size, gl.FLOAT, false, stride, off);
        }
      }
      gl.bindVertexArray(null);
      return { vao, buf };
    }

    _eye() {
      const c = this.cam, cp = Math.cos(c.pitch);
      return [
        c.target[0] + c.dist * cp * Math.cos(c.yaw),
        c.target[1] + c.dist * cp * Math.sin(c.yaw),
        c.target[2] + c.dist * Math.sin(c.pitch),
      ];
    }

    _buildScene(nowMs, dark) {
      const b = new Batch();
      const st = this.stage;
      const sc = (st && st.scene) || { r_obj: 0.14, r_eff: 0.17, obj_h: 0.20, R: 0.62, n_views: 24, bypass_lane: 0.25, sector_half_deg: 25, start_clear: 0.30, stop_dist: 0.20 };
      const boardM = (st && st.board_m) || 0.594;
      const half = boardM / 2;

      // 1. 바닥 격자 (2.2 m × 2.2 m)
      const gridCol = dark ? [0.75, 0.80, 0.88, 0.13] : [0.22, 0.28, 0.35, 0.14];
      const gridLim = 1.1;
      for (let v = -gridLim; v <= gridLim + 1e-6; v += 0.1) {
        b.line([-gridLim, v, -0.002], [gridLim, v, -0.002], gridCol);
        b.line([v, -gridLim, -0.002], [v, gridLim, -0.002], gridCol);
      }

      // 2. 표식 판 (594 × 594 mm) + AprilTag 16장
      const paperCol = dark ? [0.82, 0.84, 0.86] : [0.96, 0.96, 0.95];
      b.quad([-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0], [0, 0, 1], paperCol);
      const borderCol = [0.25, 0.30, 0.36, 0.9];
      for (const [p0, p1] of [
        [[-half, -half, 0.001], [half, -half, 0.001]], [[half, -half, 0.001], [half, half, 0.001]],
        [[half, half, 0.001], [-half, half, 0.001]], [[-half, half, 0.001], [-half, -half, 0.001]],
      ]) b.line(p0, p1, borderCol);

      if (st && st.tags) {
        const ink = [0.10, 0.12, 0.14];
        const innerPaper = [0.90, 0.91, 0.92];
        for (const t of st.tags) {
          const cs = t.corners;
          if (!cs || cs.length !== 4) continue;
          // 바깥 검정 테두리 + 안쪽 흰/검정 격자 느낌으로 표식 표현
          const p = cs.map((c) => [c[0], c[1], 0.0015]);
          b.quad(p[0], p[1], p[2], p[3], [0, 0, 1], ink);
          const cx = (cs[0][0] + cs[1][0] + cs[2][0] + cs[3][0]) / 4;
          const cy = (cs[0][1] + cs[1][1] + cs[2][1] + cs[3][1]) / 4;
          const s = (k) => cs.map((c) => [cx + (c[0] - cx) * k, cy + (c[1] - cy) * k, 0.0022]);
          const m = s(0.62), inn = s(0.30);
          b.quad(m[0], m[1], m[2], m[3], [0, 0, 1], innerPaper);
          b.quad(inn[0], inn[1], inn[2], inn[3], [0, 0, 1], ink);
        }
      }

      // 원점 좌표축 (+X 빨강, +Y 초록, +Z 파랑)
      b.line([0, 0, 0.003], [0.16, 0, 0.003], [0.90, 0.22, 0.22, 1]);
      b.line([0, 0, 0.003], [0, 0.16, 0.003], [0.18, 0.74, 0.30, 1]);
      b.line([0, 0, 0.003], [0, 0, 0.14], [0.24, 0.48, 0.95, 1]);

      // 3. 물체 경계 원, 카메라 기준 원(R), 우회 차선 원(R + bypass_lane)
      b.circle(0, 0, 0.002, sc.r_obj, [0.45, 0.50, 0.58, 0.55], 48);
      b.circle(0, 0, 0.002, sc.R, dark ? [0.45, 0.72, 1.0, 0.55] : [0.15, 0.42, 0.78, 0.50], 96);
      b.circle(0, 0, 0.002, sc.R + sc.bypass_lane, [0.88, 0.58, 0.10, 0.40], 96, true);

      // 24시점 눈금
      const nViews = sc.n_views || 24;
      for (let k = 0; k < nViews; k++) {
        const a = (k / nViews) * Math.PI * 2;
        const c = Math.cos(a), s = Math.sin(a);
        b.line([(sc.R - 0.018) * c, (sc.R - 0.018) * s, 0.0025],
               [(sc.R + 0.018) * c, (sc.R + 0.018) * s, 0.0025],
               dark ? [0.6, 0.78, 1.0, 0.45] : [0.2, 0.45, 0.8, 0.45]);
      }

      // 4. 스캔 대상 물체 (로드된 URDF 메시가 있으면 정적 VAO로 렌더링, 아니면 기본 상자/원기둥)
      if (st && st.urdf && this.urdfMesh) {
        // 정적 URDF VAO는 _frame()에서 그린다
      } else if (st && st.target_box && st.target_box.length === 3) {
        const [bw, bd, bh] = st.target_box;
        b.box([0, 0, bh / 2], 0, [bw, bd, bh], BOX_COLORS, 0, false, [0.12, 0.14, 0.18, 0.7]);
      } else {
        // 실물 모드: 대상 물체 범위 원기둥 와이어 + 반투명 원통
        const r = sc.r_obj * 0.85, h = sc.obj_h, segs = 28;
        b.circle(0, 0, h, r, [0.55, 0.62, 0.72, 0.6], segs);
        for (let i = 0; i < segs; i++) {
          const a0 = (i / segs) * Math.PI * 2, a1 = ((i + 1) / segs) * Math.PI * 2;
          const p0 = [r * Math.cos(a0), r * Math.sin(a0), 0.002];
          const p1 = [r * Math.cos(a1), r * Math.sin(a1), 0.002];
          const q0 = [p0[0], p0[1], h], q1 = [p1[0], p1[1], h];
          const nx = Math.cos((a0 + a1) / 2), ny = Math.sin((a0 + a1) / 2);
          b.quad(p0, p1, q1, q0, [nx, ny, 0], [0.55, 0.65, 0.78, 0.28], true);
          if (i % 7 === 0) b.line(p0, q0, [0.55, 0.65, 0.78, 0.5]);
        }
      }

      // 5. 장애물 상자(--fake --obstacle)
      if (st && st.obstacles) {
        for (const ob of st.obstacles) {
          const [ox, oy, ow, od, oyawDeg] = ob;
          const oh = 0.20;
          b.box([ox, oy, oh / 2], (oyawDeg * Math.PI) / 180, [ow, od, oh],
                [0.92, 0.36, 0.14], 0, false, [0.35, 0.10, 0.04, 0.95]);
        }
      }

      // 6. 로봇들 (L, H)
      const labels = [];
      if (st && st.robots) {
        for (const role of ["L", "H"]) {
          const r = st.robots[role];
          if (!r) continue;
          const rgb = ROLE_RGB[role] || [0.5, 0.5, 0.5];

          // 지나온 궤적
          if (this.opts.trail && r.trail && r.trail.length >= 2) {
            const tc = [rgb[0], rgb[1], rgb[2], 0.85];
            for (let i = 1; i < r.trail.length; i++) {
              b.line([r.trail[i - 1][0], r.trail[i - 1][1], 0.012],
                     [r.trail[i][0], r.trail[i][1], 0.012], tc);
            }
          }

          // 촬영 완료된 시점 핀
          if (this.opts.shots && r.shots) {
            for (const s of r.shots) {
              if (!s.cam) continue;
              b.pt(s.cam, [rgb[0], rgb[1], rgb[2], 0.95], 8.0);
              if (s.look) {
                const tip = [s.cam[0] + s.look[0] * 0.10, s.cam[1] + s.look[1] * 0.10, s.cam[2] + s.look[2] * 0.10];
                b.line(s.cam, tip, [rgb[0], rgb[1], rgb[2], 0.65]);
              }
            }
          }

          // 실시간 LiDAR 점
          if (this.opts.lidar && r.lidar && r.lidar.length) {
            const lc = r.activity === "waiting" ? [0.98, 0.24, 0.18, 0.95] : [0.96, 0.52, 0.18, 0.78];
            for (const pt of r.lidar) {
              b.pt([pt[0], pt[1], 0.125], lc, 5.5);
            }
          }

          // 보간된 현재 로봇 자세
          if (!r.pose) continue;
          const cur = this.interp[role] || { x: r.pose[0], y: r.pose[1], yaw: r.pose[2] };
          cur.x += (r.pose[0] - cur.x) * 0.28;
          cur.y += (r.pose[1] - cur.y) * 0.28;
          cur.yaw = lerpAngle(cur.yaw, r.pose[2], 0.28);
          this.interp[role] = cur;

          const rx = cur.x, ry = cur.y, ryaw = cur.yaw;
          const m = r.mount || { x: -0.08, y: 0, z: role === "H" ? 0.45 : 0.15, yaw_deg: 90, pitch_down_deg: role === "H" ? 37 : 15 };
          const flashLeft = Math.max(0, (this.flash[role] || 0) - nowMs) / 850;
          const isCapturing = r.activity === "capturing" || flashLeft > 0;

          // Pinky Pro 차체 (0.12 × 0.11 × 0.085 m)
          const bodyCol = [0.22 + rgb[0] * 0.45, 0.24 + rgb[1] * 0.45, 0.28 + rgb[2] * 0.45];
          b.box([rx, ry, 0.068], ryaw, [0.12, 0.11, 0.085], bodyCol, 0, false, [0.08, 0.10, 0.14, 0.8]);
          // 상단 역할 색 띠
          b.box([rx, ry, 0.114], ryaw, [0.11, 0.095, 0.010], rgb, 0, false);
          // 좌·우 바퀴
          const wL = xformPt([0, 0.056, 0.027], [rx, ry, 0], ryaw);
          const wR = xformPt([0, -0.056, 0.027], [rx, ry, 0], ryaw);
          b.box(wL, ryaw, [0.054, 0.018, 0.054], [0.12, 0.13, 0.15]);
          b.box(wR, ryaw, [0.054, 0.018, 0.054], [0.12, 0.13, 0.15]);
          // RPLiDAR C1 (차체 위 z=0.13)
          b.box([rx, ry, 0.134], ryaw, [0.054, 0.054, 0.028], [0.16, 0.18, 0.22]);
          // 진행 방향 화살표 (앞 +X)
          const f0 = xformPt([0.062, 0, 0.09], [rx, ry, 0], ryaw);
          const f1 = xformPt([0.14, 0, 0.09], [rx, ry, 0], ryaw);
          const fL = xformPt([0.112, 0.022, 0.09], [rx, ry, 0], ryaw);
          const fR = xformPt([0.112, -0.022, 0.09], [rx, ry, 0], ryaw);
          b.line(f0, f1, [1, 0.92, 0.25, 0.95]);
          b.line(f1, fL, [1, 0.92, 0.25, 0.95]);
          b.line(f1, fR, [1, 0.92, 0.25, 0.95]);

          // 전방 LiDAR 감시 부채꼴 (±sector_half_deg)
          if (this.opts.lidar) {
            const halfRad = ((sc.sector_half_deg || 25) * Math.PI) / 180;
            const warnCol = r.activity === "waiting" ? [0.96, 0.22, 0.18, 0.85] : [rgb[0], rgb[1], rgb[2], 0.30];
            const dist = sc.start_clear || 0.30;
            const zSec = 0.006;
            const o = [rx, ry, zSec];
            const segs = 10;
            let prev = null;
            for (let i = 0; i <= segs; i++) {
              const a = ryaw - halfRad + (2 * halfRad * i) / segs;
              const p = [rx + dist * Math.cos(a), ry + dist * Math.sin(a), zSec];
              if (i === 0 || i === segs) b.line(o, p, warnCol);
              if (prev) b.line(prev, p, warnCol);
              prev = p;
            }
          }

          // 뒤쪽 카메라 기둥 (mast) + D455 카메라 바
          const mastBaseZ = 0.105;
          const mastH = Math.max(0.02, m.z - mastBaseZ);
          const mastPos = xformPt([m.x, m.y, mastBaseZ + mastH / 2], [rx, ry, 0], ryaw);
          b.box(mastPos, ryaw, [0.014, 0.014, mastH], [0.68, 0.72, 0.76]);

          const camPos = xformPt([m.x, m.y, m.z], [rx, ry, 0], ryaw);
          const camYaw = ryaw + (m.yaw_deg * Math.PI) / 180;
          const camPitch = (m.pitch_down_deg * Math.PI) / 180; // 양수 = 아래로 숙임 (xformPt에서 +pitch는 +x가 -z로)
          const camCol = isCapturing ? [1.0, 0.86, 0.20] : [0.22, 0.25, 0.30];
          b.box(camPos, camYaw, [0.028, 0.124, 0.030], camCol, camPitch, false, [0.06, 0.08, 0.10, 0.9]);

          // 카메라 시야각 프러스텀 (FOV 87° × 58°)
          if (this.opts.frustum || isCapturing) {
            const depth = isCapturing ? 0.62 : 0.46;
            const th = Math.tan((43.5 * Math.PI) / 180) * depth;
            const tv = Math.tan((29.0 * Math.PI) / 180) * depth;
            const c0 = xformPt([depth, th, tv], camPos, camYaw, camPitch);
            const c1 = xformPt([depth, -th, tv], camPos, camYaw, camPitch);
            const c2 = xformPt([depth, -th, -tv], camPos, camYaw, camPitch);
            const c3 = xformPt([depth, th, -tv], camPos, camYaw, camPitch);
            const pulse = isCapturing ? (0.45 + 0.55 * Math.max(0.4, flashLeft)) : 0.32;
            const fEdge = isCapturing
              ? [1.0, 0.85, 0.18, 0.95]
              : [rgb[0], rgb[1], rgb[2], 0.55];
            for (const p of [c0, c1, c2, c3]) b.line(camPos, p, fEdge);
            b.line(c0, c1, fEdge); b.line(c1, c2, fEdge); b.line(c2, c3, fEdge); b.line(c3, c0, fEdge);
            // 촬영 중/직후 광축 강조 + 반투명 광선 원뿔
            if (isCapturing) {
              const fill = [1.0, 0.88, 0.28, 0.18 * pulse];
              const faces = [[camPos, c0, c1], [camPos, c1, c2], [camPos, c2, c3], [camPos, c3, c0]];
              for (const [p0, p1, p2] of faces) {
                const n = norm(cross(sub(p1, p0), sub(p2, p0)));
                b.tri(p0, p1, p2, n, fill, true);
              }
              const aim = xformPt([depth * 1.05, 0, 0], camPos, camYaw, camPitch);
              b.line(camPos, aim, [1.0, 0.95, 0.4, 1.0]);
            }
          }

          labels.push({
            role,
            pos: [camPos[0], camPos[1], camPos[2] + 0.09],
            activity: r.activity,
            shotK: r.shot_k,
            flash: isCapturing,
          });
        }
      }
      return { batch: b, labels };
    }

    _uploadDraw(vaoBuf, data, mode, floatsPerVert) {
      if (!data.length) return;
      const gl = this.gl;
      gl.bindVertexArray(vaoBuf.vao);
      gl.bindBuffer(gl.ARRAY_BUFFER, vaoBuf.buf);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(data), gl.DYNAMIC_DRAW);
      gl.drawArrays(mode, 0, data.length / floatsPerVert);
      gl.bindVertexArray(null);
    }

    _project(p, mvp, w, h) {
      const x = mvp[0] * p[0] + mvp[4] * p[1] + mvp[8] * p[2] + mvp[12];
      const y = mvp[1] * p[0] + mvp[5] * p[1] + mvp[9] * p[2] + mvp[13];
      const clipW = mvp[3] * p[0] + mvp[7] * p[1] + mvp[11] * p[2] + mvp[15];
      if (clipW <= 1e-4) return null;
      return [(x / clipW * 0.5 + 0.5) * w, (1 - (y / clipW * 0.5 + 0.5)) * h];
    }

    _drawHud(labels, mvp, w, h, dpr, dark) {
      if (!this.hud) return;
      const ctx = this.hud.getContext("2d");
      if (this.hud.width !== w || this.hud.height !== h) {
        this.hud.width = w;
        this.hud.height = h;
      }
      ctx.clearRect(0, 0, w, h);
      ctx.save();
      ctx.scale(dpr, dpr);
      const cw = w / dpr, ch = h / dpr;
      ctx.font = '700 11px ui-monospace, "SF Mono", Menlo, sans-serif';
      ctx.textBaseline = "middle";

      // 축 라벨
      const axX = this._project([0.19, 0, 0.01], mvp, cw, ch);
      const axY = this._project([0, 0.19, 0.01], mvp, cw, ch);
      if (axX) { ctx.fillStyle = "#e04040"; ctx.fillText("+X", axX[0] - 8, axX[1]); }
      if (axY) { ctx.fillStyle = "#2eb85c"; ctx.fillText("+Y", axY[0] - 8, axY[1]); }

      // 로드된 URDF 라벨
      if (this.stage && this.stage.urdf) {
        const u = this.stage.urdf;
        const sp = this._project([0, 0, (u.obj_h || 0.18) + 0.05], mvp, cw, ch);
        if (sp) {
          const d = u.dims_mm ? `${u.dims_mm[0].toFixed(0)}×${u.dims_mm[1].toFixed(0)}×${u.dims_mm[2].toFixed(0)}mm` : "";
          const text = `URDF · ${u.name}${d ? " · " + d : ""}`;
          const tw = ctx.measureText(text).width;
          const px = Math.round(sp[0] - (tw + 14) / 2);
          const py = Math.round(sp[1] - 11);
          ctx.fillStyle = "rgba(124, 58, 237, 0.92)";
          ctx.strokeStyle = "#ddd6fe";
          ctx.lineWidth = 1.4;
          ctx.beginPath();
          ctx.roundRect(px, py, tw + 14, 20, 3);
          ctx.fill();
          ctx.stroke();
          ctx.fillStyle = "#ffffff";
          ctx.fillText(text, px + 7, py + 10.5);
        }
      }

      // 로봇 배지
      for (const lb of labels) {
        const sp = this._project(lb.pos, mvp, cw, ch);
        if (!sp) continue;
        const act = lb.flash && lb.shotK != null ? `촬영 #${lb.shotK}` : (ACTIVITY_LABEL[lb.activity] || lb.activity);
        const text = `${lb.role} · ${act}`;
        const tw = ctx.measureText(text).width;
        const px = Math.round(sp[0] - (tw + 14) / 2);
        const py = Math.round(sp[1] - 11);
        ctx.fillStyle = lb.flash ? "rgba(230, 165, 10, 0.94)"
          : lb.activity === "waiting" ? "rgba(198, 40, 40, 0.92)"
          : dark ? "rgba(20, 26, 33, 0.88)" : "rgba(255, 255, 255, 0.92)";
        ctx.strokeStyle = lb.flash ? "#fff4c2" : ROLE_CSS[lb.role] || "#888";
        ctx.lineWidth = 1.6;
        ctx.beginPath();
        ctx.roundRect(px, py, tw + 14, 20, 3);
        ctx.fill();
        ctx.stroke();
        ctx.fillStyle = lb.flash || lb.activity === "waiting" ? "#ffffff" : (dark ? "#e8eef5" : "#121820");
        ctx.fillText(text, px + 7, py + 10.5);
      }
      ctx.restore();
    }

    _frame(nowMs) {
      if (!this.ok) return;
      const canvas = this.canvas;
      const cw = canvas.clientWidth, ch = canvas.clientHeight;
      if (!cw || !ch) return;
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const w = Math.round(cw * dpr), h = Math.round(ch * dpr);
      if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }

      const cs = getComputedStyle(document.documentElement);
      const rgba = (name) => cs.getPropertyValue(name).trim().match(/[\d.]+/g).map((v, i) => (i < 3 ? +v / 255 : +v));
      const bg = rgba("--canvas");
      const dark = bg[0] < 0.5;

      const gl = this.gl;
      gl.viewport(0, 0, w, h);
      gl.clearColor(bg[0], bg[1], bg[2], 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

      const eye = this._eye();
      const view = lookAt(eye, this.cam.target, [0, 0, 1]);
      const proj = perspective(FOV, w / h, 0.03, 40.0);
      const mvp = mul(proj, view);
      this._mvp = mvp;

      const { batch, labels } = this._buildScene(nowMs, dark);

      // 불투명 삼각형
      gl.useProgram(this.triProg.p);
      gl.uniformMatrix4fv(this.triProg.uMVP, false, mvp);
      this._uploadDraw(this.solidBuf, batch.solid, gl.TRIANGLES, 10);

      // 로드된 URDF 3D 메시
      if (this.urdfMesh && this.stage && this.stage.urdf) {
        gl.bindVertexArray(this.urdfMesh.vao);
        gl.drawArrays(gl.TRIANGLES, 0, this.urdfMesh.count);
        gl.bindVertexArray(null);
      }

      // 선분
      gl.useProgram(this.lineProg.p);
      gl.uniformMatrix4fv(this.lineProg.uMVP, false, mvp);
      this._uploadDraw(this.lineBuf, batch.lines, gl.LINES, 7);

      // 반투명 삼각형 (카메라 촬영 콘 등)
      if (batch.alpha.length) {
        gl.depthMask(false);
        gl.useProgram(this.triProg.p);
        gl.uniformMatrix4fv(this.triProg.uMVP, false, mvp);
        this._uploadDraw(this.alphaBuf, batch.alpha, gl.TRIANGLES, 10);
        gl.depthMask(true);
      }

      // 점 (LiDAR, 촬영 시점 핀)
      if (batch.pts.length) {
        gl.useProgram(this.ptProg.p);
        gl.uniformMatrix4fv(this.ptProg.uMVP, false, mvp);
        this._uploadDraw(this.ptBuf, batch.pts, gl.POINTS, 8);
      }

      this._drawHud(labels, mvp, w, h, dpr, dark);
    }

    _bindInput() {
      const c = this.canvas, pointers = new Map();
      let pinch = null;
      const rotate = (dx, dy) => {
        this.cam.yaw -= dx * 0.008;
        this.cam.pitch = Math.max(0.08, Math.min(1.54, this.cam.pitch + dy * 0.008));
      };
      const pan = (dx, dy) => {
        const cam = this.cam, s = (cam.dist * 2 * Math.tan(FOV / 2)) / Math.max(1, c.clientHeight);
        const right = [-Math.sin(cam.yaw), Math.cos(cam.yaw), 0];
        const up = [-Math.sin(cam.pitch) * Math.cos(cam.yaw), -Math.sin(cam.pitch) * Math.sin(cam.yaw), Math.cos(cam.pitch)];
        for (let k = 0; k < 3; k++) cam.target[k] += (-dx * right[k] + dy * up[k]) * s;
      };
      const zoom = (f) => { this.cam.dist = Math.max(0.45, Math.min(8.0, this.cam.dist * f)); };
      c.addEventListener("pointerdown", (e) => {
        c.setPointerCapture(e.pointerId);
        pointers.set(e.pointerId, { x: e.clientX, y: e.clientY, button: e.button, shift: e.shiftKey });
      });
      c.addEventListener("pointermove", (e) => {
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
        if (p.button === 2 || p.button === 1 || p.shift) pan(dx, dy); else rotate(dx, dy);
      });
      const release = (e) => {
        pointers.delete(e.pointerId);
        if (pointers.size < 2) pinch = null;
      };
      c.addEventListener("pointerup", release);
      c.addEventListener("pointercancel", release);
      c.addEventListener("contextmenu", (e) => e.preventDefault());
      c.addEventListener("wheel", (e) => { e.preventDefault(); zoom(Math.exp(e.deltaY * 0.0015)); }, { passive: false });
      c.addEventListener("dblclick", () => this.setPreset("3d"));
    }
  }

  root.StageView = StageView;
})(typeof window !== "undefined" ? window : globalThis);
