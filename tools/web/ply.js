// PLY(ascii, binary_little_endian, binary_big_endian) → { positions, normals, colors, indices }.
// positions·normals: Float32Array(3n), colors: Uint8Array(3n) 또는 null, indices: Uint32Array(3m) 또는 null(점구름).
// 다각형 면은 부채꼴로 삼각형을 만든다. 법선이 없고 면이 있으면 면적 가중 꼭짓점 법선을 계산한다.
(function (root) {
  "use strict";

  const TYPES = {
    char: ["Int8", 1], int8: ["Int8", 1], uchar: ["Uint8", 1], uint8: ["Uint8", 1],
    short: ["Int16", 2], int16: ["Int16", 2], ushort: ["Uint16", 2], uint16: ["Uint16", 2],
    int: ["Int32", 4], int32: ["Int32", 4], uint: ["Uint32", 4], uint32: ["Uint32", 4],
    float: ["Float32", 4], float32: ["Float32", 4], double: ["Float64", 8], float64: ["Float64", 8],
  };

  function type(name) {
    const t = TYPES[name];
    if (!t) throw new Error("PLY: 모르는 형식 " + name);
    return t;
  }

  function parseHeader(bytes) {
    // "end_header" 줄 끝까지 ASCII로 읽는다.
    const limit = Math.min(bytes.length, 1 << 16);
    let text = "";
    for (let i = 0; i < limit; i++) text += String.fromCharCode(bytes[i]);
    const m = /end_header\r?\n/.exec(text);
    if (!text.startsWith("ply") || !m) throw new Error("PLY 머리말이 아님");
    const header = { format: null, elements: [], bodyStart: m.index + m[0].length };
    for (const raw of text.slice(0, m.index).split(/\r?\n/)) {
      const w = raw.trim().split(/\s+/);
      if (w[0] === "format") header.format = w[1];
      else if (w[0] === "element") header.elements.push({ name: w[1], count: parseInt(w[2], 10), props: [] });
      else if (w[0] === "property") {
        const el = header.elements[header.elements.length - 1];
        if (w[1] === "list") el.props.push({ name: w[4], list: true, countType: type(w[2]), itemType: type(w[3]) });
        else el.props.push({ name: w[2], list: false, type: type(w[1]) });
      }
    }
    if (!["ascii", "binary_little_endian", "binary_big_endian"].includes(header.format))
      throw new Error("PLY: 모르는 format " + header.format);
    return header;
  }

  // 요소 하나를 읽어 { 속성 이름: 값 배열 } (list는 배열의 배열 대신 평탄화한 값과 시작 위치).
  function readBinary(view, pos, el, little) {
    const cols = {};
    for (const p of el.props) {
      cols[p.name] = p.list ? { offsets: new Uint32Array(el.count + 1), items: [] } : new Float64Array(el.count);
    }
    const fixed = el.props.every((p) => !p.list);
    if (fixed) {
      // 고정 길이 레코드: 속성마다 오프셋을 미리 구해 빠르게 읽는다.
      let stride = 0;
      const offs = el.props.map((p) => { const o = stride; stride += p.type[1]; return o; });
      const getters = el.props.map((p) => DataView.prototype["get" + p.type[0]]);
      for (let i = 0; i < el.count; i++) {
        const base = pos + i * stride;
        for (let j = 0; j < el.props.length; j++) cols[el.props[j].name][i] = getters[j].call(view, base + offs[j], little);
      }
      return { cols, pos: pos + el.count * stride };
    }
    for (let i = 0; i < el.count; i++) {
      for (const p of el.props) {
        if (p.list) {
          const c = view["get" + p.countType[0]](pos, little); pos += p.countType[1];
          const col = cols[p.name];
          for (let k = 0; k < c; k++) { col.items.push(view["get" + p.itemType[0]](pos, little)); pos += p.itemType[1]; }
          col.offsets[i + 1] = col.items.length;
        } else {
          cols[p.name][i] = view["get" + p.type[0]](pos, little); pos += p.type[1];
        }
      }
    }
    return { cols, pos };
  }

  function readAscii(tokens, at, el) {
    const cols = {};
    for (const p of el.props) {
      cols[p.name] = p.list ? { offsets: new Uint32Array(el.count + 1), items: [] } : new Float64Array(el.count);
    }
    for (let i = 0; i < el.count; i++) {
      for (const p of el.props) {
        if (p.list) {
          const c = +tokens[at++];
          const col = cols[p.name];
          for (let k = 0; k < c; k++) col.items.push(+tokens[at++]);
          col.offsets[i + 1] = col.items.length;
        } else cols[p.name][i] = +tokens[at++];
      }
    }
    return { cols, at };
  }

  function computeNormals(pos, idx) {
    const n = new Float32Array(pos.length);
    for (let t = 0; t < idx.length; t += 3) {
      const a = idx[t] * 3, b = idx[t + 1] * 3, c = idx[t + 2] * 3;
      const ux = pos[b] - pos[a], uy = pos[b + 1] - pos[a + 1], uz = pos[b + 2] - pos[a + 2];
      const vx = pos[c] - pos[a], vy = pos[c + 1] - pos[a + 1], vz = pos[c + 2] - pos[a + 2];
      const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      for (const k of [a, b, c]) { n[k] += nx; n[k + 1] += ny; n[k + 2] += nz; }
    }
    for (let i = 0; i < n.length; i += 3) {
      const l = Math.hypot(n[i], n[i + 1], n[i + 2]) || 1;
      n[i] /= l; n[i + 1] /= l; n[i + 2] /= l;
    }
    return n;
  }

  function parsePLY(buffer) {
    const bytes = new Uint8Array(buffer);
    const header = parseHeader(bytes);
    const data = {};
    if (header.format === "ascii") {
      let body = "";
      const CH = 1 << 15;
      for (let i = header.bodyStart; i < bytes.length; i += CH)
        body += String.fromCharCode.apply(null, bytes.subarray(i, Math.min(i + CH, bytes.length)));
      const tokens = body.trim().split(/\s+/);
      let at = 0;
      for (const el of header.elements) { const r = readAscii(tokens, at, el); data[el.name] = r.cols; at = r.at; }
    } else {
      const view = new DataView(buffer);
      const little = header.format === "binary_little_endian";
      let pos = header.bodyStart;
      for (const el of header.elements) { const r = readBinary(view, pos, el, little); data[el.name] = r.cols; pos = r.pos; }
    }

    const v = data.vertex;
    if (!v || !v.x) throw new Error("PLY: vertex x,y,z 없음");
    const n = v.x.length;
    const positions = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) { positions[3 * i] = v.x[i]; positions[3 * i + 1] = v.y[i]; positions[3 * i + 2] = v.z[i]; }

    let colors = null;
    const r = v.red || v.r, g = v.green || v.g, b = v.blue || v.b;
    if (r && g && b) {
      const vprops = header.elements.find((e) => e.name === "vertex").props;
      const rp = vprops.find((p) => p.name === "red" || p.name === "r");
      const scale = rp.type[0].startsWith("Float") ? 255 : rp.type[1] === 2 ? 1 / 257 : 1;
      colors = new Uint8Array(n * 3);
      for (let i = 0; i < n; i++) {
        colors[3 * i] = Math.min(255, Math.round(r[i] * scale));
        colors[3 * i + 1] = Math.min(255, Math.round(g[i] * scale));
        colors[3 * i + 2] = Math.min(255, Math.round(b[i] * scale));
      }
    }

    let indices = null;
    const f = data.face && (data.face.vertex_indices || data.face.vertex_index);
    if (f && f.items.length) {
      const tri = [];
      for (let i = 0; i + 1 < f.offsets.length; i++) {
        const s = f.offsets[i], e = f.offsets[i + 1];
        for (let k = s + 1; k + 1 < e; k++) tri.push(f.items[s], f.items[k], f.items[k + 1]);
      }
      indices = Uint32Array.from(tri);
    }

    let normals = null;
    if (v.nx && v.ny && v.nz) {
      normals = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) { normals[3 * i] = v.nx[i]; normals[3 * i + 1] = v.ny[i]; normals[3 * i + 2] = v.nz[i]; }
    } else if (indices) normals = computeNormals(positions, indices);

    return { positions, normals, colors, indices, vertexCount: n, triangleCount: indices ? indices.length / 3 : 0 };
  }

  root.parsePLY = parsePLY;
  if (typeof module !== "undefined" && module.exports) module.exports = { parsePLY };
})(typeof window !== "undefined" ? window : globalThis);
