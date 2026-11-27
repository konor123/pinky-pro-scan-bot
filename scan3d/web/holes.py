"""메시의 구멍 후보: 한 삼각형에만 쓰인 변(열린 경계)을 고리로 묶는다(docs/specs/webui.md).

바닥은 `floor_cut`에서 잘려 늘 열려 있어 그 테두리는 뺀다.
"""
import math

import numpy as np

MIN_EDGES = 8
MAX_HOLES = 10
FLOOR_MARGIN = 0.01


def boundary_loops(triangles):
    """삼각형 (N, 3) → 열린 경계 변을 꼭짓점으로 이은 덩어리들 [(꼭짓점 번호 배열, 변 배열)]."""
    t = np.asarray(triangles, np.int64)
    if t.size == 0:
        return []
    e = np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]])
    e.sort(axis=1)
    uniq, counts = np.unique(e, axis=0, return_counts=True)
    edges = uniq[counts == 1]
    if not len(edges):
        return []
    parent = {}

    def find(a):
        root = a
        while parent.get(root, root) != root:
            root = parent[root]
        while parent.get(a, a) != root:
            parent[a], a = root, parent[a]
        return root

    for a, b in edges.tolist():
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    groups = {}
    for i, (a, _) in enumerate(edges.tolist()):
        groups.setdefault(find(a), []).append(i)
    out = []
    for idx in groups.values():
        es = edges[idx]
        out.append((np.unique(es), es))
    return out


def find_holes(vertices, triangles, floor_cut, max_n=MAX_HOLES):
    """→ [{center:[x,y,z], normal:[nx,ny,0], perimeter_m, n_edges}] 둘레가 긴 순서."""
    v = np.asarray(vertices, float)
    holes = []
    for verts, es in boundary_loops(triangles):
        if len(es) < MIN_EDGES:
            continue
        pts = v[verts]
        if np.all(pts[:, 2] < floor_cut + FLOOR_MARGIN):
            continue                                  # 바닥 테두리
        perim = float(np.linalg.norm(v[es[:, 0]] - v[es[:, 1]], axis=1).sum())
        c = pts.mean(axis=0)
        # 중심에 가장 가까운 고리 꼭짓점(메시 위 점이어야 한다)
        near = pts[np.argmin(np.linalg.norm(pts - c, axis=1))]
        r = math.hypot(c[0], c[1])
        normal = [c[0] / r, c[1] / r, 0.0] if r > 1e-6 else [0.0, 0.0, 1.0]
        holes.append({"center": [round(float(x), 4) for x in near], "normal": normal,
                      "perimeter_m": round(perim, 3), "n_edges": int(len(es))})
    holes.sort(key=lambda h: -h["perimeter_m"])
    return holes[:max_n]


def holes_from_ply(path, floor_cut):
    import open3d as o3d

    mesh = o3d.io.read_triangle_mesh(str(path))
    return find_holes(np.asarray(mesh.vertices), np.asarray(mesh.triangles), floor_cut)
