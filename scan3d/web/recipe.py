"""레시피(부품 종류별 설정)와 치수 OK/NG 판정(docs/specs/webui.md)."""
import copy
import math
from pathlib import Path

import yaml

ALLOWED = {
    "scene": {"r_obj", "place_err", "obj_h", "R", "n_views", "crop_margin", "floor_cut"},
    "drive": {"d_target", "d_min", "d_max"},
    "recon": {"voxel", "trunc", "edge_mm"},
}


class RecipeError(ValueError):
    """레시피 파일이나 값이 틀림."""


def load(path):
    """레시피 파일 → {이름: 레시피}. 파일이 없으면 기본 하나."""
    path = Path(path)
    if not path.is_file():
        return {"default": {"label": "기본"}}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict) or not data:
        raise RecipeError(f"{path}: 레시피가 없음")
    for name, r in data.items():
        validate(name, r)
    return data


def validate(name, r):
    if not isinstance(r, dict):
        raise RecipeError(f"{name}: 객체가 아님")
    for group, keys in r.items():
        if group in ("label", "spec_mm", "tol_mm"):
            continue
        if group not in ALLOWED or not isinstance(keys, dict):
            raise RecipeError(f"{name}: 덮을 수 없는 묶음 {group}")
        bad = set(keys) - ALLOWED[group]
        if bad:
            raise RecipeError(f"{name}: {group}에서 덮을 수 없는 값 {sorted(bad)}")
        for k, v in keys.items():
            if not _num(v) or v <= 0:
                raise RecipeError(f"{name}: {group}.{k}는 유한한 양수")
    n = (r.get("scene") or {}).get("n_views")
    if n is not None and (not isinstance(n, int) or isinstance(n, bool) or not 12 <= n < 100):
        raise RecipeError(f"{name}: scene.n_views는 12 ~ 99 정수")
    spec = r.get("spec_mm")
    if spec is not None and (not isinstance(spec, list) or len(spec) != 3 or not all(_num(v) and v > 0 for v in spec)):
        raise RecipeError(f"{name}: spec_mm은 [가로, 세로, 높이] 양수 세 개")
    if spec is not None and spec[0] < spec[1]:
        raise RecipeError(f"{name}: spec_mm은 가로 ≥ 세로 순서(치수 측정과 같다)")
    tol = r.get("tol_mm")
    if tol is not None:
        tols = tol if isinstance(tol, list) else [tol]
        if len(tols) not in (1, 3) or not all(_num(v) and v >= 0 for v in tols):
            raise RecipeError(f"{name}: tol_mm은 0 이상 한 값 또는 세 값")
    if spec is not None and tol is None:
        raise RecipeError(f"{name}: spec_mm이 있으면 tol_mm도 있어야 함")


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def free_radius(board_json):
    """판 중심에서 가장 가까운 표식까지 거리(m). 물체가 이보다 크면 표식을 가린다."""
    best = math.inf
    for t in board_json["tags"]:
        xs = [c[0] for c in t["corners"]]
        ys = [c[1] for c in t["corners"]]
        dx = max(min(xs), 0.0, -max(xs))
        dy = max(min(ys), 0.0, -max(ys))
        best = min(best, math.hypot(dx, dy))
    return best


def check(name, cfg, board_json):
    """레시피를 덮은 설정이 이 판과 원에서 말이 되는지."""
    sc, d = cfg["scene"], cfg["drive"]
    r_eff = float(sc["r_obj"]) + float(sc["place_err"])
    free = free_radius(board_json)
    if r_eff > free:
        raise RecipeError(f"{name}: 물체 반지름 {r_eff:.3f} m가 표식 안쪽 {free:.3f} m보다 크다(표식을 가린다)")
    if not d["d_min"] <= d.get("d_target", d["d_min"]) <= d["d_max"]:
        raise RecipeError(f"{name}: d_min ≤ d_target ≤ d_max가 아님")
    if float(sc["R"]) - r_eff < float(d["d_min"]):
        raise RecipeError(f"{name}: 원 반지름 R − 물체 반지름이 d_min보다 작다(늘 너무 가깝다)")


def apply(cfg, r):
    """config dict에 레시피 값을 덮은 사본."""
    out = copy.deepcopy(cfg)
    for group in ALLOWED:
        for k, v in (r.get(group) or {}).items():
            out.setdefault(group, {})[k] = v
    return out


def judge(dims_mm, r):
    """잰 치수 [가로, 세로, 높이] vs 레시피 기준 → 판정 dict 또는 None(기준 없음).

    {result: "OK"|"NG", spec, measured, dev(잰 값 − 기준), tol, axes: [OK|NG …]}
    """
    spec = r.get("spec_mm")
    if spec is None:
        return None
    if not dims_mm or len(dims_mm) != 3:
        return {"result": "NG", "spec": spec, "measured": None, "dev": None, "tol": r.get("tol_mm"),
                "axes": None, "reason": "치수 없음"}
    tol = r["tol_mm"]
    tols = tol if isinstance(tol, list) else [tol] * 3
    dev = [round(float(m) - float(s), 1) for m, s in zip(dims_mm, spec)]
    axes = ["OK" if abs(d) <= t else "NG" for d, t in zip(dev, tols)]
    return {"result": "OK" if all(a == "OK" for a in axes) else "NG", "spec": spec,
            "measured": [round(float(m), 1) for m in dims_mm], "dev": dev, "tol": tols, "axes": axes}


def public(name, r):
    """화면에 보일 레시피 요약."""
    return {"name": name, "label": r.get("label", name), "spec_mm": r.get("spec_mm"), "tol_mm": r.get("tol_mm"),
            "overrides": {g: dict(r[g]) for g in ALLOWED if r.get(g)}}


def dump_cfg(cfg):
    return yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False)
