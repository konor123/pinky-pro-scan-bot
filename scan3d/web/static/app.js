// scan3d 웹 화면: 상태줄, 모듈 패널, 카메라·모델, 명령 막대.
// 서버 상태는 GET /api/state(1초마다), 동작은 POST /api/…(docs/specs/webui.md).
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const ROLES = ["L", "H"];
  const ROLE_COLOR = { L: [0.12, 0.62, 0.33], H: [0.12, 0.44, 0.82] };
  const MARK_COLOR = { user: [0.98, 0.55, 0.1], hole: [0.62, 0.3, 0.95] };
  const SEL_TINT = [255, 214, 0];
  const PHASE = {
    hello: "연결 확인", start: "출발 자세", enter: "원 진입", views: "촬영", recon: "재구성",
    done: "완료", transit: "이동", fix: "자세 보정", capture: "촬영", plan: "계획",
    approach: "접근", return: "복귀",
  };
  const KIND = { scan: "스캔", focus: "보강", closeup: "근접" };
  const KIND_TAG = { scan: "SCAN", focus: "FOCUS", closeup: "CLOSEUP" };
  const SOURCE = { lidar: "LiDAR", odom: "odom" };

  const ui = {
    mode: null, st: null, entries: [], current: null, mtime: null, tab: "stage", hasMesh: false,
    targets: [],              // {kind:"point"|"region", source, point, normal | points, normals, vertices:Set, n}
    coverage: null,           // {scores, good, low_frac, n_views}
    showCoverage: false, showCloud: false, inflight: new Set(), pending: null, loading: false, lastJobKey: null,
  };
  const view = new MeshView($("view"));
  const stageView = new StageView($("stageCanvas"), $("stageHud"));

  // ---------- 공용 ----------
  const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
  const fmt = (v, d = 1, unit = "") => {
    if (v == null || Number.isNaN(Number(v))) return "-";
    const s = Number(v).toFixed(d);
    return `${/^-0(\.0*)?$/.test(s) ? s.slice(1) : s}${unit}`;
  };
  const hms = (s) => (s == null ? "-" : `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(Math.floor(s % 60)).padStart(2, "0")}`);
  const busy = (st) => !!(st && st.job && st.job.state === "running");
  let toastTimer = null;
  function toast(text) {
    const t = $("toast");
    t.textContent = text;
    t.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.hidden = true; }, 6000);
  }
  function setStatus(text, err) { const s = $("status"); s.textContent = text || ""; s.classList.toggle("error", !!err); }
  async function post(path, body) {
    if (ui.inflight.has(path)) return null;
    ui.inflight.add(path);
    if (ui.st) render(ui.st);
    try {
      const res = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) { toast(data.message || data.error || `요청 실패 ${res.status}`); return null; }
      if (data.mode) render(data);
      return data;
    } catch (e) {
      toast(`서버 연결 끊김: ${e.message}`);
      return null;
    } finally {
      ui.inflight.delete(path);
      if (ui.st) render(ui.st);
    }
  }
  const pressed = (id, on) => $(id).setAttribute("aria-pressed", String(!!on));
  const isPressed = (id) => $(id).getAttribute("aria-pressed") === "true";

  // ---------- 탭 ----------
  function setTab(tab) {
    ui.tab = tab;
    $("tabStage").setAttribute("aria-selected", String(tab === "stage"));
    $("tabCam").setAttribute("aria-selected", String(tab === "cam"));
    $("tabModel").setAttribute("aria-selected", String(tab === "model"));
    $("stageView").hidden = tab !== "stage";
    $("camView").hidden = tab !== "cam";
    $("modelView").hidden = tab !== "model";
    if (tab === "model") view.redraw();
  }
  $("tabStage").addEventListener("click", () => setTab("stage"));
  $("tabCam").addEventListener("click", () => setTab("cam"));
  $("tabModel").addEventListener("click", () => setTab("model"));

  // 실시간 3D 스테이지 조작 버튼
  function setStagePreset(name, btnId) {
    stageView.setPreset(name);
    for (const id of ["btnStage3d", "btnStageTop", "btnStageSide"]) $(id).classList.toggle("active", id === btnId);
  }
  $("btnStage3d").addEventListener("click", () => setStagePreset("3d", "btnStage3d"));
  $("btnStageTop").addEventListener("click", () => setStagePreset("top", "btnStageTop"));
  $("btnStageSide").addEventListener("click", () => setStagePreset("side", "btnStageSide"));
  for (const [id, key] of [["btnStageFrustum", "frustum"], ["btnStageLidar", "lidar"],
    ["btnStageTrail", "trail"], ["btnStageShots", "shots"]]) {
    $(id).addEventListener("click", () => {
      const on = !isPressed(id);
      pressed(id, on);
      stageView.setOptions({ [key]: on });
    });
  }
  $("btnStageObs").addEventListener("click", () => {
    const has = !!(ui.st && ui.st.stage && ui.st.stage.obstacles && ui.st.stage.obstacles.length);
    post("/api/sim/obstacle", { on: !has });
  });

  // ---------- 상태줄 ----------
  function renderSys(st) {
    const job = st.job, running = busy(st);
    let lamp, text;
    if (st.mode !== "control") { lamp = "info"; text = "보기"; }
    else if (running) {
      const ph = job.progress && job.progress.phase;
      const blocked = ROLES.filter((r) => st.robots[r] && st.robots[r].obstacle);
      if (blocked.length) { lamp = "hold blink"; text = `장애물 대기 · ${blocked.join(",")}`; }
      else if (job.paused) { lamp = "hold blink"; text = "일시정지"; }
      else if (job.recon) { lamp = "info blink"; text = "재구성"; }
      else { lamp = "run blink"; text = `${KIND[job.kind] || job.kind} · ${PHASE[ph] || ph || ""}`; }
    } else if (st.alarms && st.alarms.active_alarm) { lamp = "stop blink"; text = "알람"; }
    else if (job && job.state === "failed") { lamp = "stop"; text = "알람"; }
    else if (job && job.state === "cancelled") { lamp = "hold"; text = "중지됨"; }
    else if (ROLES.some((r) => st.robots[r] && !st.robots[r].connected)) { lamp = "stop"; text = "연결 끊김"; }
    else { lamp = "run"; text = "대기"; }
    $("sysLamp").className = `lamp ${lamp}`;
    $("sysState").textContent = text;
    $("sysMode").textContent = st.mode !== "control" ? "보기 전용" : st.fake ? "SIM" : "LIVE";
    $("sysJob").textContent = st.mode !== "control" ? "-" : job && job.scan_id ? `${KIND_TAG[job.kind] || "JOB"} ${job.scan_id}`
      : st.last_scan ? `SCAN ${st.last_scan.scan_id}` : "-";
    $("sysElapsed").textContent = job ? hms(job.elapsed_s) : "-";
    if (st.recipe) $("sysRecipe").textContent = st.recipe.current.label;
  }
  const tick = () => { const d = new Date(), z = (v) => String(v).padStart(2, "0"); $("sysClock").textContent = `${z(d.getHours())}:${z(d.getMinutes())}:${z(d.getSeconds())}`; };
  tick();
  setInterval(tick, 1000);

  // ---------- 모듈 패널 ----------
  // 바뀔 때만 다시 그린다(매초 버튼을 새로 만들면 누르던 클릭이 사라진다)
  function setHtml(el, html) { if (el.dataset.html !== html) { el.dataset.html = html; el.innerHTML = html; } }
  const ageLamp = (a) => (a == null ? "stop" : a < 1 ? "run" : a < 3 ? "hold" : "stop");
  const STACK = { running: ["run", "켜짐"], stopped: ["", "꺼짐"], starting: ["info blink", "켜는 중"],
    stopping: ["hold blink", "끄는 중"], blocked: ["stop", "다른 스택 사용 중"], error: ["stop", "오류"],
    unknown: ["", "-"], sim: ["info", "SIM"] };
  function stackRow(r, R, running) {
    const [lamp, text] = STACK[R.stack] || ["", R.stack];
    const off = R.stack === "sim" || R.stack_busy;
    return `<tr><th>구동</th><td><span class="lamp ${lamp}"></span>${text}
      <button type="button" class="mini on" data-stack="${r}" data-act="start" ${off || R.stack === "running" ? "disabled" : ""}>켜기</button>
      <button type="button" class="mini off" data-stack="${r}" data-act="stop" ${off || running || R.stack === "stopped" ? "disabled" : ""}>끄기</button>
      ${R.stack_msg && (R.stack === "error" || R.stack === "blocked") ? `<div class="detail" title="${esc(R.stack_msg)}">${esc(R.stack_msg.slice(0, 60))}</div>` : ""}</td></tr>`;
  }
  const frontClass = (f, lim) => (f == null ? "" : f >= lim.start_clear ? "ok" : f >= lim.stop_dist ? "warn" : "bad");
  function renderModules(st) {
    for (const r of ROLES) {
      const R = st.robots[r], el = $(`mod${r}`);
      if (!R) { setHtml(el, `<tr><th>사용</th><td>안 함</td></tr>`); setHtml($(`stack${r}`), ""); continue; }
      setHtml($(`stack${r}`), stackRow(r, R, busy(st)));
      setHtml(el, `
        <tr><th>연결</th><td><span class="lamp ${R.connected ? "run" : "stop"}"></span>${R.connected ? "ON" : "OFF"}</td></tr>
        <tr><th>S/N</th><td>${esc(R.serial || "-")}</td></tr>
        <tr><th>주소</th><td>${esc(R.addr)}</td></tr>
        <tr><th>ODOM</th><td><span class="lamp ${ageLamp(R.odom_age_s)}"></span>${fmt(R.odom_age_s, 2, " s")}</td></tr>
        <tr><th>LIDAR</th><td><span class="lamp ${ageLamp(R.scan_age_s)}"></span>${fmt(R.scan_age_s, 2, " s")}</td></tr>
        <tr><th>이동</th><td>${R.connected ? (R.arc ? "원호" : "회전+직진") : "-"}</td></tr>
        <tr><th>전방</th><td class="${frontClass(R.front_m, st.limits)}">${R.front_m == null ? "-" : R.front_m >= 9.99 ? "없음" : fmt(R.front_m, 2, " m")}</td></tr>
        ${R.obstacle ? `<tr><th>장애물</th><td class="bad">${R.obstacle.event === "stop" ? "정지" : "대기"} ${fmt(R.obstacle.since_s, 0, " s")}</td></tr>` : ""}
        ${R.error && !R.connected ? `<tr><th>오류</th><td title="${esc(R.error)}">${esc(R.error.slice(0, 40))}</td></tr>` : ""}`);
    }
    const pr = st.pair, two = Object.keys(st.robots).length > 1;
    $("gapVal").textContent = !two ? "1대" : pr.gap_deg == null ? "-" : `${Math.abs(pr.gap_deg).toFixed(0)}°`;
    const b = $("gapBadge");
    b.textContent = !two ? "-" : pr.gap_deg == null ? "대기" : pr.gap_ok ? "OK" : "NG";
    b.className = `badge ${!two || pr.gap_deg == null ? "wait" : pr.gap_ok ? "ok" : "ng"}`;
    $("modPair").innerHTML = `
      <tr><th>기준</th><td>180 ± ${st.limits.gap_tol_deg}°</td></tr>
      <tr><th>시작 조건</th><td><span class="lamp ${pr.can_start ? "run" : "stop"}"></span>${pr.can_start ? "충족" : "미충족"}</td></tr>
      ${pr.gap_deg != null && !pr.gap_ok ? `<tr><th>조정</th><td>${esc(pr.hint)}</td></tr>` : ""}`;
    renderProc(st);
  }
  function prow(label, v, n, done) {
    return `<div class="prow"><span>${label}</span><span class="ptrack ${done ? "done" : ""}"><i style="width:${n ? (100 * v) / n : 0}%"></i></span><span class="mono">${v}/${n}</span></div>`;
  }
  function renderProc(st) {
    const job = st.job;
    if (!job) { $("proc").innerHTML = `<div class="proc"><div class="pline">작업 없음</div></div>`; return; }
    const pr = job.progress || {}, done = job.state === "done";
    let rows = "";
    if (job.kind === "scan" && pr.robots) {
      for (const r of ROLES) if (pr.robots[r]) rows += prow(r, pr.robots[r].view, pr.robots[r].n, done);
    } else if (job.kind === "focus" || job.kind === "closeup") {
      rows += prow("대상", done ? pr.n_targets : Math.max(0, (pr.target || 1) - 1), pr.n_targets || 0, done);
      rows += `<div class="pline">사진 ${pr.shots || 0}장</div>`;
    }
    const s = { running: job.paused ? "일시정지" : (PHASE[pr.phase] || pr.phase || "-"), done: "완료", failed: "알람", cancelled: "중지됨" }[job.state];
    rows += `<div class="pline">${KIND[job.kind] || esc(job.kind)} · ${esc(s)}</div>`;
    if (job.message) rows += `<div class="pline" title="${esc(job.message)}">${esc(job.message.slice(0, 80))}</div>`;
    $("proc").innerHTML = `<div class="proc">${rows}</div>`;
  }

  // ---------- 판정 ----------
  function renderJudge(st) {
    const j = st.judge, r = st.recipe && st.recipe.current;
    if (!j) {
      $("judge").innerHTML = `<div class="judge-head"><span class="big none">-</span><span class="mono">${r && r.spec_mm ? "대기" : "기준 없음"}</span></div>`;
      return;
    }
    const axes = ["가로", "세로", "높이"];
    const rows = j.measured ? axes.map((a, i) => `<tr><td>${a}</td><td>${fmt(j.spec[i], 0)}</td><td>${fmt(j.measured[i], 1)}</td>
      <td class="${j.axes[i] === "OK" ? "ok" : "ng"}">${j.dev[i] > 0 ? "+" : ""}${fmt(j.dev[i], 1)}</td><td>±${fmt(j.tol[i], 0)}</td></tr>`).join("") : "";
    $("judge").innerHTML = `<div class="judge-head"><span class="big ${j.result === "OK" ? "ok" : "ng"}">${j.result}</span>
      <span class="mono">${esc(j.label || j.recipe)} · ${esc(j.source || "")}</span></div>
      ${rows ? `<table class="jt mono"><thead><tr><th>mm</th><th>기준</th><th>측정</th><th>편차</th><th>허용</th></tr></thead><tbody>${rows}</tbody></table>`
        : `<div class="pline" style="padding:6px 9px">${esc(j.reason || "")}</div>`}`;
  }

  // ---------- 레시피 ----------
  let recipeKey = "";
  function renderRecipe(st) {
    const sel = $("selRecipe"), list = st.recipe.list;
    const key = JSON.stringify(list.map((r) => [r.name, r.label]));
    if (key !== recipeKey) {
      recipeKey = key;
      sel.replaceChildren(...list.map((r) => { const o = document.createElement("option"); o.value = r.name; o.textContent = r.label; return o; }));
    }
    if (document.activeElement !== sel) sel.value = st.recipe.current.name;
    sel.disabled = busy(st) || ui.inflight.has("/api/recipe");
    const c = st.recipe.current;
    sel.title = c.spec_mm ? `기준 ${c.spec_mm.join(" × ")} mm ± ${c.tol_mm}` : "기준 없음";
  }
  $("selRecipe").addEventListener("change", (e) => post("/api/recipe", { name: e.target.value }));

  // ---------- 알람 ----------
  let alarmKey = "";
  function renderAlarms(st) {
    const a = st.alarms;
    $("alarmCount").textContent = a.unacked;
    $("alarmLamp").className = `lamp ${a.active_alarm ? "stop blink" : a.active_warn ? "hold blink" : a.unacked ? "stop" : ""}`;
    $("btnAlarm").classList.toggle("on", a.active_alarm || a.unacked > 0);
    const key = JSON.stringify(a.items.map((x) => [x.id, x.active, x.acked, x.detail]));
    if (key === alarmKey) return;
    alarmKey = key;
    $("alarmTable").innerHTML = `<tr><th>시각</th><th>코드</th><th>내용</th><th>상태</th><th></th></tr>` + (a.items.length ? a.items.map((x) =>
      `<tr class="${x.level} ${x.active ? "active" : "cleared"}"><td>${esc(x.t)}</td><td>${esc(x.code)}</td>
        <td>${esc(x.text)}${x.role ? ` · ${esc(x.role)}` : ""}<div class="detail">${esc(x.detail)}</div></td>
        <td>${x.active ? "발생" : `해제 ${esc(x.cleared_t || "")}`}${x.acked ? " · 확인" : ""}</td>
        <td>${x.acked ? "" : `<button type="button" data-ack="${x.id}">확인</button>`}</td></tr>`).join("")
      : `<tr><td colspan="5">없음</td></tr>`);
  }
  $("btnAlarm").addEventListener("click", () => { $("alarmDrawer").hidden = !$("alarmDrawer").hidden; });
  $("btnAlarmClose").addEventListener("click", () => { $("alarmDrawer").hidden = true; });
  $("btnAckAll").addEventListener("click", () => post("/api/alarm/ack", {}));
  $("alarmTable").addEventListener("click", (e) => {
    const id = e.target.dataset.ack;
    if (id) post("/api/alarm/ack", { id: +id });
  });

  // ---------- 카메라 ----------
  let camKey = "";
  function camTable(R, lim) {
    const p = R.preview;
    if (!p) return "";
    if (!p.pose) return `<table class="kv"><tr><th>표식</th><td class="bad">미검출</td></tr><tr><th>깊이(중앙)</th><td>${fmt(p.depth_center_m, 3, " m")}</td></tr></table>`;
    const c = (ok, warn) => (ok ? "ok" : warn ? "warn" : "bad");
    const dOk = p.d_m >= lim.d_min && p.d_m <= lim.d_max;
    return `<div class="grid2"><table class="kv">
        <tr><th>φ</th><td>${fmt(p.phi_deg, 1, "°")}</td></tr>
        <tr><th>거리</th><td class="${c(dOk, p.d_m > lim.d_max)}">${fmt(p.d_m, 3, " m")}</td></tr>
        <tr><th>깊이(중앙)</th><td>${fmt(p.depth_center_m, 3, " m")}</td></tr>
        <tr><th>조준</th><td class="${c(Math.abs(p.aim_err_deg) <= 10, Math.abs(p.aim_err_deg) <= 20)}">${fmt(p.aim_err_deg, 1, "°")}</td></tr>
      </table><table class="kv">
        <tr><th>숙임</th><td class="${c(Math.abs(p.pitch_err_deg) <= 5, true)}">${fmt(p.pitch_deg, 1, "°")} / ${R.pitch_cfg_deg}°</td></tr>
        <tr><th>높이</th><td class="${c(Math.abs(p.z_err_m) <= 0.03, true)}">${fmt(p.cam_z_m, 3)} / ${R.z_cfg_m} m</td></tr>
        <tr><th>표식</th><td class="${c(p.n_tags >= 6, p.n_tags >= 3)}">${p.n_tags}</td></tr>
        <tr><th>재투영</th><td class="${c(p.reproj_px < 1.5, p.reproj_px < 2.5)}">${fmt(p.reproj_px, 2, " px")}</td></tr>
      </table></div>`;
  }
  const ACT_TXT = { idle: "대기", moving: "이동 중", capturing: "촬영 중", waiting: "장애물 대기", bypassing: "우회 중", holding: "일시정지" };
  let stageCamKey = "";
  function renderStage(st) {
    if (!st.stage) return;
    stageView.update(st.stage, st.robots);
    const simGrp = $("stageSimGrp");
    simGrp.hidden = !st.fake;
    if (st.fake) {
      const nObs = (st.stage.obstacles || []).length;
      const b = $("btnStageObs");
      b.textContent = nObs ? `장애물 치우기 (${nObs})` : "장애물 놓기";
      pressed("btnStageObs", nObs > 0);
      b.disabled = ui.inflight.has("/api/sim/obstacle");
    }
    const key = JSON.stringify(ROLES.map((r) => {
      const R = st.robots[r], sr = st.stage.robots && st.stage.robots[r];
      return R && sr ? [R.preview_seq, R.connected, sr.activity, sr.shot_k, sr.shot_use,
        sr.shot_age_s != null && sr.shot_age_s < 1.2, (sr.shots || []).length] : null;
    }));
    if (key === stageCamKey) return;
    stageCamKey = key;
    $("stageCams").innerHTML = ROLES.filter((r) => st.robots[r]).map((r) => {
      const R = st.robots[r], sr = (st.stage.robots && st.stage.robots[r]) || {};
      const p = R.preview;
      const flash = sr.activity === "capturing" || (sr.shot_age_s != null && sr.shot_age_s < 1.2);
      const act = flash && sr.shot_k != null ? `촬영 #${sr.shot_k}` : (ACT_TXT[sr.activity] || "대기");
      const img = R.preview_seq
        ? `<img alt="${r}" src="/api/preview/${r}.jpg?n=${R.preview_seq}">`
        : `<div class="none">${R.connected ? "시작 또는 '카메라 값'을 누르면\n실시간 촬영 화면이 표시됩니다" : "OFFLINE"}</div>`;
      const shotBadge = sr.shot_k != null ? `<div class="shotbar mono">VIEW #${sr.shot_k} · ${sr.shot_use || ""}</div>`
        : p && p.pose ? `<div class="shotbar mono">PREVIEW</div>` : "";
      const meta = p && p.pose
        ? `<span>φ <b>${fmt(p.phi_deg, 1, "°")}</b></span><span>d <b>${fmt(p.d_m, 3, "m")}</b></span><span>표식 <b>${p.n_tags}</b></span>`
        : `<span>시점 <b>${(sr.shots || []).length}/${st.limits.n_views}</b></span><span>전방 <b>${R.front_m == null ? "-" : R.front_m >= 9.99 ? "없음" : fmt(R.front_m, 2, "m")}</b></span>`;
      return `<div class="livecam ${flash ? "flash" : ""}">
        <h3><span class="lamp ${R.connected ? "run" : "stop"}"></span>${r} CAM (${R.z_cfg_m}m · ${R.pitch_cfg_deg}°)
          <span class="act mono">${esc(act)}</span></h3>
        <div class="img">${img}${shotBadge}</div>
        <div class="meta mono">${meta}</div>
      </div>`;
    }).join("");
  }

  function renderCams(st) {
    const running = busy(st);
    const key = JSON.stringify([running, ROLES.map((r) => {
      const R = st.robots[r], sr = st.stage && st.stage.robots && st.stage.robots[r];
      return R && [R.preview_seq, R.connected, sr && sr.shot_k, sr && sr.shot_age_s != null && sr.shot_age_s < 1.2];
    })]);
    if (key !== camKey) {
      camKey = key;
      $("cams").innerHTML = ROLES.filter((r) => st.robots[r]).map((r) => {
        const R = st.robots[r], sr = (st.stage && st.stage.robots && st.stage.robots[r]) || {};
        const flash = running && (sr.activity === "capturing" || (sr.shot_age_s != null && sr.shot_age_s < 1.2));
        const img = R.preview_seq ? `<img alt="${r}" src="/api/preview/${r}.jpg?n=${R.preview_seq}">`
          : `<div class="none">${R.connected ? "NO IMAGE" : "OFFLINE"}</div>`;
        return `<div class="cam ${flash ? "flash" : ""}"><h3><span class="lamp ${R.connected ? "run" : "stop"}"></span>${r} CAM
          <span class="age mono" id="age${r}"></span></h3><div class="img">${img}</div>${camTable(R, st.limits)}</div>`;
      }).join("");
    }
    for (const r of ROLES) {
      const el = $(`age${r}`), p = st.robots[r] && st.robots[r].preview;
      const sr = st.stage && st.stage.robots && st.stage.robots[r];
      if (el) el.textContent = running && sr && sr.shot_k != null ? `LIVE #${sr.shot_k}` : p ? fmt(p.age_s, 0, " s") : "";
    }
  }

  // ---------- 명령 ----------
  function renderCmds(st) {
    const running = busy(st), job = st.job, recon = running && job.recon, f = (p) => ui.inflight.has(p);
    $("btnConnect").disabled = running || f("/api/connect") || ROLES.every((r) => !st.robots[r] || st.robots[r].connected);
    $("btnStacks").disabled = st.fake || f("/api/robot/stack/start_all")
      || ROLES.every((r) => !st.robots[r] || st.robots[r].connected || st.robots[r].stack === "running" || st.robots[r].stack_busy);
    pressed("btnPreview", st.preview_on);
    $("btnPreview").disabled = running || f("/api/preview");
    $("btnStart").disabled = running || !st.pair.can_start || f("/api/scan/start");
    $("btnPause").disabled = !running || job.paused || recon || f("/api/job/pause");
    $("btnResume").disabled = !running || !job.paused || f("/api/job/resume");
    $("btnCancel").disabled = !running || recon || f("/api/job/cancel");
  }
  $("btnConnect").addEventListener("click", () => post("/api/connect"));
  $("btnStacks").addEventListener("click", () => post("/api/robot/stack/start_all"));
  for (const r of ROLES) $(`stack${r}`).addEventListener("click", (e) => {
    const b = e.target.closest("button[data-stack]");
    if (!b) return;
    if (b.dataset.act === "stop" && !confirm(`${b.dataset.stack} 로봇 구동(LiDAR·모터·에이전트)을 끕니다`)) return;
    post("/api/robot/stack", { role: b.dataset.stack, action: b.dataset.act });
  });
  $("btnPreview").addEventListener("click", () => post("/api/preview", { on: !isPressed("btnPreview") }));
  $("btnStart").addEventListener("click", async () => { if (await post("/api/scan/start")) setTab("stage"); });
  $("btnPause").addEventListener("click", () => post("/api/job/pause"));
  $("btnResume").addEventListener("click", () => post("/api/job/resume"));
  $("btnCancel").addEventListener("click", () => { if (confirm("작업 중지: 두 로봇을 세웁니다")) post("/api/job/cancel"); });

  // ---------- 모델 ----------
  async function refreshList() {
    try {
      const res = await fetch("/api/scans", { cache: "no-store" });
      const entries = await res.json();
      const changed = JSON.stringify(entries.map((e) => [e.id, e.mtime])) !== JSON.stringify(ui.entries.map((e) => [e.id, e.mtime]));
      ui.entries = entries;
      if (changed) {
        $("selScan").replaceChildren(...entries.map((e) => { const o = document.createElement("option"); o.value = e.id; o.textContent = e.label; return o; }));
        if (ui.current) $("selScan").value = ui.current;
      }
      if (!entries.length) { setStatus("NO DATA"); return; }
      const cur = entries.find((e) => e.id === ui.current);
      if (!ui.current || ($("chkFollow").checked && entries[0].id !== ui.current && !busy(ui.st))) await load(entries[0], false);
      else if (cur && cur.mtime !== ui.mtime && !busy(ui.st)) await load(cur, true);
    } catch (e) { /* 다음 주기 */ }
  }
  const scanOf = (id) => (id || "").split("/").slice(0, 2).join("/");
  function selectEntry(id) { const e = ui.entries.find((x) => x.id === id); if (e) load(e, false); }
  // 불러오는 중에 고른 것은 끝난 뒤 불러온다(마지막 것만)
  async function load(entry, keepCamera) {
    if (ui.loading) { ui.pending = { entry, keepCamera }; return; }
    ui.loading = true;
    try {
      const hasCloud = entry.files.includes("cloud.ply"), hasMeshFile = entry.files.includes("mesh.ply");
      const name = (ui.showCloud && hasCloud) || !hasMeshFile ? "cloud.ply" : "mesh.ply";
      setStatus("LOADING");
      const res = await fetch(`/files?id=${encodeURIComponent(entry.id)}&name=${name}`, { cache: "no-store" });
      if (!res.ok) throw new Error(`${res.status}`);
      const info = view.load(await res.arrayBuffer());
      if (!keepCamera || ui.current !== entry.id) view.fit();
      if (!(ui.current && scanOf(ui.current) === scanOf(entry.id))) ui.targets = [];
      for (const t of ui.targets) if (t.kind === "region") t.vertices = null;   // 다른 메시의 꼭짓점 번호는 맞지 않는다
      ui.current = entry.id;
      ui.mtime = entry.mtime;
      ui.coverage = null;
      ui.hasMesh = name === "mesh.ply" && !!info.triangles;
      ui.hasCloud = hasCloud;
      $("selScan").value = entry.id;
      $("dims").textContent = entry.dims_mm ? `${entry.dims_mm.map((v) => v.toFixed(0)).join(" × ")} mm` : "-";
      $("lnkReport").hidden = !entry.files.includes("report.txt");
      $("lnkReport").href = `/files?id=${encodeURIComponent(entry.id)}&name=report.txt`;
      $("covLow").textContent = "-";
      setStatus(view.ok ? (info.triangles ? `TRI ${info.triangles.toLocaleString()}` : `PTS ${info.vertices.toLocaleString()}`) : "WebGL2 없음", !view.ok);
      if (ui.showCoverage) await fetchCoverage();
      applyOverlay();
      drawMarkers();
      renderTargets();
    } catch (e) {
      setStatus(`LOAD FAIL ${e.message}`, true);
    } finally {
      ui.loading = false;
      const next = ui.pending;
      ui.pending = null;
      if (next) load(next.entry, next.keepCamera);
    }
  }
  $("selScan").addEventListener("change", () => { $("chkFollow").checked = false; selectEntry($("selScan").value); });
  $("btnFit").addEventListener("click", () => view.fit());
  $("btnCloud").addEventListener("click", () => {
    ui.showCloud = !ui.showCloud;
    pressed("btnCloud", ui.showCloud);
    if (ui.showCloud) setSelMode(null);
    const e = ui.entries.find((x) => x.id === ui.current);
    if (e) load(e, true);
  });
  $("btnColor").addEventListener("click", () => { const on = !isPressed("btnColor"); pressed("btnColor", on); view.setOptions({ color: on }); applyOverlay(); });
  $("btnGrid").addEventListener("click", () => { const on = !isPressed("btnGrid"); pressed("btnGrid", on); view.setOptions({ grid: on }); });

  // 덮인 정도
  async function fetchCoverage() {
    if (!ui.current || !ui.hasMesh) return;
    const id = ui.current;
    try {
      const res = await fetch(`/api/coverage?id=${encodeURIComponent(id)}`, { cache: "no-store" });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) { toast(data.message || "덮임 계산 불가"); ui.showCoverage = false; pressed("btnCoverage", false); return; }
      if (id !== ui.current) return;
      ui.coverage = data;
      $("covLow").textContent = `${fmt(data.low_frac * 100, 1, " %")} · 시점 ${data.n_views}`;
    } catch (e) {
      toast(`덮임 계산 실패: ${e.message}`);
    }
  }
  $("btnCoverage").addEventListener("click", async () => {
    ui.showCoverage = !ui.showCoverage;
    pressed("btnCoverage", ui.showCoverage);
    if (ui.showCoverage && !ui.coverage) await fetchCoverage();
    applyOverlay();
  });
  const covColor = (s, good) => (s < 1 ? [214, 59, 50] : s < good ? [232, 170, 40] : [40, 168, 90]);
  function applyOverlay() {
    const m = view.model;
    if (!m) return;
    const regions = ui.targets.filter((t) => t.kind === "region" && t.vertices);
    const cov = ui.showCoverage && ui.coverage && ui.coverage.scores.length === m.vertexCount ? ui.coverage : null;
    $("legend").hidden = !cov;
    if (!cov && !regions.length) { view.setOverlay(null); return; }
    const n = m.vertexCount, out = new Uint8Array(n * 3);
    const useOrig = m.colors && isPressed("btnColor");
    for (let i = 0; i < n; i++) {
      const c = cov ? covColor(cov.scores[i], cov.good) : useOrig ? [m.colors[3 * i], m.colors[3 * i + 1], m.colors[3 * i + 2]] : [199, 199, 199];
      out[3 * i] = c[0]; out[3 * i + 1] = c[1]; out[3 * i + 2] = c[2];
    }
    for (const r of regions) for (const v of r.vertices) {
      for (let k = 0; k < 3; k++) out[3 * v + k] = (out[3 * v + k] + 2 * SEL_TINT[k]) / 3;
    }
    view.setOverlay(out);
  }

  // 보강 대상
  function setSelMode(mode) {
    pressed("btnSelRegion", mode === "region");
    pressed("btnSelPoint", mode === "point");
    view.selectEnabled = mode === "region";
    view.pickEnabled = mode === "point";
    $("view").classList.toggle("pick", !!mode);
    if (mode) setTab("model");
  }
  $("btnSelRegion").addEventListener("click", () => setSelMode(isPressed("btnSelRegion") ? null : "region"));
  $("btnSelPoint").addEventListener("click", () => setSelMode(isPressed("btnSelPoint") ? null : "point"));
  view.onPick = (hit) => { ui.targets.push({ kind: "point", source: "user", point: hit.point, normal: hit.normal }); drawMarkers(); renderTargets(); };
  view.onSelect = (sel) => {
    ui.targets.push({ kind: "region", source: "user", points: sel.points, normals: sel.normals, vertices: sel.vertices,
      point: sel.points[Math.floor(sel.points.length / 2)], n: sel.tris.length });
    applyOverlay(); drawMarkers(); renderTargets();
  };
  $("btnClearMarks").addEventListener("click", () => { ui.targets = []; applyOverlay(); drawMarkers(); renderTargets(); });
  $("btnHoles").addEventListener("click", async () => {
    if (!ui.current) return;
    ui.inflight.add("holes"); renderModelCmds();
    try {
      const res = await fetch(`/api/holes?id=${encodeURIComponent(ui.current)}`, { cache: "no-store" });
      const data = await res.json().catch(() => ({}));
      if (!res.ok) { toast(data.message || "구멍 계산 불가"); return; }
      if (!data.holes.length) { toast("열린 구멍 없음"); return; }
      ui.targets = ui.targets.filter((t) => t.source !== "hole")
        .concat(data.holes.map((h) => ({ kind: "point", source: "hole", point: h.center, normal: h.normal })));
      drawMarkers(); renderTargets();
    } catch (err) {
      toast(`구멍 계산 실패: ${err.message}`);
    } finally { ui.inflight.delete("holes"); renderModelCmds(); }
  });
  // focus.plan과 같은 규칙: split_z 이상이거나 위를 향한 면 → H, 그 로봇이 없으면 다른 로봇
  function roleOf(point, normal) {
    const st = ui.st, lim = st && st.limits;
    const split = lim ? lim.split_z : 0.15, upSin = Math.sin(((lim ? lim.up_deg : 45) * Math.PI) / 180);
    let role = point[2] >= split || (normal && normal[2] > upSin) ? "H" : "L";
    const ok = (r) => st && st.robots && st.robots[r] && st.robots[r].connected;
    if (st && st.robots && !ok(role)) role = ROLES.find(ok) || role;
    return role;
  }
  function targetRoles(t) {
    if (t.kind === "point") return roleOf(t.point, t.normal);
    const set = new Set(t.points.map((p, i) => roleOf(p, t.normals[i])));
    return ROLES.filter((r) => set.has(r)).join("+");
  }
  let markKey = "";
  function drawMarkers() {
    const m = ui.targets.filter((t) => t.kind === "point").map((t) => ({ p: t.point, color: MARK_COLOR[t.source], size: 15 }));
    const st = ui.st;
    if (st && st.mode === "control") {
      for (const r of ROLES) {
        const p = st.robots[r] && st.robots[r].preview;
        if (p && p.pose) m.push({ p: [p.cam_x_m, p.cam_y_m, p.cam_z_m], color: ROLE_COLOR[r], size: 12 });
      }
    }
    const key = JSON.stringify(m);
    if (key !== markKey) { markKey = key; view.setMarkers(m); }
  }
  function renderTargets() {
    $("markCount").textContent = ui.targets.length;
    $("markList").innerHTML = ui.targets.map((t, i) => {
      const c = t.kind === "region" ? SEL_TINT : MARK_COLOR[t.source].map((v) => Math.round(v * 255));
      const label = t.kind === "region" ? `영역 · 면 ${t.n}` : t.source === "hole" ? "구멍" : "점";
      return `<li><span class="sw" style="background:rgb(${c.join(",")})"></span><span>${i + 1}. ${label} <span class="mono">z ${fmt(t.point[2] * 100, 1)}</span></span>
        <span class="role">${targetRoles(t)}</span><button type="button" data-i="${i}" title="삭제">×</button></li>`;
    }).join("");
    renderModelCmds();
  }
  $("markList").addEventListener("click", (e) => {
    const i = e.target.dataset.i;
    if (i == null) return;
    ui.targets.splice(+i, 1);
    applyOverlay(); drawMarkers(); renderTargets();
  });
  function renderModelCmds() {
    const st = ui.st, control = st && st.mode === "control";
    $("btnHoles").disabled = !ui.hasMesh || ui.inflight.has("holes");
    $("btnCoverage").disabled = !ui.hasMesh;
    $("btnCloud").disabled = !ui.hasCloud;
    for (const id of ["btnSelRegion", "btnSelPoint"]) $(id).disabled = !ui.hasMesh;
    if (!control) return;
    const reasons = [];
    if (!st.last_scan) reasons.push("현재 세션 스캔 없음");
    else if (ui.current && scanOf(ui.current) !== scanOf(st.last_scan.mesh_id)) reasons.push("현재 세션 스캔 아님");
    if (busy(st)) reasons.push("작업 중");
    const blocked = !ui.targets.length || reasons.length > 0;
    $("btnFocus").disabled = blocked || ui.inflight.has("/api/focus/start");
    $("btnCloseup").disabled = blocked || ui.inflight.has("/api/closeup/start");
    const so = st.limits && st.limits.closeup_standoff;
    $("btnCloseup").title = `고른 지점에 로봇이 다가가서 찍는다${so ? `(지점에서 약 ${fmt(so * 100, 0)} cm)` : ""}`;
    $("focusReasons").textContent = ui.targets.length ? reasons.join(" · ") : "";
  }
  $("btnFocus").addEventListener("click", () => {
    const markers = ui.targets.filter((t) => t.kind === "point").map((t) => ({ point: t.point, normal: t.normal }));
    const regions = ui.targets.filter((t) => t.kind === "region").map((t) => ({ points: t.points, normals: t.normals }));
    post("/api/focus/start", { markers, regions });
  });
  // 근접 촬영: 대상마다 점 하나(영역은 대표점과 그 자리의 법선)
  $("btnCloseup").addEventListener("click", () => {
    const markers = ui.targets.map((t) => {
      if (t.kind === "point") return { point: t.point, normal: t.normal || null };
      const i = t.points ? Math.floor(t.points.length / 2) : -1;
      return { point: t.point, normal: (t.normals && t.normals[i]) || null };
    });
    post("/api/closeup/start", { markers });
  });

  function renderCrops(crops, kind) {
    const box = $("crops"), panel = $("cropPanel");
    if (!crops || !crops.length) { box.innerHTML = ""; box.dataset.key = ""; panel.hidden = true; return; }
    panel.hidden = false;
    $("cropTitle").textContent = kind === "closeup" ? "근접 사진" : "보강 사진";
    $("cropCount").textContent = crops.filter((c) => c.url).length;
    const key = JSON.stringify([kind, crops.map((c) => [c.url, c.reason, c.source])]);
    if (box.dataset.key === key) return;
    box.dataset.key = key;
    box.innerHTML = crops.map((c, i) => c.url
      ? `<figure><a href="${esc(c.url)}" target="_blank" rel="noopener"><img alt="${i + 1}" src="${esc(c.url)}"></a>
         <figcaption>#${c.target + 1} ${c.role} φ${fmt(c.phi, 0, "°")} ${fmt(c.distance_m, 2, "m")}${c.source ? ` · ${esc(SOURCE[c.source] || c.source)}` : ""}</figcaption></figure>`
      : `<figure><figcaption>#${c.target + 1} ${c.role}: ${esc(c.reason)}</figcaption></figure>`).join("");
  }
  function renderJobEnd(st) {
    const job = st.job;
    if (!job) return;
    const pr = job.progress || {};
    const shots = job.kind === "focus" || job.kind === "closeup";
    renderCrops(shots ? (job.state === "done" && job.result && job.result.crops ? job.result.crops : pr.crops) : null, job.kind);
    const key = `${job.kind}:${job.scan_id}:${job.state}`;
    if (key !== ui.lastJobKey) {
      const was = ui.lastJobKey;
      ui.lastJobKey = key;
      if (was && job.state === "done" && job.kind === "closeup") {   // 메시는 그대로: 대상만 비우고 모델 탭에 머문다
        ui.targets = [];
        applyOverlay(); drawMarkers(); renderTargets();
        setTab("model");
      } else if (was && job.state === "done" && job.result && job.result.mesh_id) {
        if (job.kind === "focus") { ui.targets = []; renderTargets(); }
        refreshList().then(() => { selectEntry(job.result.mesh_id); setTab("model"); });
      }
    }
  }

  // ---------- 기록 ----------
  $("btnLog").addEventListener("click", () => { $("drawer").hidden = !$("drawer").hidden; });
  $("btnLogClose").addEventListener("click", () => { $("drawer").hidden = true; });

  // ---------- 상태 ----------
  function render(st) {
    ui.st = st;
    if (st.mode !== ui.mode) {
      ui.mode = st.mode;
      document.body.classList.toggle("view", st.mode !== "control");
      setTab(st.mode === "control" ? ui.tab : "model");
    }
    renderSys(st);
    renderModelCmds();
    if (st.mode !== "control") return;
    renderModules(st);
    renderJudge(st);
    renderRecipe(st);
    renderAlarms(st);
    renderStage(st);
    renderCams(st);
    renderCmds(st);
    renderJobEnd(st);
    drawMarkers();
    $("lastEvent").textContent = st.log[st.log.length - 1] || "-";
    const log = $("log"), text = st.log.join("\n");
    if (log.textContent !== text) { log.textContent = text; log.scrollTop = log.scrollHeight; }
  }
  async function poll() {
    try {
      const res = await fetch("/api/state", { cache: "no-store" });
      const data = await res.json();
      if (!res.ok || !data.mode) throw new Error(data.message || res.status);   // 오류 응답을 보기 전용으로 읽지 않는다
      render(data);
    } catch (e) {
      $("sysState").textContent = "서버 끊김";
      $("sysLamp").className = "lamp stop blink";
    }
  }
  async function pollLoop() {
    await poll();
    const fast = ui.st && ui.st.mode === "control" && (busy(ui.st) || ui.tab === "stage");
    setTimeout(pollLoop, fast ? 200 : 1000);
  }
  if (location.hash === "#model") ui.tab = "model";
  else if (location.hash === "#cam") ui.tab = "cam";
  poll().then(() => { refreshList(); setTimeout(pollLoop, 300); });
  setInterval(refreshList, 3000);
})();
