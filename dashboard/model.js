// Model page of the dashboard: interactive view of the six joints and a card for each.
// Uses helpers from index.html: $, el, api, S (server state), plus showDesign/showModel routing.
const JCOL = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b"];
const ROLE = [
  { what: "Turns the whole arm about the vertical axis. It aims the arm at the pick spot and at each bin.",
    axis: "Vertical (the z axis of the base)", zero: "Arm points along +x, towards the pick spot", pos: "Counter-clockwise seen from above" },
  { what: "Tilts the whole arm forward and back. It carries the heaviest gravity load of all joints, which is why it gets the geared stepper.",
    axis: "Horizontal, across the arm (y)", zero: "Upper arm vertical", pos: "Leans the arm forward, towards +x" },
  { what: "Bends the forearm against the upper arm. Together with the shoulder it sets the reach and the height of the wrist.",
    axis: "Horizontal, parallel to the shoulder axis (y)", zero: "Forearm in line with the upper arm (straight)", pos: "Folds the forearm forward and down" },
  { what: "Rolls the wrist assembly about the forearm's own axis. Turns the hand left or right without changing where the wrist is.",
    axis: "Along the forearm", zero: "Wrist pitch axis horizontal", pos: "Right-hand rule about the forearm direction" },
  { what: "Tilts the hand up and down. With the tool pointing straight down it takes the angle that cancels shoulder and elbow.",
    axis: "Across the hand (y of the rolled wrist)", zero: "Hand in line with the forearm", pos: "Tips the hand forward" },
  { what: "Spins the gripper about its own axis. It turns the object over the bin, and is the last axis of the chain.",
    axis: "Along the gripper (the tool direction)", zero: "Fingers open along the x axis of the hand", pos: "Right-hand rule about the tool direction" },
];
const M = { data: null, q: [0, 0, 0, 0, 0, 0], sel: 0 };
const svgNS = "http://www.w3.org/2000/svg";
const sv = (tag, attrs = {}, text) => {
  const e = document.createElementNS(svgNS, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (text != null) e.textContent = text;
  return e;
};
const I4 = () => [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
const mul = (A, B) => A.map((r, i) => B[0].map((_, j) => r.reduce((s, _x, k) => s + A[i][k] * B[k][j], 0)));
const trans = (v) => { const T = I4(); T[0][3] = v[0]; T[1][3] = v[1]; T[2][3] = v[2]; return T; };
const rotM = (axis, q) => {
  const c = Math.cos(q), s = Math.sin(q), T = I4();
  if (axis === "z") { T[0][0] = c; T[0][1] = -s; T[1][0] = s; T[1][1] = c; }
  else { T[0][0] = c; T[0][2] = s; T[2][0] = -s; T[2][2] = c; }
  return T;
};

function chain(deg) {
  let T = I4();
  const origins = [], axes = [];
  M.data.joints.forEach((j, i) => {
    T = mul(mul(T, trans(j.offset_mm)), rotM(j.axis, deg[i] * Math.PI / 180));
    origins.push([T[0][3], T[1][3], T[2][3]]);
    const col = j.axis === "z" ? 2 : 1;
    axes.push([T[0][col], T[1][col], T[2][col]]);
  });
  const tip = mul(T, trans([0, 0, M.data.tcp_offset_mm]));
  return { origins, axes, tcp: [tip[0][3], tip[1][3], tip[2][3]] };
}

function drawView(svg, proj, c, ground) {
  svg.replaceChildren();
  ground(svg);
  const line = (a, b, w, col) => {
    const [x1, y1] = proj(a), [x2, y2] = proj(b);
    svg.append(sv("line", { x1, y1, x2, y2, stroke: col, "stroke-width": w, "stroke-linecap": "round" }));
  };
  const links = [[0, 0, 0], c.origins[1], c.origins[2], c.origins[3], c.origins[4]];
  for (let i = 0; i < links.length - 1; i++) line(links[i], links[i + 1], 14, "var(--line)");
  line(c.origins[4], c.tcp, 8, "var(--muted)");
  const s = M.sel, o = c.origins[s], a = c.axes[s];
  const [ax, ay] = proj(o), [bx, by] = proj([o[0] + a[0] * 70, o[1] + a[1] * 70, o[2] + a[2] * 70]);
  svg.append(sv("line", { x1: ax, y1: ay, x2: bx, y2: by, stroke: JCOL[s], "stroke-width": 3, "stroke-dasharray": "6 4" }));
  svg.append(sv("circle", { cx: bx, cy: by, r: 5, fill: JCOL[s] }));
  [0, 1, 2, 3, 5, 4].forEach((i) => {            // J5 drawn last: it shares its point with J6
    const [x, y] = proj(c.origins[i]);
    svg.append(sv("circle", { cx: x, cy: y, r: i === 5 ? 20 : 15, fill: i === 5 ? "none" : JCOL[i], stroke: JCOL[i],
      "stroke-width": i === s ? 6 : 3, opacity: i === s ? 1 : 0.85 }));
    const dx = i === 5 ? 26 : (i === 4 ? -26 : 22), dy = i === 5 ? -18 : (i === 4 ? 24 : -16);
    svg.append(sv("text", { x: x + dx, y: y + dy, fill: JCOL[i], "font-size": 20, "font-weight": 700, "text-anchor": dx < 0 ? "end" : "start" }, `J${i + 1}`));
  });
  const [tx, ty] = proj(c.tcp);
  svg.append(sv("path", { d: `M${tx - 9} ${ty - 9} L${tx + 9} ${ty + 9} M${tx + 9} ${ty - 9} L${tx - 9} ${ty + 9}`, stroke: "var(--ink)", "stroke-width": 3 }));
  svg.append(sv("text", { x: tx + 14, y: ty + 28, fill: "var(--ink)", "font-size": 18 }, "tool tip"));
}

function drawModel() {
  const c = chain(M.q), d = M.data;
  drawView($("mvSide"), (p) => [p[0], -p[2]], c, (svg) => {
    svg.append(sv("line", { x1: -100, y1: 0, x2: 550, y2: 0, stroke: "var(--ink)", "stroke-width": 3 }));
    svg.append(sv("rect", { x: -60, y: -30, width: 120, height: 30, fill: "var(--line)" }));
    svg.append(sv("text", { x: 540, y: -10, "text-anchor": "end", fill: "var(--muted)", "font-size": 18 }, "side view, +x to the right"));
  });
  drawView($("mvTop"), (p) => [p[0], -p[1]], c, (svg) => {
    const L = d.layout_mm, h = L.bin_inner / 2;
    svg.append(sv("circle", { cx: 0, cy: 0, r: 60, fill: "var(--line)" }));
    svg.append(sv("circle", { cx: L.pick[0], cy: -L.pick[1], r: 18, fill: "#e0c200", opacity: 0.8 }));
    svg.append(sv("text", { x: L.pick[0], y: -L.pick[1] + 40, "text-anchor": "middle", fill: "var(--muted)", "font-size": 16 }, "pick"));
    for (const [k, p] of Object.entries(L.bins)) {
      svg.append(sv("rect", { x: p[0] - h, y: -p[1] - h, width: 2 * h, height: 2 * h, fill: "none", stroke: k === "red" ? "#d6453d" : "#3b6fd6", "stroke-width": 4 }));
      svg.append(sv("text", { x: p[0], y: -p[1] + 6, "text-anchor": "middle", fill: "var(--muted)", "font-size": 16 }, k + " bin"));
    }
    svg.append(sv("text", { x: -340, y: 330, fill: "var(--muted)", "font-size": 18 }, "top view, +x right, +y up"));
  });
  const t = c.tcp.map((v) => v.toFixed(0));
  $("mvTcp").textContent = `Tool tip: x ${t[0]}, y ${t[1]}, z ${t[2]} mm (forward kinematics, same maths as the experiments)`;
  document.querySelectorAll(".mjcard").forEach((n, i) => n.classList.toggle("sel", i === M.sel));
  document.querySelectorAll(".mjrow").forEach((n, i) => n.classList.toggle("sel", i === M.sel));
}
function setSel(i) { M.sel = i; drawModel(); }
function setPose(deg) {
  M.q = deg.map((v, i) => Math.min(M.data.joints[i].q_max_deg, Math.max(M.data.joints[i].q_min_deg, v)));
  M.data.joints.forEach((_j, i) => { $("mq" + i).value = M.q[i]; $("mqv" + i).textContent = M.q[i].toFixed(0) + "°"; });
  drawModel();
}
const kv = (k, v) => el("div", { class: "kv" }, el("span", { text: k }), el("span", { text: v }));

async function showModel() {
  $("designView").hidden = true;
  $("modelView").hidden = false;
  $("navDesign").classList.remove("active");
  $("navModel").classList.add("active");
  const box = $("modelBody");
  box.textContent = "Loading the model…";
  let d;
  try { d = await api("/api/model"); } catch (e) { box.textContent = "Could not load the model: " + e.message; return; }
  M.data = d;
  const e2 = S.results.e2, e4 = S.results.e4, g = d.geometry_mm;
  box.replaceChildren();

  const sliders = el("div", { class: "mjsliders" });
  d.joints.forEach((j, i) => sliders.append(el("div", { class: "mjrow", onclick: () => setSel(i) },
    el("span", { class: "dot", style: `background:${JCOL[i]}` }),
    el("label", { for: "mq" + i, text: j.name }),
    el("input", { type: "range", id: "mq" + i, min: j.q_min_deg, max: j.q_max_deg, step: 1, value: 0,
      oninput: (ev) => { M.q[i] = Number(ev.target.value); $("mqv" + i).textContent = M.q[i] + "°"; M.sel = i; drawModel(); } }),
    el("span", { class: "val", id: "mqv" + i, text: "0°" }))));
  const presets = el("div", { class: "row" }, el("span", { class: "hint", text: "Pose:" }),
    el("button", { type: "button", onclick: () => setPose([0, 0, 0, 0, 0, 0]) }, "Zero (straight up)"),
    el("button", { type: "button", onclick: () => setPose(d.ready_deg) }, "Ready"),
    ...Object.entries(d.poses).filter(([k]) => k !== "ready").map(([k, v]) => el("button", { type: "button", onclick: () => setPose(v) }, k)));

  box.append(el("div", { class: "panelhead" }, el("h2", { text: "The arm, joint by joint" }),
    el("p", { class: "hint", text: "Drag a slider to move one joint. Click a joint's row or card to highlight its axis (dashed line). Drawn from the saved design values." })));
  box.append(el("div", { class: "mjviews" },
    sv("svg", { id: "mvSide", viewBox: "-100 -540 650 570", role: "img", "aria-label": "Side view of the arm" }),
    sv("svg", { id: "mvTop", viewBox: "-350 -350 700 700", role: "img", "aria-label": "Top view of the arm" })));
  box.append(el("div", { class: "hint", id: "mvTcp" }), sliders, presets);
  if (d.pose_error) box.append(el("div", { class: "msg err", style: "display:block", text: "Task poses unavailable: " + d.pose_error }));

  box.append(el("h2", { style: "margin-top:20px", text: "The six joints" }));
  const cards = el("div", { class: "mjcards" });
  d.joints.forEach((j, i) => {
    const r = ROLE[i];
    const c = el("div", { class: "mjcard", onclick: () => setSel(i) },
      el("div", { class: "top" }, el("span", { class: "dot", style: `background:${JCOL[i]}` }), el("span", { class: "name", text: j.name }),
        el("span", { class: "chip " + (j.actuator === "stepper" ? "warn" : "idle"), text: j.actuator })),
      el("p", { text: r.what }),
      kv("Rotation axis", r.axis), kv("Zero position", r.zero), kv("Positive direction", r.pos),
      kv("Joint origin", `${j.offset_mm.map((v) => v.toFixed(0)).join(", ")} mm from the previous joint`),
      kv("Range", `${j.q_min_deg}° to ${j.q_max_deg}° (${j.q_max_deg - j.q_min_deg}° total)`),
      kv("Speed / acceleration limit", `${j.vmax_deg}°/s, ${j.amax_deg}°/s²`),
      kv("Drive", j.drive),
      kv("Total reduction", `${j.ratio}:1`),
      kv("Holding torque at the joint", `${j.hold_torque_Nm} N·m`),
      ...(j.steps_per_deg ? [kv("Resolution", `${j.steps_per_deg} steps per degree (1/${j.microstep} microstep)`)] : []),
      kv("Mass on this link", `${j.link_mass_g} g (${j.parts.map((p) => p.name).join(", ")})`),
      kv("Mass it has to carry", `${j.carried_mass_g} g (this link, everything beyond it, and the ${d.payload_g} g object)`));
    if (e2) c.append(kv("Worst-case load, safety factor (E2)", e2.sf[i] > 900 ? "no gravity load" : `${e2.tau_worst[i].toFixed(2)} N·m, factor ${e2.sf[i].toFixed(1)}`));
    if (e4?.axis_travel_deg) c.append(kv("Last simulated cycle (E4)", `moved ${e4.axis_travel_deg[i].toFixed(0)}° of ${j.q_max_deg - j.q_min_deg}°, peak torque use ${(e4.peak_util[i] * 100).toFixed(0)} %`));
    cards.append(c);
  });
  box.append(cards);

  box.append(el("h2", { style: "margin-top:22px", text: "How the model is built" }));
  const how = el("div", { class: "how" });
  how.innerHTML = `
    <p><strong>Chain.</strong> A serial chain of six revolute (hinge) joints. Each joint sits at a fixed offset from the previous one along the link,
    then rotates about its own axis. Dimensions now: table to shoulder <b>${g.D1.toFixed(0)}</b> mm, upper arm <b>${g.L2.toFixed(0)}</b> mm,
    forearm <b>${g.L3.toFixed(0)}</b> mm (the forearm roll joint is <b>${g.L3A.toFixed(0)}</b> mm from the elbow), wrist centre to tool tip <b>${g.D6.toFixed(0)}</b> mm.</p>
    <p><strong>Spherical wrist.</strong> The J4, J5 and J6 axes meet at one point, the wrist centre (J5 and J6 are drawn at the same spot, J6 as a ring).
    So J1 to J3 alone decide where the wrist is, and J4 to J6 alone decide how the hand is oriented. The inverse kinematics is solved
    in closed form (up to eight solutions; the one nearest the current pose is used).</p>
    <p><strong>Gripper.</strong> The fingers are a seventh actuator, not an axis of the arm. They open and close along the hand's x axis.</p>
    <p><strong>Forward kinematics.</strong> Joint after joint, multiply a translation by the offset and a rotation about the axis. The sliders above do exactly this in the page.</p>
    <p><strong>Dynamics.</strong> Every part in the mass budget is a point mass on its link (${d.total_g.toFixed(0)} g in total including the ${d.payload_g} g object).
    Gravity torque at a joint is the sum, over all masses beyond it, of weight times the horizontal lever arm about that joint's axis. The physics simulation uses the same masses,
    the same axes and the reflected rotor inertia of each drive, and the two models are checked against each other on every run.</p>
    <p><strong>Colours.</strong> J1 blue, J2 red, J3 green, J4 purple, J5 orange, J6 brown, on this page.</p>
    <p class="hint">Link shapes in the simulation video are simple primitives; the model describes joints, masses and limits, not the CAD shape.</p>`;
  box.append(how);
  setPose(M.q.every((v) => v === 0) ? d.ready_deg : M.q);
}
function showDesign() {
  $("designView").hidden = false;
  $("modelView").hidden = true;
  $("navDesign").classList.add("active");
  $("navModel").classList.remove("active");
}
function route() { (location.hash === "#model" ? showModel : showDesign)(); }
window.addEventListener("hashchange", route);
