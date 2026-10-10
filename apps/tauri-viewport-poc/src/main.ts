import { invoke } from "@tauri-apps/api/core";
import "./style.css";
import { SceneManager } from "./scene-manager";
import type { SceneDto, SelectionState, DisplayState, NodeDto, MemberDto } from "./types";

type ModelKey = "portal" | "inclined" | "grid2" | "grid5" | "grid10";

const MODELS: { key: ModelKey; label: string }[] = [
  { key: "portal", label: "Portal Frame (4 nodes, 3 members)" },
  { key: "inclined", label: "Inclined Frame (5 nodes, 4 members)" },
  { key: "grid2", label: "Grid 2×2" },
  { key: "grid5", label: "Grid 5×5" },
  { key: "grid10", label: "Grid 10×10" },
];

let currentModel: ModelKey = "portal";
let display: DisplayState = {
  deformationScale: 0,
  showGrid: true,
  showAxes: true,
  showLocalAxes: false,
  showDeformed: true,
  showUndeformed: false,
};

let sm: SceneManager;

async function loadModel(key: ModelKey): Promise<SceneDto> {
  switch (key) {
    case "portal":
      return invoke<SceneDto>("get_scene");
    case "inclined":
      return invoke<SceneDto>("get_scene_inclined");
    case "grid2":
      return invoke<SceneDto>("get_scene_grid", { nx: 2, nz: 2 });
    case "grid5":
      return invoke<SceneDto>("get_scene_grid", { nx: 5, nz: 5 });
    case "grid10":
      return invoke<SceneDto>("get_scene_grid", { nx: 10, nz: 10 });
  }
}

function fmt(v: number, digits = 4): string {
  if (Math.abs(v) < 1e-12) return "0";
  return v.toExponential(digits);
}

function fmtVec(x: number, y: number, z: number, digits = 4): string {
  return `[${fmt(x, digits)}, ${fmt(y, digits)}, ${fmt(z, digits)}]`;
}

function buildPanel(): HTMLElement {
  const panel = document.getElementById("panel")!;

  let html = "";

  html += `<div class="section">
    <h2>Model</h2>
    <select id="model-select">
      ${MODELS.map((m) => `<option value="${m.key}"${m.key === currentModel ? " selected" : ""}>${m.label}</option>`).join("")}
    </select>
  </div>`;

  html += `<div class="section">
    <h2>Display</h2>
    <label>Deformation Scale: <span id="scale-value" class="value">0</span></label>
    <input type="range" id="scale-slider" min="0" max="10000" value="0" step="1">
    ${checkbox("show-undeformed", "Show undeformed wireframe", display.showUndeformed)}
    ${checkbox("show-local-axes", "Show local axes (selected member)", display.showLocalAxes)}
    ${checkbox("show-grid", "Show grid", display.showGrid)}
    ${checkbox("show-axes", "Show global axes", display.showAxes)}
  </div>`;

  html += `<div class="section" id="selection-section">
    <h2>Selection</h2>
    <div class="info">Click a node or member in the viewport.</div>
  </div>`;

  panel.innerHTML = html;
  return panel;
}

function checkbox(id: string, label: string, checked: boolean): string {
  return `<div class="checkbox-row">
    <input type="checkbox" id="${id}"${checked ? " checked" : ""}>
    <label for="${id}">${label}</label>
  </div>`;
}

function wirePanelEvents(sm: SceneManager): void {
  const modelSelect = document.getElementById("model-select") as HTMLSelectElement;
  modelSelect.addEventListener("change", async () => {
    currentModel = modelSelect.value as ModelKey;
    setStatus("Loading…");
    try {
      const scene = await loadModel(currentModel);
      sm.loadScene(scene);
      setStatus(`${scene.n_nodes} nodes, ${scene.n_members} members`);
    } catch (e) {
      setStatus("Error");
      showError(String(e));
    }
  });

  const slider = document.getElementById("scale-slider") as HTMLInputElement;
  const scaleValue = document.getElementById("scale-value")!;
  slider.addEventListener("input", () => {
    const v = parseInt(slider.value, 10);
    display.deformationScale = v;
    scaleValue.textContent = String(v);
    sm.setDeformationScale(v);
  });

  wireCheckbox("show-undeformed", "showUndeformed", sm);
  wireCheckbox("show-local-axes", "showLocalAxes", sm);
  wireCheckbox("show-grid", "showGrid", sm);
  wireCheckbox("show-axes", "showAxes", sm);
}

function wireCheckbox(id: string, key: keyof DisplayState, sm: SceneManager): void {
  const cb = document.getElementById(id) as HTMLInputElement;
  cb.addEventListener("change", () => {
    display[key] = cb.checked as never;
    sm.setDisplayState({ [key]: cb.checked } as Partial<DisplayState>);
  });
}

function updateSelectionPanel(sel: SelectionState): void {
  const section = document.getElementById("selection-section")!;
  const data = sm.getSceneData();
  if (!data) return;

  if (sel.type === "none") {
    section.innerHTML = `<h2>Selection</h2><div class="info">Click a node or member in the viewport.</div>`;
    return;
  }

  if (sel.type === "node") {
    const node = data.nodes.find((n) => n.id === sel.id);
    if (node) section.innerHTML = nodeHtml(node);
  } else {
    const member = data.members.find((m) => m.id === sel.id);
    if (member) section.innerHTML = memberHtml(member, data);
  }
}

function propRow(label: string, value: string): string {
  return `<div class="property"><span class="lbl">${label}</span><span class="val">${value}</span></div>`;
}

function nodeHtml(n: NodeDto): string {
  let html = `<h2>Node ${n.id}</h2>`;
  html += propRow("Position", fmtVec(n.position.x, n.position.y, n.position.z));
  html += propRow("Displacement", fmtVec(n.displacement.x, n.displacement.y, n.displacement.z));
  html += propRow("Rotation", fmtVec(n.rotation.x, n.rotation.y, n.rotation.z));
  html += propRow("Reaction F", fmtVec(n.reaction_force.x, n.reaction_force.y, n.reaction_force.z));
  html += propRow("Reaction M", fmtVec(n.reaction_moment.x, n.reaction_moment.y, n.reaction_moment.z));
  return html;
}

function memberHtml(m: MemberDto, data: SceneDto): string {
  const ni = data.nodes[m.node_i];
  const nj = data.nodes[m.node_j];
  const dx = nj.position.x - ni.position.x;
  const dy = nj.position.y - ni.position.y;
  const dz = nj.position.z - ni.position.z;
  const len = Math.hypot(dx, dy, dz);

  let html = `<h2>Member ${m.id}</h2>`;
  html += propRow("Nodes", `${m.node_i} → ${m.node_j}`);
  html += propRow("Length", fmt(len, 4));
  html += propRow("Ref vec", fmtVec(m.ref_vec.x, m.ref_vec.y, m.ref_vec.z));
  html += propRow("Local x̂", fmtVec(m.local_x_axis.x, m.local_x_axis.y, m.local_x_axis.z));
  html += propRow("Local ŷ", fmtVec(m.local_y_axis.x, m.local_y_axis.y, m.local_y_axis.z));
  html += propRow("Local ẑ", fmtVec(m.local_z_axis.x, m.local_z_axis.y, m.local_z_axis.z));

  html += `<h2 style="margin-top:12px">End Forces (Local)</h2>`;
  const labels = ["Fx_i", "Fy_i", "Fz_i", "Mx_i", "My_i", "Mz_i", "Fx_j", "Fy_j", "Fz_j", "Mx_j", "My_j", "Mz_j"];
  for (let i = 0; i < 12; i++) {
    html += propRow(labels[i], fmt(m.end_forces_local[i], 3));
  }

  html += `<h2 style="margin-top:12px">End Forces (Global)</h2>`;
  for (let i = 0; i < 12; i++) {
    html += propRow(labels[i], fmt(m.end_forces_global[i], 3));
  }

  return html;
}

function setStatus(msg: string): void {
  const el = document.getElementById("status");
  if (el) el.textContent = msg;
}

function showError(msg: string): void {
  const section = document.getElementById("selection-section")!;
  section.innerHTML = `<h2>Error</h2><div class="error">${msg}</div>`;
}

async function main(): Promise<void> {
  const canvas = document.getElementById("canvas") as HTMLCanvasElement;
  sm = new SceneManager(canvas, (sel: SelectionState) => updateSelectionPanel(sel));

  buildPanel();
  wirePanelEvents(sm);

  window.addEventListener("resize", () => sm.resize());

  try {
    const scene = await loadModel(currentModel);
    sm.loadScene(scene);
    setStatus(`${scene.n_nodes} nodes, ${scene.n_members} members`);
  } catch (e) {
    setStatus("Error");
    showError(String(e));
  }
}

main();
