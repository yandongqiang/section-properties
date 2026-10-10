import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import type { SceneDto, SelectionState, DisplayState, NodeDto, MemberDto } from "./types";

const NODE_RADIUS = 0.08;
const MEMBER_RADIUS = 0.03;
const AXIS_LENGTH = 1.0;

export class SceneManager {
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;
  private controls: OrbitControls;
  private raycaster: THREE.Raycaster;

  private nodeGroup: THREE.Group;
  private memberGroup: THREE.Group;
  private undeformedGroup: THREE.Group;
  private localAxesGroup: THREE.Group;
  private gridHelper: THREE.GridHelper | null = null;
  private axesHelper: THREE.AxesHelper | null = null;

  private nodeMeshes: THREE.Mesh[] = [];
  private memberMeshes: THREE.Mesh[] = [];
  private undeformedLines: THREE.LineSegments | null = null;

  private sceneData: SceneDto | null = null;
  private display: DisplayState = {
    deformationScale: 0,
    showGrid: true,
    showAxes: true,
    showLocalAxes: false,
    showDeformed: true,
    showUndeformed: false,
  };
  private selection: SelectionState = { type: "none" };

  private nodeMaterial: THREE.MeshPhongMaterial;
  private nodeSelectedMaterial: THREE.MeshPhongMaterial;
  private memberMaterial: THREE.MeshPhongMaterial;
  private memberSelectedMaterial: THREE.MeshPhongMaterial;

  private onSelectionChange: (sel: SelectionState) => void;

  constructor(
    canvas: HTMLCanvasElement,
    onSelectionChange: (sel: SelectionState) => void,
  ) {
    this.onSelectionChange = onSelectionChange;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1a1a2e);

    const w = canvas.clientWidth || 800;
    const h = canvas.clientHeight || 600;

    this.camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 1000);
    this.camera.position.set(8, 6, 8);

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setSize(w, h);
    this.renderer.setPixelRatio(window.devicePixelRatio);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(2, 1.5, 2);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;

    this.raycaster = new THREE.Raycaster();

    const ambient = new THREE.AmbientLight(0x404040, 1.5);
    this.scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(10, 15, 5);
    this.scene.add(dir);

    this.nodeGroup = new THREE.Group();
    this.memberGroup = new THREE.Group();
    this.undeformedGroup = new THREE.Group();
    this.localAxesGroup = new THREE.Group();
    this.scene.add(this.nodeGroup, this.memberGroup, this.undeformedGroup, this.localAxesGroup);

    this.nodeMaterial = new THREE.MeshPhongMaterial({ color: 0x4ec9b0 });
    this.nodeSelectedMaterial = new THREE.MeshPhongMaterial({ color: 0xff6b6b });
    this.memberMaterial = new THREE.MeshPhongMaterial({ color: 0x569cd6 });
    this.memberSelectedMaterial = new THREE.MeshPhongMaterial({ color: 0xff6b6b });

    this.createGridAndAxes();

    canvas.addEventListener("click", (e) => this.handleClick(e, canvas));

    this.animate();
  }

  private createGridAndAxes(): void {
    this.gridHelper = new THREE.GridHelper(20, 20, 0x444466, 0x222244);
    this.axesHelper = new THREE.AxesHelper(2);
    this.scene.add(this.gridHelper, this.axesHelper);
  }

  loadScene(data: SceneDto): void {
    this.sceneData = data;
    this.selection = { type: "none" };
    this.clearGroup(this.nodeGroup);
    this.clearGroup(this.memberGroup);
    this.clearGroup(this.undeformedGroup);
    this.clearGroup(this.localAxesGroup);
    this.nodeMeshes = [];
    this.memberMeshes = [];
    this.undeformedLines = null;

    this.createNodes(data);
    this.createMembers(data);
    this.createUndeformedWireframe(data);
    this.fitCamera(data);

    this.applyDisplay();
    this.updateSelectionVisuals();
  }

  private createNodes(data: SceneDto): void {
    const geo = new THREE.SphereGeometry(NODE_RADIUS, 16, 12);
    for (const node of data.nodes) {
      const mesh = new THREE.Mesh(geo, this.nodeMaterial.clone());
      mesh.userData = { type: "node", id: node.id };
      const p = this.deformedPosition(node);
      mesh.position.set(p.x, p.y, p.z);
      this.nodeGroup.add(mesh);
      this.nodeMeshes.push(mesh);
    }
  }

  private createMembers(data: SceneDto): void {
    for (const member of data.members) {
      const ni = data.nodes[member.node_i];
      const nj = data.nodes[member.node_j];
      const pi = this.deformedPosition(ni);
      const pj = this.deformedPosition(nj);
      const len = Math.hypot(pj.x - pi.x, pj.y - pi.y, pj.z - pi.z);
      if (len < 1e-10) continue;

      const geo = new THREE.CylinderGeometry(MEMBER_RADIUS, MEMBER_RADIUS, len, 8);
      const mesh = new THREE.Mesh(geo, this.memberMaterial.clone());
      mesh.userData = { type: "member", id: member.id };
      this.positionCylinder(mesh, pi, pj);
      this.memberGroup.add(mesh);
      this.memberMeshes.push(mesh);
    }
  }

  private createUndeformedWireframe(data: SceneDto): void {
    const positions: number[] = [];
    for (const member of data.members) {
      const pi = data.nodes[member.node_i].position;
      const pj = data.nodes[member.node_j].position;
      positions.push(pi.x, pi.y, pi.z, pj.x, pj.y, pj.z);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    const mat = new THREE.LineBasicMaterial({ color: 0x666666, transparent: true, opacity: 0.4 });
    this.undeformedLines = new THREE.LineSegments(geo, mat);
    this.undeformedGroup.add(this.undeformedLines);
  }

  private positionCylinder(mesh: THREE.Mesh, pi: THREE.Vector3, pj: THREE.Vector3): void {
    const mid = new THREE.Vector3().addVectors(pi, pj).multiplyScalar(0.5);
    mesh.position.copy(mid);
    const dir = new THREE.Vector3().subVectors(pj, pi).normalize();
    const quat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    mesh.quaternion.copy(quat);
  }

  private deformedPosition(node: NodeDto): THREE.Vector3 {
    const s = this.display.showDeformed ? this.display.deformationScale : 0;
    return new THREE.Vector3(
      node.position.x + s * node.displacement.x,
      node.position.y + s * node.displacement.y,
      node.position.z + s * node.displacement.z,
    );
  }

  private fitCamera(data: SceneDto): void {
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (const n of data.nodes) {
      const p = n.position;
      minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
      minZ = Math.min(minZ, p.z); maxZ = Math.max(maxZ, p.z);
    }
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    const cz = (minZ + maxZ) / 2;
    const range = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 1);
    this.controls.target.set(cx, cy, cz);
    this.camera.position.set(cx + range, cy + range * 0.7, cz + range);
    this.camera.near = range * 0.01;
    this.camera.far = range * 100;
    this.camera.updateProjectionMatrix();
  }

  setDeformationScale(scale: number): void {
    this.display.deformationScale = scale;
    if (this.sceneData) this.updateDeformedGeometry();
  }

  setDisplayState(display: Partial<DisplayState>): void {
    this.display = { ...this.display, ...display };
    this.applyDisplay();
    if (this.sceneData) this.updateDeformedGeometry();
  }

  private updateDeformedGeometry(): void {
    if (!this.sceneData) return;
    const data = this.sceneData;
    for (let i = 0; i < this.nodeMeshes.length && i < data.nodes.length; i++) {
      const p = this.deformedPosition(data.nodes[i]);
      this.nodeMeshes[i].position.set(p.x, p.y, p.z);
    }
    for (let i = 0; i < this.memberMeshes.length && i < data.members.length; i++) {
      const m = data.members[i];
      const pi = this.deformedPosition(data.nodes[m.node_i]);
      const pj = this.deformedPosition(data.nodes[m.node_j]);
      this.positionCylinder(this.memberMeshes[i], pi, pj);
    }
    this.updateLocalAxes();
  }

  private applyDisplay(): void {
    if (this.gridHelper) this.gridHelper.visible = this.display.showGrid;
    if (this.axesHelper) this.axesHelper.visible = this.display.showAxes;
    if (this.undeformedLines) this.undeformedLines.visible = this.display.showUndeformed;
    this.updateLocalAxes();
  }

  private updateLocalAxes(): void {
    this.clearGroup(this.localAxesGroup);
    if (!this.display.showLocalAxes || !this.sceneData) return;
    const sel = this.selection;
    if (sel.type !== "member") return;

    const data = this.sceneData;
    const member = data.members.find((m) => m.id === sel.id);
    if (!member) return;

    const ni = data.nodes[member.node_i];
    const nj = data.nodes[member.node_j];
    const pi = this.deformedPosition(ni);
    const pj = this.deformedPosition(nj);
    const mid = new THREE.Vector3().addVectors(pi, pj).multiplyScalar(0.5);

    const axes = [
      { v: member.local_x_axis, color: 0xff4444 },
      { v: member.local_y_axis, color: 0x44ff44 },
      { v: member.local_z_axis, color: 0x4477ff },
    ];
    for (const { v, color } of axes) {
      const dir = new THREE.Vector3(v.x, v.y, v.z);
      const arrow = new THREE.ArrowHelper(dir, mid, AXIS_LENGTH, color, 0.1, 0.06);
      this.localAxesGroup.add(arrow);
    }
  }

  private clearGroup(group: THREE.Group): void {
    for (const child of group.children) {
      if (child instanceof THREE.Mesh) {
        child.geometry.dispose();
        if (child.material instanceof THREE.Material) child.material.dispose();
      }
      if (child instanceof THREE.LineSegments) {
        child.geometry.dispose();
        if (child.material instanceof THREE.Material) child.material.dispose();
      }
      if (child instanceof THREE.ArrowHelper) {
        child.dispose();
      }
    }
    group.clear();
  }

  private handleClick(e: MouseEvent, canvas: HTMLCanvasElement): void {
    if (!this.sceneData) return;
    const rect = canvas.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    this.raycaster.setFromCamera(new THREE.Vector2(x, y), this.camera);

    const nodeHits = this.raycaster.intersectObjects(this.nodeMeshes, false);
    const memberHits = this.raycaster.intersectObjects(this.memberMeshes, false);

    const nodeDist = nodeHits.length > 0 ? nodeHits[0].distance : Infinity;
    const memberDist = memberHits.length > 0 ? memberHits[0].distance : Infinity;

    if (nodeDist < memberDist && nodeHits.length > 0) {
      const id = nodeHits[0].object.userData.id as number;
      this.setSelection({ type: "node", id });
    } else if (memberHits.length > 0) {
      const id = memberHits[0].object.userData.id as number;
      this.setSelection({ type: "member", id });
    } else {
      this.setSelection({ type: "none" });
    }
  }

  private setSelection(sel: SelectionState): void {
    this.selection = sel;
    this.updateSelectionVisuals();
    this.updateLocalAxes();
    this.onSelectionChange(sel);
  }

  private updateSelectionVisuals(): void {
    const sel = this.selection;
    for (const mesh of this.nodeMeshes) {
      const id = mesh.userData.id as number;
      const isSel = sel.type === "node" && sel.id === id;
      mesh.material = isSel ? this.nodeSelectedMaterial : this.nodeMaterial;
    }
    for (const mesh of this.memberMeshes) {
      const id = mesh.userData.id as number;
      const isSel = sel.type === "member" && sel.id === id;
      mesh.material = isSel ? this.memberSelectedMaterial : this.memberMaterial;
    }
  }

  getSelection(): SelectionState {
    return this.selection;
  }

  getSceneData(): SceneDto | null {
    return this.sceneData;
  }

  resize(): void {
    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w === 0 || h === 0) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  private animate = (): void => {
    requestAnimationFrame(this.animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  dispose(): void {
    this.clearGroup(this.nodeGroup);
    this.clearGroup(this.memberGroup);
    this.clearGroup(this.undeformedGroup);
    this.clearGroup(this.localAxesGroup);
    if (this.gridHelper) {
      this.gridHelper.dispose();
      this.scene.remove(this.gridHelper);
    }
    if (this.axesHelper) {
      this.axesHelper.dispose();
      this.scene.remove(this.axesHelper);
    }
    this.renderer.dispose();
  }
}
