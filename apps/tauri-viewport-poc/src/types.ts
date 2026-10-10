export interface Vec3Dto {
  x: number;
  y: number;
  z: number;
}

export interface NodeDto {
  id: number;
  position: Vec3Dto;
  displacement: Vec3Dto;
  rotation: Vec3Dto;
  reaction_force: Vec3Dto;
  reaction_moment: Vec3Dto;
}

export interface MemberDto {
  id: number;
  node_i: number;
  node_j: number;
  ref_vec: Vec3Dto;
  local_x_axis: Vec3Dto;
  local_y_axis: Vec3Dto;
  local_z_axis: Vec3Dto;
  end_forces_local: number[];
  end_forces_global: number[];
}

export interface SceneDto {
  nodes: NodeDto[];
  members: MemberDto[];
  n_nodes: number;
  n_members: number;
}

export type SelectionState =
  | { type: "none" }
  | { type: "node"; id: number }
  | { type: "member"; id: number };

export interface DisplayState {
  deformationScale: number;
  showGrid: boolean;
  showAxes: boolean;
  showLocalAxes: boolean;
  showDeformed: boolean;
  showUndeformed: boolean;
}
