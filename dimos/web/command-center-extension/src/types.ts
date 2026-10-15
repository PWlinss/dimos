import { EncodedOptimizedGrid, OptimizedGrid } from './optimizedCostmap';

export type EncodedVector = Encoded<"vector"> & {
  c: number[];
};

export class Vector {
  coords: number[];
  constructor(...coords: number[]) {
    this.coords = coords;
  }

  static decode(data: EncodedVector): Vector {
    return new Vector(...data.c);
  }
}

export interface LatLon {
  lat: number;
  lon: number;
  alt?: number;
}

export type EncodedPath = Encoded<"path"> & {
  points: Array<[number, number]>;
};

export class Path {
  constructor(public coords: Array<[number, number]>) {}

  static decode(data: EncodedPath): Path {
    return new Path(data.points);
  }
}

export type EncodedCostmap = Encoded<"costmap"> & {
  grid: EncodedOptimizedGrid;
  origin: EncodedVector;
  resolution: number;
  origin_theta: number;
};

export class Costmap {
  constructor(
    public grid: Grid,
    public origin: Vector,
    public resolution: number,
    public origin_theta: number,
  ) {
    this.grid = grid;
    this.origin = origin;
    this.resolution = resolution;
    this.origin_theta = origin_theta;
  }

  private static decoder: OptimizedGrid | null = null;

  static decode(data: EncodedCostmap): Costmap {
    // Use a singleton decoder to maintain state for delta updates
    if (!Costmap.decoder) {
      Costmap.decoder = new OptimizedGrid();
    }

    const float32Data = Costmap.decoder.decode(data.grid);
    const shape = data.grid.shape;

    // Create a Grid object from the decoded data
    const grid = new Grid(float32Data, shape);

    return new Costmap(
      grid,
      Vector.decode(data.origin),
      data.resolution,
      data.origin_theta,
    );
  }
}

export class Grid {
  constructor(
    public data: Float32Array | Float64Array | Int32Array | Int8Array,
    public shape: number[],
  ) {}
}

export type Drawable = Costmap | Vector | Path;

export type Encoded<T extends string> = {
  type: T;
};

export interface FullStateData {
  costmap?: EncodedCostmap;
  robot_pose?: EncodedVector;
  gps_location?: LatLon;
  gps_travel_goal_points?: LatLon[];
  path?: EncodedPath;
  agent_available?: boolean;
  memory_available?: boolean;
  agent_idle?: boolean | null;
  agent_messages?: ChatMessage[];
  camera_frame?: CameraFrame;
}

export interface CameraFrame {
  src: string;
  ts: number;
}

export interface ChatToolCall {
  id: string;
  name: string;
  args: unknown;
}

export interface ChatMessage {
  id: string;
  role: "human" | "ai" | "tool";
  content: string;
  tool_calls: ChatToolCall[];
  tool_call_id?: string;
  name?: string;
  ts: number;
}

export interface MemoryPosition {
  x: number;
  y: number;
  z: number;
}

export interface MemoryRecord {
  id: string;
  timestamp: number;
  coordinate_frame: string;
  position: MemoryPosition;
  thumbnail: string | null;
  similarity: number | null;
  label: string | null;
}

export interface MemoryLocation {
  id: string;
  name: string;
  position: MemoryPosition;
  coordinate_frame: string;
  timestamp: number;
}

export interface MemorySnapshot {
  stats: Record<string, number>;
  records: MemoryRecord[];
  locations: MemoryLocation[];
}

export interface TwistCommand {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

export interface AppState {
  costmap: Costmap | null;
  robotPose: Vector | null;
  gpsLocation: LatLon | null;
  gpsTravelGoalPoints: LatLon[] | null;
  path: Path | null;
  cameraFrame: CameraFrame | null;
  socketConnected: boolean;
  agentAvailable: boolean | null;
  agentStateReady: boolean;
  memoryAvailable: boolean | null;
  agentPanelSeen: boolean;
  agentPanelAgentVisible: boolean;
  agentPanelMemoryVisible: boolean;
  agentIdle: boolean | null;
  agentMessages: ChatMessage[];
  memoryRecords: MemoryRecord[];
  memoryLocations: MemoryLocation[];
  memoryStats: Record<string, number>;
}

export type AppAction =
  | { type: "SET_COSTMAP"; payload: Costmap }
  | { type: "SET_ROBOT_POSE"; payload: Vector }
  | { type: "SET_GPS_LOCATION"; payload: LatLon }
  | { type: "SET_GPS_TRAVEL_GOAL_POINTS"; payload: LatLon[] }
  | { type: "SET_PATH"; payload: Path }
  | { type: "SET_CAMERA_FRAME"; payload: CameraFrame }
  | { type: "SET_AGENT_MESSAGE"; payload: ChatMessage }
  | { type: "SET_AGENT_IDLE"; payload: boolean }
  | { type: "RESET_AGENT_CONNECTION"; payload: boolean }
  | { type: "SET_MEMORY"; payload: MemorySnapshot }
  | { type: "SET_MEMORY_RECORDS"; payload: MemoryRecord[] }
  | { type: "SET_FULL_STATE"; payload: Partial<AppState> };
