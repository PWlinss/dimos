import { io, Socket } from "socket.io-client";

import {
  AppAction,
  ChatMessage,
  CameraFrame,
  Costmap,
  EncodedCostmap,
  EncodedPath,
  EncodedVector,
  FullStateData,
  LatLon,
  MemoryRecord,
  MemorySnapshot,
  Path,
  TwistCommand,
  Vector,
} from "./types";

export default class Connection {
  socket: Socket;
  dispatch: React.Dispatch<AppAction>;

  constructor(dispatch: React.Dispatch<AppAction>) {
    this.dispatch = dispatch;
    this.socket = io();

    this.socket.on("connect", () => {
      this.dispatch({ type: "RESET_AGENT_CONNECTION", payload: true });
    });
    this.socket.on("disconnect", () => {
      this.dispatch({ type: "RESET_AGENT_CONNECTION", payload: false });
    });

    this.socket.on("costmap", (data: EncodedCostmap) => {
      const costmap = Costmap.decode(data);
      this.dispatch({ type: "SET_COSTMAP", payload: costmap });
    });

    this.socket.on("robot_pose", (data: EncodedVector) => {
      const robotPose = Vector.decode(data);
      this.dispatch({ type: "SET_ROBOT_POSE", payload: robotPose });
    });

    this.socket.on("gps_location", (data: LatLon) => {
      this.dispatch({ type: "SET_GPS_LOCATION", payload: data });
    });

    this.socket.on("gps_travel_goal_points", (data: LatLon[]) => {
      this.dispatch({ type: "SET_GPS_TRAVEL_GOAL_POINTS", payload: data });
    });

    this.socket.on("path", (data: EncodedPath) => {
      const path = Path.decode(data);
      this.dispatch({ type: "SET_PATH", payload: path });
    });

    this.socket.on("camera_frame", (frame: CameraFrame) => {
      this.dispatch({ type: "SET_CAMERA_FRAME", payload: frame });
    });

    this.socket.on("full_state", (data: FullStateData) => {
      const state: Partial<{
        costmap: Costmap;
        robotPose: Vector;
        gpsLocation: LatLon;
        gpsTravelGoalPoints: LatLon[];
        path: Path;
        agentAvailable: boolean;
        memoryAvailable: boolean;
        agentIdle: boolean | null;
        agentMessages: ChatMessage[];
        cameraFrame: CameraFrame;
      }> = {};

      if (data.costmap != undefined) {
        state.costmap = Costmap.decode(data.costmap);
      }
      if (data.robot_pose != undefined) {
        state.robotPose = Vector.decode(data.robot_pose);
      }
      if (data.gps_location != undefined) {
        state.gpsLocation = data.gps_location;
      }
      if (data.path != undefined) {
        state.path = Path.decode(data.path);
      }
      if (data.agent_available != undefined) state.agentAvailable = data.agent_available;
      if (data.memory_available != undefined) state.memoryAvailable = data.memory_available;
      if (data.agent_idle !== undefined) state.agentIdle = data.agent_idle;
      if (data.agent_messages != undefined) state.agentMessages = data.agent_messages;
      if (data.camera_frame != undefined) state.cameraFrame = data.camera_frame;

      this.dispatch({ type: "SET_FULL_STATE", payload: state });
    });

    this.socket.on("agent_message", (message: ChatMessage) => {
      this.dispatch({ type: "SET_AGENT_MESSAGE", payload: message });
    });
    this.socket.on("agent_idle", (idle: boolean) => {
      this.dispatch({ type: "SET_AGENT_IDLE", payload: idle });
    });
  }

  worldClick(worldX: number, worldY: number): void {
    this.socket.emit("click", [worldX, worldY]);
  }

  startExplore(): void {
    this.socket.emit("start_explore");
  }

  stopExplore(): void {
    this.socket.emit("stop_explore");
  }

  sendMoveCommand(linear: [number, number, number], angular: [number, number, number]): void {
    const twist: TwistCommand = {
      linear: {
        x: linear[0],
        y: linear[1],
        z: linear[2],
      },
      angular: {
        x: angular[0],
        y: angular[1],
        z: angular[2],
      },
    };
    this.socket.emit("move_command", twist);
  }

  sendGpsGoal(goal: LatLon): void {
    this.socket.emit("gps_goal", goal);
  }

  sendAgentMessage(text: string): Promise<void> {
    return this.request("agent_send", { text });
  }

  fetchMemory(): Promise<MemorySnapshot> {
    return this.request("memory_snapshot", {});
  }

  searchMemory(query: string): Promise<MemoryRecord[]> {
    return this.request("memory_search", { query });
  }

  private request<T>(event: string, payload: unknown): Promise<T> {
    return new Promise((resolve, reject) => {
      if (!this.socket.connected) {
        reject(new Error("连接已断开，请重新连接后重试。"));
        return;
      }

      const timeout = window.setTimeout(() => reject(new Error("请求超时，请重试。")), 12000);
      this.socket.volatile.emit(event, payload, (response: { ok: boolean; data?: T; error?: string }) => {
        window.clearTimeout(timeout);
        if (!response?.ok) reject(new Error(response?.error || "请求失败。"));
        else resolve(response.data as T);
      });
    });
  }

  stopMoveCommand(): void {
    const twist: TwistCommand = {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    };
    this.socket.emit("move_command", twist);
  }

  disconnect(): void {
    this.socket.disconnect();
  }
}
