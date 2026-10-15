import * as React from "react";

import Connection from "./Connection";
import ExplorePanel from "./ExplorePanel";
import GpsButton from "./GpsButton";
import Button from "./Button";
import KeyboardControlPanel from "./KeyboardControlPanel";
import VisualizerWrapper from "./components/VisualizerWrapper";
import LeafletMap from "./components/LeafletMap";
import AgentMemoryPanel from "./AgentMemoryPanel";
import { AppAction, AppState, LatLon, MemoryRecord } from "./types";

function appReducer(state: AppState, action: AppAction): AppState {
  switch (action.type) {
    case "SET_COSTMAP":
      return { ...state, costmap: action.payload };
    case "SET_ROBOT_POSE":
      return { ...state, robotPose: action.payload };
    case "SET_GPS_LOCATION":
      return { ...state, gpsLocation: action.payload };
    case "SET_GPS_TRAVEL_GOAL_POINTS":
      return { ...state, gpsTravelGoalPoints: action.payload };
    case "SET_PATH":
      return { ...state, path: action.payload };
    case "SET_CAMERA_FRAME":
      return { ...state, cameraFrame: action.payload };
    case "SET_AGENT_MESSAGE":
      return state.agentMessages.some((message) => message.id === action.payload.id)
        ? state
        : { ...state, agentMessages: [...state.agentMessages, action.payload].slice(-200) };
    case "SET_AGENT_IDLE":
      return { ...state, agentIdle: action.payload };
    case "RESET_AGENT_CONNECTION":
      return action.payload
        ? { ...state, socketConnected: true }
        : {
            ...state,
            socketConnected: false,
            agentAvailable: null,
            memoryAvailable: null,
            agentStateReady: false,
            agentIdle: null,
          };
    case "SET_MEMORY":
      return {
        ...state,
        memoryRecords: action.payload.records,
        memoryLocations: action.payload.locations,
        memoryStats: action.payload.stats,
      };
    case "SET_MEMORY_RECORDS":
      return { ...state, memoryRecords: action.payload };
    case "SET_FULL_STATE":
    {
      const messages = action.payload.agentMessages
        ? [...new Map([...state.agentMessages, ...action.payload.agentMessages].map((message) => [message.id, message])).values()].slice(-200)
        : state.agentMessages;
      const agentAvailable = action.payload.agentAvailable;
      const memoryAvailable = action.payload.memoryAvailable;
      return {
        ...state,
        ...action.payload,
        agentMessages: messages,
        agentStateReady: typeof agentAvailable === "boolean" && action.payload.agentIdle !== undefined,
        agentPanelSeen: state.agentPanelSeen || agentAvailable === true || memoryAvailable === true,
        agentPanelAgentVisible: agentAvailable ?? state.agentPanelAgentVisible,
        agentPanelMemoryVisible: memoryAvailable ?? state.agentPanelMemoryVisible,
      };
    }
    default:
      return state;
  }
}

const initialState: AppState = {
  costmap: null,
  robotPose: null,
  gpsLocation: null,
  gpsTravelGoalPoints: null,
  path: null,
  cameraFrame: null,
  socketConnected: false,
  agentAvailable: null,
  agentStateReady: false,
  memoryAvailable: null,
  agentPanelSeen: false,
  agentPanelAgentVisible: false,
  agentPanelMemoryVisible: false,
  agentIdle: null,
  agentMessages: [],
  memoryRecords: [],
  memoryLocations: [],
  memoryStats: {},
};

export default function App(): React.ReactElement {
  const [state, dispatch] = React.useReducer(appReducer, initialState);
  const [isGpsMode, setIsGpsMode] = React.useState(false);
  const connectionRef = React.useRef<Connection | null>(null);

  React.useEffect(() => {
    connectionRef.current = new Connection(dispatch);

    return () => {
      if (connectionRef.current) {
        connectionRef.current.disconnect();
      }
    };
  }, []);

  const handleWorldClick = React.useCallback((worldX: number, worldY: number) => {
    connectionRef.current?.worldClick(worldX, worldY);
  }, []);

  const handleStartExplore = React.useCallback(() => {
    connectionRef.current?.startExplore();
  }, []);

  const handleStopExplore = React.useCallback(() => {
    connectionRef.current?.stopExplore();
  }, []);

  const handleGpsGoal = React.useCallback((goal: LatLon) => {
    connectionRef.current?.sendGpsGoal(goal);
  }, []);

  const handleSendMoveCommand = React.useCallback(
    (linear: [number, number, number], angular: [number, number, number]) => {
      connectionRef.current?.sendMoveCommand(linear, angular);
    },
    [],
  );

  const handleStopMoveCommand = React.useCallback(() => {
    connectionRef.current?.stopMoveCommand();
  }, []);

  const handleReturnHome = React.useCallback(() => {
    connectionRef.current?.worldClick(0, 0);
  }, []);

  const handleStop = React.useCallback(() => {
    if (state.robotPose) {
      connectionRef.current?.worldClick(state.robotPose.coords[0]!, state.robotPose.coords[1]!);
    }
  }, [state.robotPose]);

  const handleAgentSend = React.useCallback((text: string) => connectionRef.current!.sendAgentMessage(text), []);
  const handleMemoryRefresh = React.useCallback(async () => {
    const snapshot = await connectionRef.current!.fetchMemory();
    dispatch({ type: "SET_MEMORY", payload: snapshot });
  }, []);
  const handleMemorySearch = React.useCallback(async (query: string): Promise<MemoryRecord[]> => {
    const records = await connectionRef.current!.searchMemory(query);
    dispatch({ type: "SET_MEMORY_RECORDS", payload: records.slice(0, 3) });
    return records.slice(0, 3);
  }, []);

  const showAgentPanel = state.agentPanelSeen;

  return (
    <div style={{ position: "relative", width: "100%", height: "100%", display: "flex", flexDirection: "column", overflow: "hidden" }}>
      <div style={{ position: "relative", flex: "1 1 auto", minHeight: showAgentPanel ? 220 : 0 }}>
        {isGpsMode ? (
          <LeafletMap
            gpsLocation={state.gpsLocation}
            gpsTravelGoalPoints={state.gpsTravelGoalPoints}
            onGpsGoal={handleGpsGoal}
          />
        ) : (
          <VisualizerWrapper data={state} onWorldClick={handleWorldClick} />
        )}
        <aside
          aria-label="Go2 live camera"
          style={{
            position: "absolute",
            top: 12,
            right: 12,
            width: "min(34vw, 320px)",
            aspectRatio: "16 / 9",
            overflow: "hidden",
            border: "1px solid #687987",
            borderRadius: 6,
            background: "#10161c",
            boxShadow: "0 2px 12px #0008",
            pointerEvents: "none",
            zIndex: 500,
          }}
        >
          {state.cameraFrame ? (
            <img
              src={state.cameraFrame.src}
              alt="Live Go2 camera"
              style={{ width: "100%", height: "100%", objectFit: "cover", display: "block" }}
            />
          ) : (
            <span style={{ color: "#d4dde5", font: "12px system-ui", padding: 8, display: "block" }}>
              Waiting for camera…
            </span>
          )}
        </aside>
      <div
        style={{
          position: "absolute",
          bottom: 0,
          left: 0,
          display: "flex",
          width: "100%",
          padding: 5,
          gap: 5,
          alignItems: "flex-end",
        }}
      >
        <GpsButton
          onUseGps={() => setIsGpsMode(true)}
          onUseCostmap={() => setIsGpsMode(false)}
        ></GpsButton>
        <ExplorePanel onStartExplore={handleStartExplore} onStopExplore={handleStopExplore} />
        <Button onClick={handleReturnHome} isActive={false}>Go Home</Button>
        <Button onClick={handleStop} isActive={false}>Stop</Button>
        <KeyboardControlPanel
          onSendMoveCommand={handleSendMoveCommand}
          onStopMoveCommand={handleStopMoveCommand}
        />
      </div>
      </div>
      {showAgentPanel && (
        <AgentMemoryPanel
          connected={state.socketConnected}
          agentVisible={state.agentPanelAgentVisible}
          memoryVisible={state.agentPanelMemoryVisible}
          agentEnabled={state.agentStateReady && state.agentAvailable === true}
          memoryEnabled={state.memoryAvailable === true}
          agentIdle={state.agentIdle}
          messages={state.agentMessages}
          records={state.memoryRecords}
          locations={state.memoryLocations}
          stats={state.memoryStats}
          onSendMessage={handleAgentSend}
          onRefreshMemory={handleMemoryRefresh}
          onSearchMemory={handleMemorySearch}
        />
      )}
    </div>
  );
}
