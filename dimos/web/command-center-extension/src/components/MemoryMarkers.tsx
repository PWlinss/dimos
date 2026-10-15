import * as React from "react";

import { MemoryLocation, MemoryRecord } from "../types";

interface Props {
  records: MemoryRecord[];
  locations: MemoryLocation[];
  worldToPx: (x: number, y: number) => [number, number];
}

export default function MemoryMarkers({ records, locations, worldToPx }: Props): React.ReactElement {
  return (
    <g className="memory-observation-markers" pointerEvents="none" aria-label="Memory observation positions">
      {records.filter((record) => record.coordinate_frame === "odom").map((record) => {
        const [x, y] = worldToPx(record.position.x, record.position.y);
        return <g key={`record-${record.id}`} transform={`translate(${x}, ${y})`}>
          <title>{`记忆观察位置：${record.label || record.id}；不是物体 3D 位置`}</title>
          <circle r={8} fill="#35c6d5" fillOpacity={0.2} stroke="#35c6d5" strokeWidth={2} />
        </g>;
      })}
      {locations.filter((location) => location.coordinate_frame === "odom").map((location) => {
        const [x, y] = worldToPx(location.position.x, location.position.y);
        return <g key={`location-${location.id}`} transform={`translate(${x}, ${y})`}>
          <title>{`命名地点观察位置：${location.name}；不是物体 3D 位置`}</title>
          <path d="M 0 -9 L 8 0 L 0 9 L -8 0 Z" fill="#f1b94e" stroke="#171b20" strokeWidth={1.5} />
        </g>;
      })}
    </g>
  );
}
