from dataclasses import dataclass, field

from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos


class FakeROSMessage:
    def get_fields_and_field_types(self) -> dict[str, str]:
        return {
            key: ""
            for key in vars(self)
            if not key.startswith("_")
        }


@dataclass
class FakeStamp(FakeROSMessage):
    sec: int = 0
    nanosec: int = 0


@dataclass
class FakeHeader(FakeROSMessage):
    stamp: FakeStamp = field(default_factory=FakeStamp)
    frame_id: str = ""


@dataclass
class FakePosition(FakeROSMessage):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class FakeOrientation(FakeROSMessage):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class FakePose(FakeROSMessage):
    position: FakePosition = field(default_factory=FakePosition)
    orientation: FakeOrientation = field(default_factory=FakeOrientation)


@dataclass
class FakeMapInfo(FakeROSMessage):
    resolution: float = 0.05
    width: int = 0
    height: int = 0
    origin: FakePose = field(default_factory=FakePose)


@dataclass
class FakeOccupancyGrid(FakeROSMessage):
    header: FakeHeader = field(default_factory=FakeHeader)
    info: FakeMapInfo = field(default_factory=FakeMapInfo)
    data: list[int] = field(default_factory=list)


def test_occupancy_grid_ros_to_dimos_preserves_grid_data() -> None:
    ros_grid = FakeOccupancyGrid()
    ros_grid.header.frame_id = "map"
    ros_grid.header.stamp.sec = 123
    ros_grid.header.stamp.nanosec = 456_000_000
    ros_grid.info.resolution = 0.05
    ros_grid.info.width = 3
    ros_grid.info.height = 2
    ros_grid.info.origin.position.x = -1.0
    ros_grid.info.origin.position.y = 2.0
    ros_grid.data = [0, 100, -1, 42, 0, 100]

    converted = ros_to_dimos(ros_grid, OccupancyGrid)

    assert converted.frame_id == "map"
    assert converted.width == 3
    assert converted.height == 2
    assert converted.resolution == 0.05
    assert converted.origin.position.x == -1.0
    assert converted.origin.position.y == 2.0
    assert converted.grid.tolist() == [[0, 100, -1], [42, 0, 100]]
    assert abs(converted.ts - 123.456) < 1e-6
