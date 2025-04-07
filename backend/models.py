from typing import Optional
from pydantic import BaseModel
from typing import  Dict, Any, Optional


class MsgPayload(BaseModel):
    msg_id: Optional[int]
    msg_name: str




class BatteryInfo(BaseModel):
    remaining: Optional[float] = None
    voltage: Optional[float] = None
    temperature: Optional[float] = None

class Position(BaseModel):
    x: float
    y: float
    z: float

class Attitude(BaseModel):
    roll: float
    pitch: float
    yaw: float

class GPSInfo(BaseModel):
    latitude: float
    longitude: float
    altitude: float

class Velocity(BaseModel):
    north: float
    east: float
    down: float

class DroneStatus(BaseModel):
    connected: bool = False
    armed: bool = False
    in_air: bool = False
    flight_mode: str = "unknown"



class DroneInfo(BaseModel):
    id: str
    name: str
    status: DroneStatus
    battery: Optional[BatteryInfo] = None
    position: Optional[Position] = None
    attitude: Optional[Attitude] = None
    gps: Optional[GPSInfo] = None
    velocity: Optional[Velocity] = None

class DroneCommand(BaseModel):
    command: str
    parameters: Optional[Dict[str, Any]] = None

class DroneResponse(BaseModel):
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None

class DroneCommand(BaseModel):
    command: str
    parameters: Optional[Dict[str, Any]] = None

class DroneResponse(BaseModel):
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None
