from fastapi import FastAPI, HTTPException
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Optional

from .models import DroneInfo,DroneCommand,DroneResponse
from .dronecontrol.dronemanager import DroneManager
from .dronecontrol.drone import DroneMAVSDK



@asynccontextmanager
async def lifespan(app: FastAPI):
    drone_manager = DroneManager(DroneMAVSDK)
    app.state.drone_manager = drone_manager
    
    try:
        await drone_manager.load_drone_connections()
        yield
    finally:
        drone_manager.save_drone_connections()

app = FastAPI(title="POC1 - Drone Control API",lifespan=lifespan)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)



@app.get("/api")
def index() -> dict[str, str]:
    return {"message": "POC1 - Drone Control API"}
@app.get("/api/drones", response_model=List[DroneInfo])
async def get_all_drones():
    drone_manager = app.state.drone_manager
    drones = []
    
    for drone_id, drone in drone_manager.drones.items():
        # Get battery info - first battery in the dictionary (index 0)
        battery_info = None
        if hasattr(drone, "batteries") and drone.batteries and 0 in drone.batteries:
            battery = drone.batteries[0]
            battery_info = {
                "remaining": battery.remaining,  # Percentage remaining
                "voltage": battery.voltage,      # Voltage
                "temperature": battery.temperature  # Temperature in degrees C
            }
        
        # Get position data from position_ned property (North, East, Down coordinates)
        position = None
        if hasattr(drone, "position_ned"):
            pos = drone.position_ned
            position = {
                "x": float(pos[0]),  # North
                "y": float(pos[1]),  # East
                "z": float(pos[2])   # Down
            }
        
        # Get attitude (roll, pitch, yaw)
        attitude = None
        if hasattr(drone, "attitude"):
            att = drone.attitude
            attitude = {
                "roll": float(att[0]),
                "pitch": float(att[1]),
                "yaw": float(att[2])
            }
        
        # Collect status information
        status = {
            "connected": getattr(drone, "is_connected", False),
            "armed": getattr(drone, "is_armed", False),
            "in_air": getattr(drone, "in_air", False),
            "flight_mode": str(getattr(drone, "flightmode", "unknown"))
        }
        
        # Get GPS data if available
        gps_info = None
        if hasattr(drone, "position_global"):
            gps = drone.position_global
            gps_info = {
                "latitude": float(gps[0]),
                "longitude": float(gps[1]),
                "altitude": float(gps[2])
            }
            
        # Get velocity data if available
        velocity = None
        if hasattr(drone, "velocity"):
            vel = drone.velocity
            velocity = {
                "north": float(vel[0]),
                "east": float(vel[1]),
                "down": float(vel[2])
            }
        
        # Create the complete drone info object
        drone_info = DroneInfo(
            id=drone_id,
            name=drone_id,  # Use ID as name if no specific name property
            status=status,
            battery=battery_info,
            position=position,
            attitude=attitude,
            gps=gps_info,
            velocity=velocity
        )
        
        drones.append(drone_info)
    
    return drones

@app.post("/api/drones/connect", response_model=DroneResponse)
async def connect_drone(drone_id: str="drone1", connection_string: str = "udp://:14540"):
    """Connect to a drone"""
    drone_manager = app.state.drone_manager

    try:

        # Directly await the async method
        result = await drone_manager.connect_to_drone(
            drone_id, None, None, connection_string, timeout=30.0
        )
        if result:
            return DroneResponse(
                success=True,
                message=f"Successfully connected to drone {drone_id}",
                data={"drone_id": drone_id}
            )
        else:
            return DroneResponse(
                success=False,
                message=f"Failed to connect to drone {drone_id}"
            )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/arm", response_model=DroneResponse)
async def arm_drone(drone_id: str):
    drone_manager = app.state.drone_manager

    """Arm a specific drone"""
    if drone_id not in drone_manager.drones:
        raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    
    try:
        # Directly await the async method
        result = await drone_manager.arm([drone_id])
        return DroneResponse(
            success=True,
            message=f"Armed drone {drone_id}",
            data={"drone_id": drone_id}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/disarm", response_model=DroneResponse)
async def disarm_drone(drone_id: str):
    """Disarm a specific drone"""
    drone_manager = app.state.drone_manager

    if drone_id not in drone_manager.drones:
        raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    
    try:
        result = await drone_manager.disarm([drone_id])
        return DroneResponse(
            success=True,
            message=f"Disarmed drone {drone_id}",
            data={"drone_id": drone_id}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/takeoff", response_model=DroneResponse)
async def takeoff_drone(drone_id: str, altitude: float = 5.0):
    """Command a specific drone to take off"""

    drone_manager = app.state.drone_manager
    if drone_id not in drone_manager.drones:
        raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    
    try:
        result = await drone_manager.takeoff([drone_id], altitude)
        return DroneResponse(
            success=True,
            message=f"Drone {drone_id} taking off to {altitude}m",
            data={"drone_id": drone_id, "altitude": altitude}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/land", response_model=DroneResponse)
async def land_drone(drone_id: str):
    """Command a specific drone to land"""
    drone_manager = app.state.drone_manager

    if drone_id not in drone_manager.drones:
        raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    
    try:
        result = await drone_manager.land([drone_id])
        return DroneResponse(
            success=True,
            message=f"Drone {drone_id} landing",
            data={"drone_id": drone_id}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/flyto", response_model=DroneResponse)
async def fly_to_position(
    drone_id: str, 
    x: float, 
    y: float, 
    z: float, 
    yaw: Optional[float] = None
):
    """Command a specific drone to fly to a position"""

    drone_manager = app.state.drone_manager
    if drone_id not in drone_manager.drones:
        raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    
    try:
        result = await drone_manager.fly_to(
            drone_id, 
            local=[x, y, z], 
            yaw=yaw
        )
        return DroneResponse(
            success=True,
            message=f"Drone {drone_id} flying to position",
            data={"drone_id": drone_id, "position": {"x": x, "y": y, "z": z, "yaw": yaw}}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/drones/{drone_id}/command", response_model=DroneResponse)
async def execute_command(drone_id: str, command: DroneCommand):
    """Execute a custom command on a specific drone"""
    drone_manager = app.state.drone_manager

    if drone_id not in drone_manager.drones:
        raise HTTPException(status_code=404, detail=f"Drone {drone_id} not found")
    
    try:
        cmd = command.command.lower()
        params = command.parameters or {}
        
        if cmd == "arm":
            result = await drone_manager.arm([drone_id])
            return DroneResponse(
                success=True, 
                message=f"Armed drone {drone_id}"
            )
        
        elif cmd == "disarm":
            result = await drone_manager.disarm([drone_id])
            return DroneResponse(
                success=True, 
                message=f"Disarmed drone {drone_id}"
            )
        
        elif cmd == "takeoff":
            altitude = params.get("altitude", 5.0)
            result = await drone_manager.takeoff([drone_id], altitude)
            return DroneResponse(
                success=True, 
                message=f"Drone {drone_id} taking off to {altitude}m",
                data={"altitude": altitude}
            )
        
        elif cmd == "land":
            result = await drone_manager.land([drone_id])
            return DroneResponse(
                success=True, 
                message=f"Drone {drone_id} landing"
            )
        
        elif cmd == "flyto":
            if not all(k in params for k in ['x', 'y', 'z']):
                raise HTTPException(
                    status_code=400, 
                    detail="Missing position parameters (x, y, z)"
                )
            
            x = params.get('x')
            y = params.get('y')
            z = params.get('z')
            yaw = params.get('yaw')
            
            result = await drone_manager.fly_to(
                drone_id, 
                local=[x, y, z], 
                yaw=yaw
            )
            return DroneResponse(
                success=True, 
                message=f"Drone {drone_id} flying to position",
                data={"position": {"x": x, "y": y, "z": z, "yaw": yaw}}
            )
        
        else:
            return DroneResponse(
                success=False,
                message=f"Unsupported command: {cmd}"
            )
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))