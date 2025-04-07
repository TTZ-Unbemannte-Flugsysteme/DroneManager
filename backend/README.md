# PoC1 Backend

This guide will help you set up and run the PoC1 backend application.

## Prerequisites

- Visual Studio Code
- Python 3.7+
- Git

## Setup Instructions

1. **Extract and open the folder in Visual Studio Code**
    ```bash
    # Open VS Code and use File > Open Folder to navigate to the extracted project
    # Or use the command line:
    code path/to/PoC1/backend
    ```

2. **Create a virtual environment**
    ```bash
    python -m venv .venv
    ```

3. **Activate the virtual environment**
    - On Windows:
      ```bash
      .\.venv\Scripts\activate
      ```
    - On macOS/Linux:
      ```bash
      source .venv/bin/activate
      ```

4. **Install required dependencies**
    ```bash
    pip install -r requirements.txt
    ```

5. **Run the FastAPI development server**
    ```bash
    fastapi dev .\main.py
    ```

6. **Access the API documentation**
    - Open your browser and navigate to [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs)
    - The Swagger UI will display all available endpoints and allow you to test them

7. **Follow along with the demonstration video**
    - Make sure your simulation is running while testing the endpoints
    - Refer to the video for specific API interaction examples

## Troubleshooting

- If you encounter any issues with dependencies, ensure your Python version is compatible
- Check that all required services are running before testing the API


## API Documentation

The PoC1 backend provides a RESTful API for controlling drones remotely. Here's a detailed reference for all available endpoints:

### Core Endpoints

#### Get All Drones
- **Endpoint**: `GET /api/drones`
- **Description**: Retrieves information about all connected drones
- **Response**: List of drone objects containing ID, status, position, attitude, GPS coordinates, and battery information

#### Connect a Drone
- **Endpoint**: `POST /api/drones/connect`
- **Description**: Establishes a connection to a drone
- **Parameters**:
    - `drone_id` (string): Unique identifier for the drone
    - `connection_string` (string): Connection string in format `protocol://address:port` (e.g., `udp://:14540`)
- **Example**:
    ```json
    {
        "drone_id": "drone1",
        "connection_string": "udp://:14540"
    }
    ```

### Drone Control Endpoints

#### Arm Drone
- **Endpoint**: `POST /api/drones/{drone_id}/arm`
- **Description**: Arms the specified drone
- **Path Parameter**: `drone_id` (string)

#### Disarm Drone
- **Endpoint**: `POST /api/drones/{drone_id}/disarm`
- **Description**: Disarms the specified drone
- **Path Parameter**: `drone_id` (string)

#### Takeoff
- **Endpoint**: `POST /api/drones/{drone_id}/takeoff`
- **Description**: Commands the drone to take off to a specified altitude
- **Path Parameter**: `drone_id` (string)
- **Query Parameter**: `altitude` (float, default: 5.0m)

#### Land
- **Endpoint**: `POST /api/drones/{drone_id}/land`
- **Description**: Commands the drone to land
- **Path Parameter**: `drone_id` (string)

#### Fly to Position
- **Endpoint**: `POST /api/drones/{drone_id}/flyto`
- **Description**: Commands the drone to fly to a specific position
- **Path Parameter**: `drone_id` (string)
- **Request Body**:
    - `x` (float): North coordinate in meters
    - `y` (float): East coordinate in meters
    - `z` (float): Down coordinate in meters (negative values are up)
    - `yaw` (float, optional): Yaw angle in degrees

#### Execute Command
- **Endpoint**: `POST /api/drones/{drone_id}/command`
- **Description**: Executes a custom command on the drone
- **Path Parameter**: `drone_id` (string)
- **Request Body**:
    - `command` (string): Command to execute (e.g., `arm`, `disarm`, `takeoff`, `land`, `flyto`)
    - `parameters` (object): Command-specific parameters
- **Example**:
    ```json
    {
        "command": "flyto",
        "parameters": {
            "x": 10.0,
            "y": 5.0,
            "z": -3.0,
            "yaw": 90.0
        }
    }
    ```

## Working with the API

### Connection Process
1. Connect to a drone using the `/api/drones/connect` endpoint
2. Verify connection status with the `/api/drones` endpoint
3. Arm the drone using the `arm` endpoint
4. Take off to begin flight operations

### Flight Operations
- Use the `flyto` endpoint for precise positioning
- Monitor drone status using the `/api/drones` endpoint
- Always land the drone using the `land` endpoint before disconnecting

### Error Handling
The API returns appropriate HTTP status codes:
- 200: Success
- 404: Drone not found
- 500: Server error (with detailed error message)

### Response Format
All endpoints return responses in the following format:
```json
{
    "success": true,
    "message": "Operation description",
    "data": { /* operation-specific data */ }
}
```
