Backend : FAST API : cd backend
*Create a virtual environment**
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


Frontend: 
1. **Clone the repository and navigate to the frontend directory**
    ```bash
    cd frontend
    ```

2. **Install dependencies**
    ```bash
    npm install
    ```

3. **Set up environment variables**
    
    Create a `.env.local` file in the root directory with the following values:
    ```
    VITE_API_URL=http://localhost:8000/api
    VITE_DEFAULT_DRONE_NAME=drone1
    VITE_DEFAULT_UDP=udp://:14540
    ```

## Running the Application

1. **Start the development server**
    ```bash
    npm run dev
    ```

2. **Access the application**
    
    Open your browser and navigate to [http://localhost:5173](http://localhost:5173)

3. **Build for production**
    ```bash
    npm run build
    ```

## Prototype Mode

The application is designed to work in two modes:

1. **Connected Mode**: Connects to the backend API described in the documentation to control real drones.
2. **Prototype Mode**: If the connection to the actual drone or Gazebo simulator fails, the application automatically falls back to using static data from the frontend files.

This approach was taken due to potential issues with connecting to Gazebo in some environments. The prototype mode ensures that users can still experience the full UI and workflow even without a functional drone connection.

## Features

- **Drone Connection**: Connect to drones via the backend API
- **Drone Monitoring**: View drone status, position, and telemetry data
- **Map View**: Visualize drone positions in a map interface
- **Drone Swap**: Simulate drone swap operations when battery is low
- **Responsive Design**: Works on both desktop and mobile devices

## Key Components

### Connection Store

Located in `src/stores/connection-store.tsx`, this Zustand store manages the connection state and drone positions:

```typescript
// Key features of the connection store
export const useConnectionStore = create<ConnectionState>()(
  persist(
     (set) => ({
        connected: false,
        droneId: null,
        swapped: false,
        searchStarted: false,
        dronePoss: dronePositions || [],
        setDronePoss: (dronePoss) => set({ dronePoss }),
        setSearchStarted: (searchStarted) => set({ searchStarted }),
        setSwapped: (swapped) => set({ swapped }),
        setConnected: (connected, droneId = null) =>
          set({
             connected,
             droneId: connected ? droneId : null,
          }),
     }),
     {
        name: "drone-connection-state",
        storage: createJSONStorage(() => sessionStorage),
     }
  )
);
```

### API Integration

The application uses React Query to interact with the backend API:

```typescript
// Example of API integration for connecting to a drone
const { mutate: connectDrone, isPending: isConnecting } = useConnectDrone();

// Usage
connectDrone(
  {
     drone_id: droneId,
     connection_string: connectionString,
  },
  {
     onSuccess: () => {
        setIsConnected(true);
        setShowConnectionPopup(false);
     },
     onError: (error) => {
        // Handle connection errors
     },
  }
);
```

### Fallback to Static Data

If the connection to the real drone fails, the application uses static data from `src/data/droneData.ts`:

```typescript
// Sample static drone data
export const droneData = [
  {
     id: 1,
     name: "John",
     image: "/image1.png",
     status: {
        connection: { text: "Connected", color: "bg-green-500" },
        armed: { text: "Armed", color: "bg-green-500" },
        flight: { text: "In Air", color: "bg-green-500" },
        time: "50 mins",
        speed: "12km/h",
     },
  },
  // More drones...
];
```

## Troubleshooting

- If you can't connect to the backend API, check that the backend server is running and that your `.env.local` file has the correct API URL.
- If you experience issues with the Gazebo simulator, the application will automatically use static data to demonstrate functionality.
- For connection issues, verify the UDP port settings in the connection dialog.

## Development

The project structure follows a component-based architecture with the following organization:

- `/src/components`: Reusable UI components
- `/src/screens`: Main application screens
- `/src/stores`: State management with Zustand
- `/src/api`: API integration with React Query
- `/src/data`: Static data for prototype mode
- `/src/lib`: Utility functions and helpers

When extending the application, consider adding new components to the appropriate directory and updating the state management as needed.
