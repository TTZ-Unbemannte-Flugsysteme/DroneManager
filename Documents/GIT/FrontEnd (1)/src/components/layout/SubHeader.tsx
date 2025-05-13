import { useState, useEffect } from "react";
import { XIcon, WifiIcon } from "lucide-react";
import { useConnectionStore } from "../../stores/connection-store";
import { Button } from "../../components/ui/button";

import {
  useConnectDrone,
  useDisconnectDrone,
  useGetDrones,
} from "../../api/useDrone";

export const Header1 = (): JSX.Element => {
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const [showConnectionPopup, setShowConnectionPopup] = useState(false);
  const [droneId, setDroneId] = useState(
    import.meta.env.VITE_DEFAULT_DRONE_NAME || "drone1"
  );
  const [connectionString, setConnectionString] = useState(
    import.meta.env.VITE_DEFAULT_UDP || "udp://:14540"
  );

  const { connected: isConnected, setConnected: setIsConnected } =
    useConnectionStore();

  // Use React Query hooks
  const {
    mutate: connectDrone,
    isPending: isConnecting,
    error: connectError,
  } = useConnectDrone();

  const { mutate: disconnectDrone } = useDisconnectDrone();

  const { data: dronesData, isSuccess: isDronesLoaded } = useGetDrones();

  // Check for existing drone connections when drones data is loaded
  useEffect(() => {
    if (isDronesLoaded && dronesData && dronesData.length > 0) {
      setIsConnected(true);
      setDroneId(dronesData[0].id);
    }
  }, [isDronesLoaded, dronesData, setIsConnected]);

  // Handle connection errors
  useEffect(() => {
    if (connectError) {
      setConnectionError(
        connectError instanceof Error
          ? connectError.message
          : "Connection failed"
      );
    }
  }, [connectError]);

  const handleConnect = async () => {
    setConnectionError(null);
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
          setConnectionError(
            error instanceof Error ? error.message : "Connection failed"
          );
          setIsConnected(true);
        },
      }
    );
  };

  const handleDisconnect = async () => {
    disconnectDrone(droneId);
    setIsConnected(false);
  };

  return (
   <>
    <div className="flex items-center justify-between w-full h-[60px] bg-[#2D2D2D] px-4 mb-3 rounded-[4px]">
        <div className="flex items-center gap-4">
          <Button
            variant="outline"
            className="flex ml-4 items-center gap-2 h-[32px] w-[180px] px-3 bg-[#e8e8e85d] rounded-[8px] border-none backdrop-blur-[24px] hover:bg-[#3d3d3d] transition-colors"
          >
            <img className="w-4 h-4" alt="Mission" src="/vector-5.svg" />
            <span className="text-[#dbdbdb] text-xs font-medium">Mission</span>
            <img className="" alt="Vector" src="/Vector.png" />
            
          </Button>
        </div>

        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2 mr-32">
            <div className="w-6 h-6 bg-green-400 rounded-full"></div>
            <span className="text-white text-xs ml-8">3 Drone detected</span>
          </div>
             <Button
            variant="outline"
            className="h-[32px] px-4 bg-transparent border-white text-xs text-white font-medium rounded-[10px]  hover:bg-black hover:text-white  "
          >
            Connect All
          </Button>
        </div>
      </div>
   </>
  );
};
