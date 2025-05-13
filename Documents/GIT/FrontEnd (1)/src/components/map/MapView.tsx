// @ts-nocheck
import { useState, useEffect } from "react";
import { Button } from "../../components/ui/button";
import { Loader, MapPinIcon } from "lucide-react";
import { DroneMarker } from "../drones/DroneMarker";
import DroneCardsSidebar from "../drones/DroneCardsSidebar";
import { useConnectionStore } from "../../stores/connection-store";
import { useFlyDrone } from "../../api/useDrone";
import { useWebSocket } from "../../hooks/useWebSocket";
import { DroneMarkerData } from "./types";

export const MapView = (): JSX.Element => {
  const {
    connected,
    setSearchStarted,
    dronePoss: dronePositions,
    setDronePoss,
  } = useConnectionStore();

  const { mutate, isPending } = useFlyDrone();

  const { isConnected, droneData } = useWebSocket();
  const [droneMarkers, setDroneMarkers] = useState<DroneMarkerData[]>([]);
  // console.log(droneData)
  useEffect(() => {
    if (droneData?.drones) {
      const markers: DroneMarkerData[] = Object.entries(droneData.drones).map(([droneName, droneInfo]) => ({
        id: droneName,
        name: droneName,
        type: "active",
        className: "",
        movedClassName: "",
        pulse: droneInfo.in_air,
        wsData: droneInfo
      }));
      setDroneMarkers(markers);
    }
  }, [droneData]);


  const handleSearchStart = async () => {
    await new Promise((resolve) => setTimeout(resolve, 1000)); // Simulate async operation
    setSearchStarted(true);
    setDronePoss(
      dronePositions.map((drone) => ({
        ...drone,
        className:
          drone.id === 1 ? drone.movedClassName || "" : drone.className || "",
      }))
    );
    await new Promise((resolve) => setTimeout(resolve, 2000)); // Simulate async operation
    setDronePoss(
      dronePositions.map((drone) => ({
        ...drone,
        className:
          drone.id === 1 ? drone.movedClassName || "" : drone.className || "",
        pulse: drone.id === 1,
      }))
    );
  };

  // Function to handle server-side search start
  const handleServerSearchStart = () => {
    mutate(
      {
        droneId: "1",
      },
      {
        onSuccess: async () => {
          await handleSearchStart();
        },
        onError: () => {
          alert("Error during search start");
          setSearchStarted(false);
        },
      }
    );
  };

  return (
    <>
    <div className="relative w-full">
 

      {/* Map content */}
      <div className="relative h-[300px] mx-24 bg-[#494949]  overflow-hidden">
        {/* Dark inner section */}
        <div  />
        {/* className="absolute w-[80%] h-[250px] top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 bg-[#494949]" */}
        {/* Drone positions */}
        {/* {dronePositions.map((drone) => (
          <DroneMarker key={drone.id} drone={drone} />
        ))} */}
        {/* Render all drone markers */}
        {droneMarkers.map((drone) => (
          <DroneMarker 
            key={drone.id} 
            drone={drone}
            wsData={drone.wsData}
          />
        ))}
      </div>
    </div>
    </>
  );
};
