import { Button } from "../../components/ui/button";
import { Loader, MapPinIcon } from "lucide-react";
import { DroneMarker } from "../drones/DroneMarker";
import DroneCardsSidebar from "../drones/DroneCardsSidebar";
import { useConnectionStore } from "../../stores/connection-store";
import { useFlyDrone } from "../../api/useDrone";

export const MapView = (): JSX.Element => {
  const {
    connected,
    setSearchStarted,
    dronePoss: dronePositions,
    setDronePoss,
  } = useConnectionStore();

  const { mutate, isPending } = useFlyDrone();

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
    <div className="relative w-full">
      {/* Top controls */}
      <div className="flex items-center mb-3">
        <Button
          variant="outline"
          className="flex flex-col w-[54px] h-[58px] items-center justify-center gap-1.5 bg-[#e7e7e74c] rounded-[10px] backdrop-blur-[11.5px] border-none hover:bg-[#e7e7e770] transition-colors"
        >
          <img className="w-5 h-5" alt="Mission" src="/vector-5.svg" />
          <span className="text-white text-xs">Mission</span>
        </Button>

        <div className="flex items-center gap-4 ml-4">
          {connected && (
            <Button
              onClick={handleSearchStart}
              className="h-[58px] px-8 bg-[#ffffff4c] rounded-[10px] text-white text-xs hover:bg-[#ffffff60] transition-colors focus:ring-2 focus:ring-white/20 focus:outline-none"
            >
              {isPending && <Loader className="animate-spin" />} Start Search
              Operation
            </Button>
          )}

          <div className="h-[58px]">
            <div className="flex h-full items-center gap-3 px-5 bg-[#e7e7e74c] rounded-[10px]">
              <MapPinIcon className="w-4 h-5 text-white" />
              <div className="text-white text-xs">Somewhere 10, G101</div>
            </div>
          </div>
        </div>
        <div className="flex items-center gap-4 ml-4 md:hidden">
          <DroneCardsSidebar />
        </div>
      </div>

      {/* Map content */}
      <div className="relative h-[450px] bg-[#494949] rounded-lg overflow-hidden">
        {/* Dark inner section */}
        <div className="absolute w-[80%] h-[250px] top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2 bg-[#2d2d2d]" />

        {/* Drone positions */}
        {dronePositions.map((drone) => (
          <DroneMarker key={drone.id} drone={drone} />
        ))}
      </div>
    </div>
  );
};
