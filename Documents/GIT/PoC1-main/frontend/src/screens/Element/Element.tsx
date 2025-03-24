import { useState } from "react";
import { Header } from "../../components/layout/Header";
import { MapView } from "../../components/map/MapView";
import { MapTypeSelector } from "../../components/map/MapTypeSelector";
import { MapControls } from "../../components/map/MapControls";
import { dronePositions } from "../../data/droneData";
import { mapTypes } from "../../data/mapData";
import { MapType } from "../../components/map/types";
import MapCircle from "../../components/map/MapCircle";
import DroneCardsSidebar from "../../components/drones/DroneCardsSidebar";

export const Element = (): JSX.Element => {
  const [activeMapTypes, setActiveMapTypes] = useState<MapType[]>(mapTypes);

  const handleMapTypeSelect = (id: number) => {
    setActiveMapTypes(
      activeMapTypes.map((mapType) => ({
        ...mapType,
        active: mapType.id === id,
      }))
    );
  };

  const handleZoomIn = () => {
    console.log("Zoom in");
    // Implement zoom in functionality
  };

  const handleZoomOut = () => {
    console.log("Zoom out");
    // Implement zoom out functionality
  };

  return (
    <>
      <div className="bg-[#353535] flex relative">
        {/* Map Circle (same position on both layouts) */}
        <div className="fixed left-4 bottom-4 z-50">
          <MapCircle />
        </div>

        {/* Map Controls - desktop positioning */}
        <div className="fixed right-[375px] bottom-4 z-50 md:flex gap-4 h-auto hidden">
          <MapTypeSelector
            mapTypes={activeMapTypes}
            onSelect={handleMapTypeSelect}
          />
          <MapControls onZoomIn={handleZoomIn} onZoomOut={handleZoomOut} />
        </div>

        {/* Mobile controls - mobile only */}
        <div className="fixed right-4 bottom-4 z-50 flex gap-2 h-auto md:hidden">
          <MapTypeSelector
            mapTypes={activeMapTypes}
            onSelect={handleMapTypeSelect}
          />
          <MapControls onZoomIn={handleZoomIn} onZoomOut={handleZoomOut} />
        </div>

        {/* Content container */}
        <div className="flex-1 overflow-y-auto">
          <Header />

          {/* Main content area */}
          <div className="flex gap-8 p-8">
            <div className="flex-1 items-start align-top">
              <MapView dronePositions={dronePositions} />
            </div>

            {/* Sidebar container - DroneCardsSidebar handles responsiveness */}
            <div className="w-[315px] flex-col justify-between hidden md:flex">
              <DroneCardsSidebar />
            </div>
          </div>
        </div>
      </div>
    </>
  );
};
