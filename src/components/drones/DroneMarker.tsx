// @ts-nocheck
import { useState, useEffect } from "react";
import { DronePosition } from "./types";
import { cn } from "../../lib/utils";
import { useConnectionStore } from "../../stores/connection-store";

const DURATION = 500; // Reduced for smoother movement

interface DroneMarkerProps {
  drone: DronePosition;
  wsData: {
    position: [number, number, number];
    heading: number;
    conn: boolean;
    armed: boolean;
    in_air: boolean;
  };
}

export const DroneMarker = ({
  drone,
  wsData,
}: DroneMarkerProps): JSX.Element => {
  const [isAnimating, setIsAnimating] = useState(false);
  const [isPulsing, setIsPulsing] = useState(wsData?.in_air || false);
  const [position, setPosition] = useState({ left: "45%", top: "25%" });

  useEffect(() => {
    if (wsData?.position) {
      const [x, y] = wsData.position;

      // Convert the raw position values to screen coordinates
      const convertToScreenPosition = (value: number) => {
        // Scale down the large values to fit in our view
        // Assuming values are roughly between -100 and 100
        const scaled = (value + 100) / 100; // Convert to 0-1 range
        return Math.max(5, Math.min(95, scaled * 10)); // Convert to 5-95% range
      };

      // Directly use x and y for positioning
      const leftPercent = convertToScreenPosition(x);
      const topPercent = convertToScreenPosition(y);

      setPosition({
        left: `${leftPercent}%`,
        top: `${topPercent}%`,
      });

      setIsPulsing(wsData.in_air);
    }
  }, [wsData]);


  // console.log(wsData)
  return (
    <div
      key={drone.id}
      className={cn(
        "absolute w-[74px] h-16",
        "transition-all ease-out duration-500"
      )}
      style={{
        left: position.left,
        top: position.top, // Center the drone at its position
      }}
    >
      {/* Drone name label */}
      <div className="absolute -top-6 left-0 w-full text-center text-xs font-semibold text-white bg-black/50 px-2 py-0.5 rounded-full">
        {drone.id.charAt(0).toUpperCase() + drone.id.slice(1)}
      </div>

      <div className="relative w-14 h-[55px]">
        {/* Shadow/background that rotates with the drone */}
        <div
          className="absolute w-full h-full origin-center"
          style={{
            transform: `rotate(${wsData?.heading || 0}deg)`,
            transition: "transform 500ms ease-out",
          }}
        >
          <img
            className="absolute w-[60px] h-[55px] top-1/2 left-1/2 -translate-x-1/4 -translate-y-1/2"
            alt="Polygon"
            src="/polygon-3.svg"
          />
        </div>

        {/* Drone body */}
        <div className="absolute w-[55px] h-[38px] top-2.5 left-1/2 -translate-x-1/2">
          <div
            className={cn(
              "absolute w-[38px] h-[38px] top-0 left-1/2 -translate-x-1/2",
              wsData?.conn ? "bg-[#54cde2]" : "bg-[#d9d9d9]",
              "rounded-[19px] overflow-hidden"
              // wsData?.in_air && "border-2 border-orange-600 animate-[pulse_2s_ease-in-out]"
            )}
          >
            <img
              className="absolute w-5 h-5 top-[9px] left-[9px] transition-transform duration-500"
              alt="Quadcopter"
              src="/quadcopter.svg"
              style={{
                transform: `rotate(${wsData?.heading || 0}deg)`,
              }}
            />
          </div>
        </div>
      </div>

      {/* Status indicators */}
      {/* <div className="absolute -bottom-2 left-0 w-full flex justify-center gap-1">
        <div
          className={cn(
            "w-2 h-2 rounded-full",
            wsData?.conn ? "bg-green-500" : "bg-red-500"
          )}
        />
        <div
          className={cn(
            "w-2 h-2 rounded-full",
            wsData?.in_air ? "bg-blue-500" : "bg-gray-500"
          )}
        />
      </div> */}
    </div>
  );
};
