import { useState, useEffect } from "react";
import { DronePosition } from "./types";
import { cn } from "../../lib/utils";
import { useConnectionStore } from "../../stores/connection-store";

const DURATION = 2000;
interface DroneMarkerProps {
  drone: DronePosition;
}

export const DroneMarker = ({ drone }: DroneMarkerProps): JSX.Element => {
  const [isAnimating, setIsAnimating] = useState(false);
  const [isPulsing, setIsPulsing] = useState(drone.pulse);
  const { connected } = useConnectionStore();
  useEffect(() => {
    if (isAnimating) {
      const timer = setTimeout(() => {
        setIsAnimating(false);
      }, DURATION);

      return () => clearTimeout(timer);
    }
  }, [isAnimating]);

  useEffect(() => {
    if (isPulsing) {
      const timer = setTimeout(() => {
        setIsPulsing(false);
      }, DURATION + 2000);

      return () => clearTimeout(timer);
    }
  }, [isPulsing]);

  const handleMove = () => {
    setIsAnimating(true);
    setIsPulsing(true);
  };

  return (
    <>
      <div
        key={drone.id}
        onClick={handleMove}
        className={cn(
          "absolute w-[74px] h-16",
          "transition-all duration-2000 ease-linear",
          isAnimating && "animating",
          drone.className
        )}
        style={{
          transitionDuration: DURATION + "ms",
        }}
      >
        {drone.type === "active" && connected ? (
          <div className="relative w-14 h-[55px] top-1">
            <img
              className="absolute w-[53px] h-[55px] top-0 left-[3px]"
              alt="Polygon"
              src="/polygon-3.svg"
            />
            <div className="absolute w-[55px] h-[38px] top-2.5 left-0">
              <div
                className={cn(
                  "absolute w-[38px] h-[38px] top-0 left-0 bg-[#54cde2] rounded-[19px] overflow-hidden",
                  (isPulsing || drone.pulse) &&
                    "border-2 border-orange-600 animate-[pulse_2s_ease-in-out]"
                )}
              >
                <img
                  className="absolute w-5 h-5 top-[9px] left-[9px]"
                  alt="Quadcopter"
                  src="/quadcopter.svg"
                />
              </div>
              <img
                className="absolute w-3 h-[13px] top-[11px] left-10"
                alt="Polygon"
                src="/polygon-1.svg"
              />
            </div>
          </div>
        ) : (
          <div className="absolute w-[55px] h-[38px] top-3.5 left-0">
            <div
              className={cn(
                "absolute w-[38px] h-[38px] top-0 left-0 bg-[#d9d9d9] rounded-[19px] overflow-hidden border-0",
                (isPulsing || drone.pulse) &&
                  "border-2 border-orange-600 animate-[pulse_2s_ease-in-out]"
              )}
            >
              <img
                className="absolute w-5 h-5 top-[9px] left-[9px]"
                alt="Quadcopter"
                src="/quadcopter.svg"
              />
            </div>
          </div>
        )}

        {/* Optional: Add a visual indicator during animation */}
        {isAnimating && (
          <div className="absolute -bottom-4 left-1/2 transform -translate-x-1/2 text-xs text-white bg-black/50 px-2 py-0.5 rounded-full whitespace-nowrap">
            Moving...
          </div>
        )}
      </div>
    </>
  );
};
