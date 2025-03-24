import { DronePosition } from "./types";

interface DroneMarkerProps {
  drone: DronePosition;
}

export const DroneMarker = ({ drone }: DroneMarkerProps): JSX.Element => {
  return (
    <div key={drone.id} className={`absolute w-[74px] h-16 ${drone.className}`}>
      {drone.type === "active" ? (
        <div className="relative w-14 h-[55px] top-1">
          <img
            className="absolute w-[53px] h-[55px] top-0 left-[3px]"
            alt="Polygon"
            src="/polygon-3.svg"
          />
          <div className="absolute w-[55px] h-[38px] top-2.5 left-0">
            <div className="absolute w-[38px] h-[38px] top-0 left-0 bg-[#54cde2] rounded-[19px] overflow-hidden">
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
          <div className="absolute w-[38px] h-[38px] top-0 left-0 bg-[#d9d9d9] rounded-[19px] overflow-hidden">
            <img
              className="absolute w-5 h-5 top-[9px] left-[9px]"
              alt="Quadcopter"
              src="/quadcopter.svg"
            />
          </div>
        </div>
      )}
    </div>
  );
};
