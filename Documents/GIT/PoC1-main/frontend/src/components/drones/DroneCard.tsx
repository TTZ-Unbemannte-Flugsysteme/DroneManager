import { XIcon } from "lucide-react";
import { Card, CardContent } from "../../components/ui/card";
import { DroneData } from "./types";

interface DroneCardProps {
  drone: DroneData;
}

export const DroneCard = ({ drone }: DroneCardProps): JSX.Element => {
  return (
    <Card className="bg-[#3535354c] rounded-[10px] border border-solid border-[#ffffff1a] backdrop-blur-[7px]">
      <div className="p-[18px] pb-0 flex justify-between items-center">
        <span className="font-normal text-white text-xs">{drone.name}</span>
        <XIcon className="w-2.5 h-2.5 text-white cursor-pointer" />
      </div>

      <CardContent
        className={`p-5 ${
          drone.image
            ? `bg-[url(${drone.image})] bg-cover bg-[50%_50%]`
            : "bg-[#1a1a1a]"
        } rounded-[20px] mt-2 h-[196px] relative`}
      >
        <div className="absolute bottom-0 left-0 right-0 h-[61px] bg-[#b2b1b14c] rounded-[10px] backdrop-blur-[3px] p-2">
          <div className="grid grid-cols-3 gap-2 text-xs text-white">
            <div className="flex items-center gap-2">
              <div
                className={`w-2 h-2 ${drone.status.connection.color} rounded`}
              />
              <span>{drone.status.connection.text}</span>
            </div>
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 ${drone.status.armed.color} rounded`} />
              <span>{drone.status.armed.text}</span>
            </div>
            <div className="flex items-center gap-2">
              <div className={`w-2 h-2 ${drone.status.flight.color} rounded`} />
              <span>{drone.status.flight.text}</span>
            </div>
            <div className="flex items-center gap-2">
              <img
                className="w-[13px] h-1.5"
                alt="Group"
                src={`/group${
                  drone.id === 1 ? "" : drone.id === 2 ? "-1" : "-3"
                }.png`}
              />
              <span>{drone.status.time}</span>
            </div>
            <div className="flex items-center gap-2">
              <img
                className="w-[13px] h-2.5"
                alt="Vector"
                src="/vector-3.svg"
              />
              <span>{drone.status.speed}</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="flex space-x-1">
                <img
                  className="w-3.5 h-3.5"
                  alt="Vector"
                  src="/vector-10.svg"
                />
                <img className="w-3 h-3" alt="Vector" src="/vector.svg" />
                <img className="w-3.5 h-3.5" alt="Vector" src="/vector-8.svg" />
              </div>
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
};
