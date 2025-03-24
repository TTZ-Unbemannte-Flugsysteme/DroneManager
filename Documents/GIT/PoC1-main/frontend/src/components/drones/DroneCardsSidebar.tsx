import { useState } from "react";
import { droneData } from "../../data/droneData";
import { DroneCard } from "./DroneCard";
import { Sheet, SheetContent, SheetTrigger } from "../../components/ui/sheet";
import { ScrollArea } from "../../components/ui/scroll-area";
import { PanelsRightBottom } from "lucide-react";
import { Button } from "../ui/button";

const DroneCardsSidebar = () => {
  const [open, setOpen] = useState(false);

  return (
    <>
      {/* Visible on larger screens, hidden on mobile */}
      <div className="hidden md:block">
        <ScrollArea className="md:h-[calc(100vh-110px)]">
          <div className="space-y-4 pr-4">
            {droneData.map((drone) => (
              <DroneCard key={drone.id} drone={drone} />
            ))}
          </div>
        </ScrollArea>
      </div>

      {/* Sheet for mobile devices */}
      <div className="md:hidden">
        <Sheet open={open} onOpenChange={setOpen}>
          <SheetTrigger asChild>
            <Button
              variant="outline"
              size="icon"
              className=" bg-[#1a1a1a80] backdrop-blur-md hover:bg-[#ffffff20] border-[#ffffff20]"
            >
              <PanelsRightBottom className="h-5 w-5 text-white" />
            </Button>
          </SheetTrigger>
          <SheetContent
            side="right"
            className="w-[85vw] sm:w-[350px] bg-[#353535] border-none"
          >
            <div className="py-6">
              <h2 className="text-lg font-semibold text-white mb-4">Drones</h2>
              <ScrollArea className="h-[calc(100vh-150px)]">
                <div className="space-y-4 pr-4">
                  {droneData.map((drone) => (
                    <DroneCard key={drone.id} drone={drone} />
                  ))}
                </div>
              </ScrollArea>
            </div>
          </SheetContent>
        </Sheet>
      </div>
    </>
  );
};

export default DroneCardsSidebar;
