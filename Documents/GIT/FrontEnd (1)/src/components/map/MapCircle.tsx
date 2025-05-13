import { Button } from "../../components/ui/button";


const MapCircle = () => {
  return (
    <>
    <div className="absolute left-0 bottom-6 w-[300px] h-[300px] ">
      <div className="relative h-full w-full">
        <div className="absolute inset-0">
          <div className="relative h-full w-full  overflow-hidden">
            <img
              className="absolute top-[10px] left-[10px]"
              alt="Map"
              src="/map1.png"
            />
            {/* <div className="absolute inset-0 bg-[#ffffff45] blur-[2px]" /> */}
          </div>
        </div>
        <img
          className="absolute w-[40px] h-[40px] top-[80px] left-[100px]"
          alt="Location Pin"
          src="/pin1.png"
        />
      </div>
    </div>
    <div className="flex items-center gap-4">
          <Button
            variant="outline"
            className="flex ml-4 items-center gap-4 h-[36px] w-[250px] px-3 bg-[#e8e8e85d] rounded-[8px] border-none backdrop-blur-[24px] hover:bg-[#3d3d3d] transition-colors"
          >
            <img className="w-4 h-4" alt="Mission" src="/vector-4.svg" />
            <span className="text-[#dbdbdb] text-xs font-medium">Esplanade 10, building G101</span>
            
          </Button>
        </div>
    </>
    
    
  );
};

export default MapCircle;
