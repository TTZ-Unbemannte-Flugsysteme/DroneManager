import { XIcon } from "lucide-react";

export const Header = (): JSX.Element => {
  return (
    <div className="h-[45px] flex items-center bg-[#2d2d2d] px-6">
      <div className="w-[34px] h-[34px] bg-[#54cde2] rounded-sm flex-shrink-0" />

      <div className="ml-6 flex items-center">
        <img className="h-8 w-auto" alt="Logo" src="/group-2.png" />

        <div className="ml-4 flex items-center gap-2 border-l border-gray-500 pl-4">
          <span className="text-[#dbdbdb] text-xs font-medium">Operation</span>
          <button className="hover:bg-[#3d3d3d] p-1 rounded">
            <XIcon className="w-3 h-3 text-white" />
          </button>
        </div>
      </div>

      <div className="ml-auto flex items-center gap-4">
        <div className="w-2 h-2 bg-green-400 rounded-full"></div>
        <span className="text-white text-xs">Connected</span>
      </div>
    </div>
  );
};
