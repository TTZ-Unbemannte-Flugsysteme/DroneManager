const MapCircle = () => {
  return (
    <div className="absolute left-0 bottom-0 w-[180px] h-[180px]">
      <div className="relative h-full w-full">
        <div className="absolute inset-0">
          <div className="relative h-full w-full rounded-full overflow-hidden">
            <img
              className="absolute w-[160px] h-[160px] top-[10px] left-[10px]"
              alt="Map"
              src="/image-150.png"
            />
            <div className="absolute inset-0 bg-[#ffffff45] rounded-full blur-[2px]" />
          </div>
        </div>
        <img
          className="absolute w-[20px] h-[30px] top-[75px] left-[80px]"
          alt="Location Pin"
          src="/vector-4.svg"
        />
      </div>
    </div>
  );
};

export default MapCircle;
