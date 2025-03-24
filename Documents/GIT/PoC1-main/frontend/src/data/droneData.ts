import { DroneData, DronePosition } from "../components/drones/types";

// Data for drone cards
export const droneData: DroneData[] = [
  {
    id: 1,
    name: "Drone1",
    image: "/image.png",
    status: {
      connection: { text: "Connected", color: "bg-[#8bf600]" },
      armed: { text: "Armed", color: "bg-[#55cde2]" },
      flight: { text: "In Air", color: "bg-[#55cde2]" },
      time: "50 mins",
      speed: "13km/h",
    },
  },
  {
    id: 2,
    name: "Luke",
    image: "/image-1.png",
    status: {
      connection: { text: "Connected", color: "bg-[#8bf600]" },
      armed: { text: "Armed", color: "bg-[#55cde2]" },
      flight: { text: "In Air", color: "bg-[#55cde2]" },
      time: "50 mins",
      speed: "13km/h",
    },
  },
  {
    id: 3,
    name: "Tom",
    image: null, // No image, using dark background
    status: {
      connection: { text: "Disconnected", color: "bg-[#f64e00]" },
      armed: { text: "Armed", color: "bg-[#f64e00]" },
      flight: { text: "In Air", color: "bg-[#f64e00]" },
      time: "50 mins",
      speed: "0km/h",
    },
  },
];

// Updated drone positions with responsive classes
export const dronePositions: DronePosition[] = [
  {
    id: 1,
    className: "top-[35%] left-[15%] md:top-[200px] md:left-[176px]",
    type: "standard",
  },
  {
    id: 2,
    className: "top-[25%] left-[45%] md:top-[235px] md:left-[442px]",
    type: "active",
  },
  {
    id: 3,
    className: "top-[30%] left-[45%] md:top-[290px] md:left-[442px]",
    type: "active",
  },
];