export interface DroneStatus {
  connection: { text: string; color: string };
  armed: { text: string; color: string };
  flight: { text: string; color: string };
  time: string;
  speed: string;
}

export interface DroneData {
  id: number;
  name: string;
  image: string | null;
  status: DroneStatus;
}

export interface DronePosition {
  id: number;
  className: string;
  type: "standard" | "active";
}