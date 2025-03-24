import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { Element } from "./screens/Element";

createRoot(document.getElementById("app") as HTMLElement).render(
  <StrictMode>
    <Element />
  </StrictMode>
);
