import { Canvas } from "@react-three/fiber";
import { Mujoco } from "./components/Mujoco";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import * as THREE from "three";

import "./App.css";
import "./index.css";
import { DepthOfField, EffectComposer } from "@react-three/postprocessing";
import { useState } from "react";

const App = () => {
  const [frameIndex, setFrameIndex] = useState(0);
  const [maxFrame, setMaxFrame]     = useState(0);
  return (
    <div id="canvas-container" className="relative w-screen h-screen">
      <button
        onClick={() => {
          const el = document.getElementById("canvas-container");
          if (el?.requestFullscreen) {
            el.requestFullscreen();
          }
        }}
        className="absolute top-4 right-4 z-20 p-2 bg-white bg-opacity-75 rounded"
      >
        Full Screen
      </button>

      <div className="absolute top-4 left-4 z-20 p-2 bg-white bg-opacity-75 rounded">
        <label className="block mb-1">
          Frame: {frameIndex} / {maxFrame}
        </label>
        <input
          type="range"
          min={0}
          max={maxFrame}
          value={frameIndex}
          onChange={e => setFrameIndex(Number(e.target.value))}
        />
      </div>
      <div className="flex items-center justify-center w-full h-full">
      <Canvas
        shadows="soft"
        dpr={window.devicePixelRatio}
        style={{ width: "70%", height: "70%" }}
        onCreated={(state) => {
          state.scene.background = new THREE.Color(0x264059);
        }}
      >
        <ambientLight color={0xffffff} intensity={0.1} />
        <spotLight
          position={[0, 2, 2]}
          angle={0.15}
          penumbra={1}
          decay={1}
          intensity={3.14}
        />
        <PerspectiveCamera makeDefault position={[2.0, 1.7, 1.7]} fov={45} />
        <OrbitControls makeDefault />
        <Mujoco
          sceneUrl={"humanoid.xml"}
          frameIndex={frameIndex}
          onLoadedTrajectory={T=> {
            setMaxFrame(T - 1);
          }}
        />

        <EffectComposer>
          <DepthOfField
            focusDistance={0}
            focalLength={0.02}
            bokehScale={2}
            height={480}
          />
        </EffectComposer>
      </Canvas>
      </div>
    </div>
  );
};
export default App;
