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
    <div className="relative w-full h-full border-4 border-blue-500">
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
      <Canvas
        shadows="soft"
        dpr={window.devicePixelRatio}
        style={{
          borderRadius: "inherit",
          margin: "0 auto", // Center horizontally.
          width: 600,
          height: 400
        }}
        onCreated={(state) => {
          state.scene.background = new THREE.Color(0x264059);
        }}
      >
        {/* <AdaptiveDpr /> */}
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
        {/* Post-Processing Effects */}
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
  );
};
export default App;
