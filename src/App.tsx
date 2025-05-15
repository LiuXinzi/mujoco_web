
import { useState, useRef, useEffect } from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import * as THREE from "three";

import { Mujoco, MujocoHandle } from "./components/Mujoco";

import ZoomSlider from "./components/utils";

import { DepthOfField, EffectComposer } from "@react-three/postprocessing";

import "./App.css";
import "./index.css";

function CameraSync({ mujocoRef }: { mujocoRef: React.RefObject<MujocoHandle> }) {
  const { camera, controls } = useThree();          
  const last = useRef<number>(0);

  useFrame(() => {
    const d = mujocoRef.current?.camera?.getZoom?.();
    if (typeof d === "number" && Math.abs(d - last.current) > 1e-4) {
      const dir = camera.position.clone().normalize().multiplyScalar(d);
      camera.position.copy(dir);
      (controls as any)?.update?.();                 
      last.current = d;
    }
  });

  return null;                                    
}

const App = () => {
  const [dist, setDist] = useState(3);
  const mujocoRef = useRef<MujocoHandle>(null);

  useEffect(() => {
    const wheel = (e: WheelEvent) => {
      const factor = 1 + e.deltaY * 0.002;
      mujocoRef.current?.camera?.zoom(factor);
      const z = mujocoRef.current?.camera?.getZoom?.();
      if (typeof z === "number") setDist(z);
    };
    window.addEventListener("wheel", wheel, { passive: true });
    return () => window.removeEventListener("wheel", wheel);
  }, []);

  return (
    <div className="w-full h-full border-4 border-blue-500" style={{ position: "relative" }}>
      <Canvas
        dpr={window.devicePixelRatio}
        shadows="soft"
        style={{ borderRadius: "inherit", margin: "0 auto", width: 600, height: 400 }}
        onCreated={({ scene }) => (scene.background = new THREE.Color(0x264059))}
      >
        <ambientLight intensity={0.1} />
        <spotLight position={[0, 2, 2]} angle={0.15} penumbra={1} decay={1} intensity={3.14} />

        <PerspectiveCamera makeDefault position={[0, 0, 1]} fov={45} />
        <OrbitControls makeDefault enableZoom={false} />

        <CameraSync mujocoRef={mujocoRef} />

        <Mujoco ref={mujocoRef} sceneUrl={"agility_cassie/scene.xml"} />

        <EffectComposer>
          <DepthOfField focusDistance={0} focalLength={0.02} bokehScale={2} height={480} />
        </EffectComposer>
      </Canvas>

      <ZoomSlider
        value={dist}
        onChange={(v) => {
          setDist(v);
          mujocoRef.current?.camera?.setZoom(v);     
        }}
      />
    </div>
  );
};

export default App;
