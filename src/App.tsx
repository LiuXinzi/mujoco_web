/* ---------- React & Three 基础 ---------- */
import { useState, useRef, useEffect } from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import * as THREE from "three";

/* ---------- MuJoCo 组件（已暴露 camera） ---------- */
import { Mujoco, MujocoHandle } from "./components/Mujoco";

/* ---------- UI：滑块 ---------- */
import ZoomSlider from "./components/utils";

/* ---------- 后期景深 ---------- */
import { DepthOfField, EffectComposer } from "@react-three/postprocessing";

/* ---------- 样式 ---------- */
import "./App.css";
import "./index.css";

/* === ★ 新增：把 MuJoCo 距离同步到 Three.js 相机 === */
function CameraSync({ mujocoRef }: { mujocoRef: React.RefObject<MujocoHandle> }) {
  const { camera, controls } = useThree();           // controls 由 drei 自动注入
  const last = useRef<number>(0);

  useFrame(() => {
    const d = mujocoRef.current?.camera?.getZoom?.();
    if (typeof d === "number" && Math.abs(d - last.current) > 1e-4) {
      const dir = camera.position.clone().normalize().multiplyScalar(d);
      camera.position.copy(dir);
      (controls as any)?.update?.();                 // 同步 OrbitControls 内部状态
      last.current = d;
    }
  });

  return null;                                       // 不渲染任何内容
}

const App = () => {
  const [dist, setDist] = useState(3);
  const mujocoRef = useRef<MujocoHandle>(null);

  /* ===== 全局滚轮 → MuJoCo zoom ===== */
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
        {/* ---------- 灯光 ---------- */}
        <ambientLight intensity={0.1} />
        <spotLight position={[0, 2, 2]} angle={0.15} penumbra={1} decay={1} intensity={3.14} />

        {/* ---------- Three.js 相机固定方向，距离由 CameraSync 决定 ---------- */}
        <PerspectiveCamera makeDefault position={[0, 0, 1]} fov={45} />
        <OrbitControls makeDefault enableZoom={false} />

        {/* ---------- 同步器：每帧把距离写给 Three.js 相机 ---------- */}
        <CameraSync mujocoRef={mujocoRef} />

        {/* ---------- MuJoCo 场景 ---------- */}
        <Mujoco ref={mujocoRef} sceneUrl={"agility_cassie/scene.xml"} />

        {/* ---------- 后期效果 ---------- */}
        <EffectComposer>
          <DepthOfField focusDistance={0} focalLength={0.02} bokehScale={2} height={480} />
        </EffectComposer>
      </Canvas>

      {/* ---------- 滑块控件 ---------- */}
      <ZoomSlider
        value={dist}
        onChange={(v) => {
          setDist(v);
          mujocoRef.current?.camera?.setZoom(v);     // 绝对距离写进 MuJoCo
        }}
      />
    </div>
  );
};

export default App;
