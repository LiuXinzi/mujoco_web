import { useState, useRef } from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";

import { Mujoco } from "./components/Mujoco";
import ZoomSlider from "./components/utils";

import { DepthOfField, EffectComposer } from "@react-three/postprocessing";

import "./App.css";
import "./index.css";

/* ───────── 相机控制器：滑块 ←→ 滚轮 双向同步 ───────── */
function CameraRig({
  distRef,
  setDist,
}: {
  distRef: React.MutableRefObject<number>;
  setDist: (v: number) => void;
}) {
  const { camera } = useThree();
  const controls = useRef<OrbitControlsImpl | null>(null);

  // 记录上一次已同步到相机的半径，避免重复写
  const lastDist = useRef(distRef.current);

  useFrame(() => {
    /* ---------- ① 若滑块改变，把相机推到目标半径 ---------- */
    const targetRadius = distRef.current;
    if (Math.abs(targetRadius - lastDist.current) > 1e-4) {
      const dir = camera.position.clone().normalize().multiplyScalar(targetRadius);
      camera.position.copy(dir);
      lastDist.current = targetRadius;
    }

    /* ---------- ② 更新 OrbitControls 内部状态 ---------- */
    controls.current?.update();

    /* ---------- ③ 读取滚轮缩放后的真实半径，如有变化回写滑块 ---------- */
    const actualRadius = camera.position.length();
    if (Math.abs(actualRadius - distRef.current) > 0.05) {
      distRef.current = actualRadius; // 写回 ref（供 ① 判断）
      setDist(actualRadius);          // 更新滑块 UI
      lastDist.current = actualRadius;
    }
  });

  // 开启鼠标滚轮缩放
  return <OrbitControls ref={controls} makeDefault enableZoom />;
}

const App = () => {
  /* ------- 状态：滑块当前值 ------- */
  const [dist, setDist] = useState(3);          // 初始 3 m
  const distRef = useRef(dist);                 // 不触发渲染的即时数值

  return (
    <div className="w-full h-full border-4 border-blue-500" style={{ position: "relative" }}>
      <Canvas
        shadows="soft"
        dpr={window.devicePixelRatio}
        style={{ borderRadius: "inherit", margin: "0 auto", width: 600, height: 400 }}
        onCreated={(state) => {
          state.scene.background = new THREE.Color(0x264059);
        }}
      >
        {/* ---------------- 灯光 ---------------- */}
        <ambientLight color={0xffffff} intensity={0.1} />
        <spotLight position={[0, 2, 2]} angle={0.15} penumbra={1} decay={1} intensity={3.14} />

        {/* ---------------- 相机 & 控制器 ---------------- */}
        <PerspectiveCamera makeDefault position={[2, 1.7, 1.7]} fov={45} />
        <CameraRig distRef={distRef} setDist={setDist} />

        {/* ---------------- MuJoCo 场景 ---------------- */}
        <Mujoco sceneUrl={"agility_cassie/scene.xml"} />

        {/* ---------------- 后期效果 ---------------- */}
        <EffectComposer>
          <DepthOfField focusDistance={0} focalLength={0.02} bokehScale={2} height={480} />
        </EffectComposer>
      </Canvas>

      {/* ---------------- 滑块控件 ---------------- */}
      <ZoomSlider
        value={dist}
        onChange={(v) => {
          setDist(v);          // 更新 UI
          distRef.current = v; // 写回 ref → 下一帧相机同步
        }}
      />
    </div>
  );
};

export default App;
