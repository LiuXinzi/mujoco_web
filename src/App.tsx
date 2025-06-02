// App.tsx

import { useState, useRef } from "react";
import { Canvas, useThree, useFrame } from "@react-three/fiber";
import { OrbitControls, PerspectiveCamera } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";

import { Mujoco } from "./components/Mujoco";
import ZoomSlider from "./components/utils";

import { EffectComposer, DepthOfField } from "@react-three/postprocessing";

import "./App.css";
import "./index.css";

/* ───────── 相机控制器：滑块 ↔ 滚轮 双向同步 ───────── */
function CameraRig({
  distRef,
  setDist,
}: {
  distRef: React.MutableRefObject<number>;
  setDist: (v: number) => void;
}) {
  const { camera } = useThree();
  const controls = useRef<OrbitControlsImpl | null>(null);
  const lastDist = useRef(distRef.current);

  useFrame(() => {
    // ① 滑块值改变 → 推相机到目标半径
    const targetRadius = distRef.current;
    if (Math.abs(targetRadius - lastDist.current) > 1e-4) {
      const dir = camera.position.clone().normalize().multiplyScalar(targetRadius);
      camera.position.copy(dir);
      lastDist.current = targetRadius;
    }

    // ② 更新 OrbitControls 内部状态
    controls.current?.update();

    // ③ 鼠标滚轮缩放 → 读取相机真实半径 → 回写滑块
    const actualRadius = camera.position.length();
    if (Math.abs(actualRadius - distRef.current) > 0.05) {
      distRef.current = actualRadius;
      setDist(actualRadius);
      lastDist.current = actualRadius;
    }
  });

  return <OrbitControls ref={controls} makeDefault enableZoom />;
}

const App = () => {
  // 滑块当前值
  const [dist, setDist] = useState(3);
  // 不触发渲染的实时距离
  const distRef = useRef(dist);

  return (
    // 外层容器：相对定位 + 全屏铺满
    <div className="relative w-screen h-screen">
      {/* Canvas 撑满父容器 */}
      <Canvas
        shadows="soft"
        dpr={window.devicePixelRatio}
        className="w-full h-full"
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
        <DepthEffects />
      </Canvas>

      {/* ---------------- 滑块控件 ---------------- */}
      <ZoomSlider
        value={dist}
        onChange={(v) => {
          setDist(v);
          distRef.current = v; // 下一帧同步相机
        }}
      />
    </div>
  );
};

export default App;

/* 
  DepthEffects 组件：自动根据 Canvas 真实高度调整 DepthOfField
*/
function DepthEffects() {
  const { size } = useThree();
  return (
    <EffectComposer>
      <DepthOfField
        focusDistance={0}
        focalLength={0.02}
        bokehScale={2}
        height={size.height}
      />
    </EffectComposer>
  );
}
