import {
  memo,
  useEffect,
  useRef,
  useState,
  useImperativeHandle,
  forwardRef,
} from "react";

import * as THREE from "three";
import { useFrame, useThree } from "@react-three/fiber";

import { UpdateProps } from "./UpdateProps";
import { MujocoContainer } from "./MujocoContainer";
import {
  loadMujocoModule,
  buildThreeScene,
  updateThreeScene,
  loadMujocoScene,
} from "./mujocoUtils";

export interface MujocoHandle {
  camera: any | null;
}

export interface MujocoProps {
  sceneUrl: string;
}

const MujocoComponent = forwardRef<MujocoHandle, MujocoProps>(
  ({ sceneUrl }, ref) => {
    const MAX_SIMULATION_LAG_MS = 35;

    const { scene } = useThree();
    const loadingSceneRef = useRef(false);
    const errorRef = useRef(false);

    const mujocoTimeRef = useRef(0);
    const updatePropsRef = useRef<UpdateProps>();
    const tmpVecRef = useRef(new THREE.Vector3());

    const [mujocoContainer, setMujocoContainer] =
      useState<MujocoContainer | null>(null);

    const cameraRef = useRef<any | null>(null);

    useEffect(() => {
      (async () => {
        try {
          const container = await loadMujocoModule();
          setMujocoContainer(container);
        } catch (e) {
          errorRef.current = true;
          console.error(e);
        }
      })();
    }, []);

    useEffect(() => {
      if (!mujocoContainer) return;
      (async () => {
        try {
          loadingSceneRef.current = true;
          loadMujocoScene(mujocoContainer, sceneUrl);
          updatePropsRef.current = await buildThreeScene(
            mujocoContainer,
            scene,
          );

          if (!cameraRef.current) {
            const Module: any = mujocoContainer.getMujocoModule(); // ★ 修改
            cameraRef.current = new Module.Camera();
            cameraRef.current.setZoom(3);
            (window as any).mujocoCam = cameraRef.current;
            console.log("[Debug] MuJoCo Camera 已挂载到 window.mujocoCam");
          }
        } catch (e) {
          errorRef.current = true;
          console.error(e);
        } finally {
          loadingSceneRef.current = false;
        }
      })();
    }, [mujocoContainer, sceneUrl, scene]);

    useFrame(({ clock }) => {
      if (!mujocoContainer || loadingSceneRef.current || errorRef.current) return;

      const simulation = mujocoContainer.getSimulation();
      const model = simulation.model();
      if (!model) return;

      const timeMS = clock.getElapsedTime() * 1000;
      const timestep = model.getOptions().timestep;

      if (timeMS - mujocoTimeRef.current > MAX_SIMULATION_LAG_MS) {
        mujocoTimeRef.current = timeMS;
      }
      while (mujocoTimeRef.current < timeMS) {
        simulation.step();
        mujocoTimeRef.current += timestep * 1000;
      }
      if (updatePropsRef.current) {
        updateThreeScene(mujocoContainer, updatePropsRef.current, tmpVecRef.current);
      }
    });

        useImperativeHandle(
      ref,
      () => ({ camera: cameraRef.current }),
      [cameraRef.current]
    );

        return null;
      },
    );

export const Mujoco = memo(MujocoComponent);
