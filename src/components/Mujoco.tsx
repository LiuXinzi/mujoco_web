import { memo, useEffect, useRef, useState } from "react";

import * as THREE from "three";

import { useFrame, useThree } from "@react-three/fiber";

import { UpdateProps } from "./UpdateProps";
import { MujocoContainer } from "./MujocoContainer";
import {
  loadMujocoModule,
  buildThreeScene,
  updateThreeScene,
  loadMujocoScene
} from "./mujocoUtils";

export interface MujocoProps {
  sceneUrl: string;
  frameIndex: number;
  onLoadedTrajectory?: (T: number) => void;
}

export const MujocoComponent = ({ 
  sceneUrl,
  frameIndex,
  onLoadedTrajectory,
 }: MujocoProps) => {
  // The 35ms threshold acts as a safeguard to prevent the simulation from
  // accumulating too much lag, which could degrade performance or accuracy.
  const MAX_SIMULATION_LAG_MS = 35.0;

  const { scene } = useThree();

  // This is to block the scene rendering until the scene has been loaded.
  const loadingSceneRef = useRef<boolean>(false);

  // True if the scene fails to load.
  const errorRef = useRef<boolean>(false);

  // Variables used to update the ThreeJS scene.
  const mujocoTimeRef = useRef(0);
  const updatePropsRef = useRef<UpdateProps>();
  const tmpVecRef = useRef<THREE.Vector3>(new THREE.Vector3(0, 0, 0));

  // The container of the MuJoCo module, model, state and simulation.
  const [mujocoContainer, setMujocoContainer] = useState<MujocoContainer | null>(null);

  // Load MuJoCo WASM with a default empty scene when the component mounts.
  useEffect(() => {
    const setupMujocoModule = async () => {
      try {
        const mujocoContainer = await loadMujocoModule();
        if (mujocoContainer) {
          setMujocoContainer(mujocoContainer);
        }
      } catch (error: unknown) {
        errorRef.current = true;
        console.error(error);
      }
    };
    setupMujocoModule();
  }, []);

  // Load a scene each time the scene URL changes.
  useEffect(() => {
    const setupMujocoScene = async () => {
      try {
        if (mujocoContainer) {
          // Time-consuming operation that should be moved into a Worker.
          loadMujocoScene(mujocoContainer, sceneUrl);

          // updatePropsRef.current = await buildThreeScene(mujocoContainer, scene);
          const props = await buildThreeScene(mujocoContainer, scene);
          updatePropsRef.current = props;

          // 通知父组件，有多少帧、多少点
          if (onLoadedTrajectory) {
            const { raw, T, N } = mujocoContainer.getTrajectory();
            onLoadedTrajectory(T);
          }
        }
      } catch (error: unknown) {
        errorRef.current = true;
        console.error(error);
      }
    };
    if (mujocoContainer) {
      try {
        loadingSceneRef.current = true;
        setupMujocoScene();
      } finally {
        loadingSceneRef.current = false;
      }
    }
  }, [mujocoContainer, scene, sceneUrl]);
  useEffect(() => {
    if (updatePropsRef.current) {
      updatePropsRef.current.updatePointCloud(frameIndex);
    }
  }, [frameIndex]);
  // Update the Three.js scene with information from the MuJoCo simulation.
  useFrame(({ clock }) => {
    if (!mujocoContainer || loadingSceneRef.current || errorRef.current) {
      return;
    }

    const simulation = mujocoContainer.getSimulation();
    const model = simulation.model();
    if (!model || !simulation) {
      return;
    }

    const timeMS = clock.getElapsedTime() * 1000;
    const timestep = model.getOptions().timestep;

    // If the real elapsed time (timeMS) has advanced more than 35 milliseconds
    // beyond the simulation's current time, the simulation time is reset to
    // match the real time. This prevents the simulation from falling too far
    // behind real time, which could happen if the rendering or simulation steps lag.
    if (timeMS - mujocoTimeRef.current > MAX_SIMULATION_LAG_MS) {
      mujocoTimeRef.current = timeMS;
    }
    // This while loop ensures that the simulation progresses in fixed timesteps
    // until it catches up with the real elapsed time.
    while (mujocoTimeRef.current < timeMS) {
      simulation.step();
      mujocoTimeRef.current += timestep * 1000;
    }

    if (!updatePropsRef.current) {
      return;
    }
    updateThreeScene(mujocoContainer, updatePropsRef.current, tmpVecRef.current);
  });

  return null; // This component doesn't render anything directly.
};

// Memoize the named component
export const Mujoco = memo(MujocoComponent);
