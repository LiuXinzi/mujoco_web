import * as THREE from "three";
import { Body } from "./mujocoUtils";

/**
 * This class is just a container of different variables used to update the
 * Three.js scene.
 */
export class UpdateProps {

    private bodies: { [key: number]: Body } = {};
    private lights: THREE.Light[] = [];
    private cylinders: THREE.InstancedMesh<THREE.CylinderGeometry> | null = null;
    private spheres: THREE.InstancedMesh<THREE.SphereGeometry> | null = null;

    /** 点云对象 */
    public pointCloud!: THREE.Points;
    /** 扁平的轨迹数据 [T*N*3] */
    public trajectory!: Float32Array;
    /** 帧数 */
    public frameCount!: number;
    /** 单帧点数 */
    public pointCount!: number;

    constructor(bodies: { [key: number]: Body }, lights: THREE.Light[], cylinders: THREE.InstancedMesh<THREE.CylinderGeometry>, spheres: THREE.InstancedMesh<THREE.SphereGeometry>) {
        this.bodies = bodies;
        this.lights = lights;
        this.cylinders = cylinders;
        this.spheres = spheres;
    }

    public setPointCloud(
            points: THREE.Points,
            trajectory: Float32Array,
            T: number,
            N: number
            ): void {
    this.pointCloud = points;
    this.trajectory = trajectory;
    this.frameCount = T;
    this.pointCount = N;
    }
    public updatePointCloud(k: number): void {
        const posAttr = this.pointCloud.geometry.attributes
            .position as THREE.BufferAttribute;
        const array = posAttr.array as Float32Array;
        const start = k * this.pointCount * 3;
        array.set(this.trajectory.subarray(start, start + this.pointCount * 3));
        posAttr.needsUpdate = true;
    }
    public getBodies(): { [key: number]: Body } {
        return this.bodies;
    }

    public getLights(): THREE.Light[] {
        return this.lights;
    }

    public getCylinders(): THREE.InstancedMesh<THREE.CylinderGeometry> | null {
        return this.cylinders;
    }

    public getSpheres(): THREE.InstancedMesh<THREE.SphereGeometry> | null {
        return this.spheres;
    }

}