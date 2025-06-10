import { MujocoModule, Simulation } from "../wasm/mujoco_wasm";

/**
 * A container for the MuJoCo WASM module and simulation.
 */
export class MujocoContainer {
    private mujocoModule: MujocoModule;
    private simulation: Simulation;
    private trajectoryRaw: Float32Array | null = null;
    private trajectoryT = 0;
    private trajectoryN = 0;
    constructor(mujocoModule: MujocoModule, simulation: Simulation) {
        this.mujocoModule = mujocoModule;
        this.simulation = simulation;
    }
    public setTrajectory(raw: Float32Array, T: number, N: number): void {
        this.trajectoryRaw = raw;
        this.trajectoryT = T;
        this.trajectoryN = N;
    }
    public getTrajectory(): { raw: Float32Array; T: number; N: number } {
        if (!this.trajectoryRaw) {
            throw new Error("Trajectory not initialized");
            }
        return { raw: this.trajectoryRaw, T: this.trajectoryT, N: this.trajectoryN };
    }

    public getMujocoModule(): MujocoModule {
        return this.mujocoModule;
    }
    public setMujocoModule(mujocoModule: MujocoModule): void {
        this.mujocoModule = mujocoModule;
    }
    public getSimulation(): Simulation {
        return this.simulation;
    }
    public setSimulation(simulation: Simulation): void {
        this.simulation = simulation;
    }
}