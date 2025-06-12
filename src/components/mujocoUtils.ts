import * as THREE from "three";
import load_mujoco, { MujocoModule } from "../wasm/mujoco_wasm";
import { mjtGeom } from "../wasm/mujoco_model_enums";
import { MujocoContainer } from "./MujocoContainer";
import { UpdateProps } from "./UpdateProps";
import { createMirrotCheckerboard } from "./threeUtils";

// Virtual Filesystem used by the WASM container.
const VIRTUAL_FILE_SYSTEM = "/working";

// The folder containing all examples.
const EXAMPLES_FOLDER = "./examples/scenes/";

// The name of ThreeJS group which contains the whole MuJoCo scene.
const ROOT_OBJECT_NAME = "MuJoCo Root";

/**
 * Represents a Three.js Mesh object with an optional body ID.
 */
export class BodyMesh extends THREE.Object3D {
  bodyID?: number;
}

/**
 * Represents a Three.js Group object with an optional body ID and a flag indicating
 * if the mesh has been customized.
 */
export class Body extends THREE.Object3D {
  bodyID?: number;
  has_custom_mesh?: boolean;
}

/**
 * Retrieves the position vector from a buffer at a specified index, applies optional
 * swizzling to match Three.js coordinate system, and sets it to the target Vector3.
 *
 * @function getPosition
 * @param buffer The buffer containing position data.
 * @param index The index of the vector in the buffer.
 * @param target The target Vector3 to set the position.
 * @param swizzle Whether to swizzle the coordinates for Three.js.
 * @returns The updated target Vector3 with the position set.
 */
const getPosition = (
  buffer: Float32Array | Float64Array,
  index: number,
  target: THREE.Vector3,
  swizzle: boolean = true
) => {
  if (swizzle) {
    return target.set(
      buffer[index * 3 + 0],
      buffer[index * 3 + 2],
      -buffer[index * 3 + 1]
    );
  } else {
    return target.set(
      buffer[index * 3 + 0],
      buffer[index * 3 + 1],
      buffer[index * 3 + 2]
    );
  }
};

/**
 * Retrieves the quaternion from a buffer at a specified index, applies optional
 * swizzling to match Three.js coordinate system, and sets it to the target Quaternion.
 *
 * @param buffer The buffer containing quaternion data.
 * @param index The index of the quaternion in the buffer.
 * @param target The target Quaternion to set the orientation.
 * @param swizzle Whether to swizzle the coordinates for Three.js.
 * @returns The updated target Quaternion with the orientation set.
 */
const getQuaternion = (
  buffer: Float32Array | Float64Array,
  index: number,
  target: THREE.Quaternion,
  swizzle: boolean = true
) => {
  if (swizzle) {
    return target.set(
      -buffer[index * 4 + 1],
      -buffer[index * 4 + 3],
      buffer[index * 4 + 2],
      -buffer[index * 4 + 0]
    );
  } else {
    return target.set(
      buffer[index * 4 + 0],
      buffer[index * 4 + 1],
      buffer[index * 4 + 2],
      buffer[index * 4 + 3]
    );
  }
};

/**
 * Decodes null-terminated strings from a specified segment of a buffer.
 * 
 * This function is useful for extracting null-terminated strings from a 
 * WebAssembly memory buffer or other binary data sources. It handles cases
 * where the buffer might be a `SharedArrayBuffer`, which is incompatible 
 * with `TextDecoder.decode()` due to specification restrictions.
 * 
 * The function creates a copy of the relevant segment of the buffer to a 
 * non-shared `Uint8Array`, decodes it as UTF-8, and splits it into an array
 * of strings using the null character (`\0`) as the delimiter.
 * 
 * @param buffer The memory buffer containing binary data, typically a 
 *               `SharedArrayBuffer` or `ArrayBuffer`.
 * @param length The total length of the buffer in bytes.
 * @param byteOffset The byte offset within the buffer to start reading from.
 * @param byteLength The number of bytes to read starting from `byteOffset`.
 * @returns An array of decoded strings extracted from the specified segment 
 *          of the buffer.
 */
const decode = (buffer: ArrayBufferLike, length: number, byteOffset: number, byteLength: number): string[] => {
  const textDecoder = new TextDecoder("utf-8");

  // When using WebAssembly threads (pthreads) with Emscripten, the memory is
  // shared between threads using SharedArrayBuffer.
  // The model.names buffer is a view on a SharedArrayBuffer.
  // According to the specification, TextDecoder.decode() cannot be used with
  // SharedArrayBuffer or views on it.

  // Create a non-shared buffer by copying data from the shared buffer.
  const nonSharedBuffer = new Uint8Array(length);
  nonSharedBuffer.set(
    new Uint8Array(buffer, byteOffset, byteLength)
  );

  // Decode the non-shared buffer and split it using the null character.
  const values = textDecoder.decode(nonSharedBuffer).split("\0");
  return values;
}

/**
 * Copies all necessary asset files to the MuJoCo WASM module's virtual filesystem.
 * This includes XML models, textures, and mesh data required for simulations.
 *
 * @param mujocoModule The loaded MuJoCo WASM module.
 * @returns A promise that resolves when all assets have been copied.
 * @throws Throws an error if fetching or writing any file fails.
 */
const copyMujocoModuleAssets = async (mujocoModule: MujocoModule) => {
  const allFiles = [
    "22_humanoids.xml",
    "adhesion.xml",
    "agility_cassie/assets/achilles-rod.obj",
    "agility_cassie/assets/cassie-texture.png",
    "agility_cassie/assets/foot-crank.obj",
    "agility_cassie/assets/foot.obj",
    "agility_cassie/assets/heel-spring.obj",
    "agility_cassie/assets/hip-pitch.obj",
    "agility_cassie/assets/hip-roll.obj",
    "agility_cassie/assets/hip-yaw.obj",
    "agility_cassie/assets/knee-spring.obj",
    "agility_cassie/assets/knee.obj",
    "agility_cassie/assets/pelvis.obj",
    "agility_cassie/assets/plantar-rod.obj",
    "agility_cassie/assets/shin.obj",
    "agility_cassie/assets/tarsus.obj",
    "agility_cassie/cassie.xml",
    "agility_cassie/scene.xml",
    "arm26.xml",
    "balloons/balloons.xml",
    "car.xml",
    "empty.xml",
    "hammock.xml",
    "humanoid.xml",
    "humanoid_body.xml",
    "mug/mug.obj",
    "mug/mug.png",
    "mug/mug.xml",
    "scene.xml",
    "shadow_hand/assets/f_distal_pst.obj",
    "shadow_hand/assets/f_knuckle.obj",
    "shadow_hand/assets/f_middle.obj",
    "shadow_hand/assets/f_proximal.obj",
    "shadow_hand/assets/forearm_0.obj",
    "shadow_hand/assets/forearm_1.obj",
    "shadow_hand/assets/forearm_collision.obj",
    "shadow_hand/assets/lf_metacarpal.obj",
    "shadow_hand/assets/mounting_plate.obj",
    "shadow_hand/assets/palm.obj",
    "shadow_hand/assets/th_distal_pst.obj",
    "shadow_hand/assets/th_middle.obj",
    "shadow_hand/assets/th_proximal.obj",
    "shadow_hand/assets/wrist.obj",
    "shadow_hand/left_hand.xml",
    "shadow_hand/right_hand.xml",
    "shadow_hand/scene_left.xml",
    "shadow_hand/scene_right.xml",
    "simple.xml",
    "slider_crank.xml",
    "model_with_tendon.xml",
    "spot/meshes/arm/collision/arm_link_el0.obj",
    "spot/meshes/arm/collision/arm_link_el1_lip.obj",
    "spot/meshes/arm/collision/arm_link_el1_main.obj",
    "spot/meshes/arm/collision/arm_link_sh0_base.obj",
    "spot/meshes/arm/collision/arm_link_sh0_left_motor.obj",
    "spot/meshes/arm/collision/arm_link_sh0_right_motor.obj",
    "spot/meshes/arm/collision/arm_link_sh1.obj",
    "spot/meshes/arm/collision/arm_link_wr0.obj",
    "spot/meshes/arm/collision/arm_link_wr1.obj",
    "spot/meshes/arm/visual/arm_link_el0.obj",
    "spot/meshes/arm/visual/arm_link_el1_0.obj",
    "spot/meshes/arm/visual/arm_link_el1_1.obj",
    "spot/meshes/arm/visual/arm_link_sh0.obj",
    "spot/meshes/arm/visual/arm_link_sh1_0.obj",
    "spot/meshes/arm/visual/arm_link_sh1_1.obj",
    "spot/meshes/arm/visual/arm_link_wr0_0.obj",
    "spot/meshes/arm/visual/arm_link_wr0_1.obj",
    "spot/meshes/arm/visual/arm_link_wr1_0.obj",
    "spot/meshes/arm/visual/arm_link_wr1_1.obj",
    "spot/meshes/base/collision/body_collision.obj",
    "spot/meshes/base/collision/left_lower_leg_collision.obj",
    "spot/meshes/base/collision/left_upper_leg_collision.obj",
    "spot/meshes/base/collision/right_lower_leg_collision.obj",
    "spot/meshes/base/collision/right_upper_leg_collision.obj",
    "spot/meshes/base/visual/body_0.obj",
    "spot/meshes/base/visual/body_1.obj",
    "spot/meshes/base/visual/front_left_hip.obj",
    "spot/meshes/base/visual/front_left_lower_leg.obj",
    "spot/meshes/base/visual/front_left_upper_leg_0.obj",
    "spot/meshes/base/visual/front_left_upper_leg_1.obj",
    "spot/meshes/base/visual/front_right_hip.obj",
    "spot/meshes/base/visual/front_right_lower_leg.obj",
    "spot/meshes/base/visual/front_right_upper_leg_0.obj",
    "spot/meshes/base/visual/front_right_upper_leg_1.obj",
    "spot/meshes/base/visual/rear_left_hip.obj",
    "spot/meshes/base/visual/rear_left_lower_leg.obj",
    "spot/meshes/base/visual/rear_left_upper_leg_0.obj",
    "spot/meshes/base/visual/rear_left_upper_leg_1.obj",
    "spot/meshes/base/visual/rear_right_hip.obj",
    "spot/meshes/base/visual/rear_right_lower_leg.obj",
    "spot/meshes/base/visual/rear_right_upper_leg_0.obj",
    "spot/meshes/base/visual/rear_right_upper_leg_1.obj",
    "spot/meshes/gripper/collision/front_jaw.obj",
    "spot/meshes/gripper/collision/jaw_tooth.obj",
    "spot/meshes/gripper/collision/left_finger.obj",
    "spot/meshes/gripper/collision/left_hinge.obj",
    "spot/meshes/gripper/collision/left_tooth.obj",
    "spot/meshes/gripper/collision/middle_jaw.obj",
    "spot/meshes/gripper/collision/right_finger.obj",
    "spot/meshes/gripper/collision/right_hinge.obj",
    "spot/meshes/gripper/collision/right_tooth.obj",
    "spot/meshes/gripper/visual/arm_link_fngr_0.obj",
    "spot/meshes/gripper/visual/arm_link_fngr_1.obj",
    "spot/spot_components/actuators_sensors.xml",
    "spot/spot_components/arm.xml",
    "spot/spot_components/assets.xml",
    "spot/spot_components/cartire.xml",
    "spot/spot_components/contact.xml",
    "spot/spot_components/legs.xml",
    "spot/spot_components/params_and_default.xml",
    "spot/textures/bdaii_spot_wrap.png",
    "spot/spot.xml"
  ];

  const requests = allFiles.map((url) => fetch(`${EXAMPLES_FOLDER}${url}`));
  const responses = await Promise.all(requests);
  for (let i = 0; i < responses.length; i++) {
    const split = allFiles[i].split("/");
    let working = `${VIRTUAL_FILE_SYSTEM}/`;
    // Create the directory structure if it doesn't exist.
    for (let f = 0; f < split.length - 1; f++) {
      working += split[f];
      if (!mujocoModule.FS.analyzePath(working).exists) {
        mujocoModule.FS.mkdir(working);
      }
      working += "/";
    }
    if (
      allFiles[i].endsWith(".png") ||
      allFiles[i].endsWith(".stl") ||
      allFiles[i].endsWith(".skn")
    ) {
      const data = new Uint8Array(await responses[i].arrayBuffer());
      mujocoModule.FS.writeFile(`${VIRTUAL_FILE_SYSTEM}/` + allFiles[i], data);
    } else {
      const text = await responses[i].text();
      mujocoModule.FS.writeFile(`${VIRTUAL_FILE_SYSTEM}/` + allFiles[i], text);
    }
  }
};

/**
 * Initializes the MuJoCo WASM module by loading it, setting up the virtual filesystem,
 * copying necessary assets, and creating a default simulation.
 *
 * @returns A promise that resolves to a MujocoContainer containing the loaded module and simulation.
 * @throws Throws an error if the module fails to load or initialize.
 */
export const loadMujocoModule = async (): Promise<MujocoContainer> => {
  // Load the WASM module.
  const mujocoModule = await load_mujoco();
  if (mujocoModule) {
    console.log("MuJoCo WASM module loaded successfully.");
  } else {
    throw new Error("MuJoCo WASM module returned an invalid value.");
  }

  // Initialize the file system.
  mujocoModule.FS.mkdir(VIRTUAL_FILE_SYSTEM);
  mujocoModule.FS.mount(mujocoModule.MEMFS, { root: "." }, VIRTUAL_FILE_SYSTEM);

  // Copy all necessary assets.
  await copyMujocoModuleAssets(mujocoModule);
  console.log("Successfully copied over all necessary assets.");

  // Create the default simulation.
  const model = new mujocoModule.Model(`${VIRTUAL_FILE_SYSTEM}/empty.xml`);

  // Check for errors in the MuJoCo model.
  if (model.getError() != "") {
    throw new Error(`Could not load the default model: ${model.getError()}`);
  }

  const state = new mujocoModule.State(model);
  const simulation = new mujocoModule.Simulation(model, state);
  console.log("Successfully created the default simulation.");
  // console.log(simulation)

  return new MujocoContainer(mujocoModule, simulation);
};

/**
 * Loads a MuJoCo scene from a specified URL into the provided MujocoContainer.
 * This involves freeing any existing simulation, loading the new model and state,
 * and initializing a new simulation.
 * 
 * IMPORTANT: this can be a very time-consuming computation.
 *
 * @param mujocoContainer The container holding the MuJoCo module and simulation.
 * @param sceneURL The URL path to the MuJoCo scene XML file to load.
 * @returns A promise that resolves when the scene has been successfully loaded.
 * @throws Throws an error if the scene fails to load or initialize.
 */
export const loadMujocoScene = (mujocoContainer: MujocoContainer, sceneURl: string): void => {
  const mujocoModule = mujocoContainer.getMujocoModule();
  const simulation = mujocoContainer.getSimulation();

  // Free the old simulation.
  if (simulation != null) {
    simulation.free();
  }

  const newModel = new mujocoModule.Model(`${VIRTUAL_FILE_SYSTEM}/${sceneURl}`);
  const newState = new mujocoModule.State(newModel);
  const newSimulation = new mujocoModule.Simulation(newModel, newState);
  console.log(newModel)
  console.log(newSimulation)
  // Check for errors in the MuJoCo model.
  if (newModel.getError() != "") {
    throw new Error(`Could not load the model "${sceneURl}": ${newModel.getError()}`);
  }

  mujocoContainer.setSimulation(newSimulation);
  const T = 10;
  const N = 6890;

  const rawArray = (mujocoModule as any).generateRandomPointCloud(T, N);
  // console.log("aaa",rawArray.size())
  const len = rawArray.size()
  const raw = new Float32Array(len);
  for (let i = 0; i < len; i++) {
  raw[i] = rawArray.get(i);
  }
  console.log(raw)
  mujocoContainer.setTrajectory(raw, T, N);
  console.log(mujocoContainer)
  console.log(`Successfully loaded the scene: ${sceneURl}`);
  
};



/**
 * Builds a Three.js scene based on the MuJoCo simulation data. It parses the MuJoCo
 * model, creates corresponding Three.js geometries and materials, sets up lighting,
 * and organizes objects into a hierarchical structure.
 *
 * @param mujocoContainer The container holding the MuJoCo module and simulation.
 * @param sceneUrl The URL path to the MuJoCo scene XML file to load.
 * @param scene The Three.js scene to which objects will be added.
 * @returns An object containing references to bodies, lights, and tendon meshes for updates.
 * @throws Throws an error if loading the MuJoCo scene fails.
 */
export const buildThreeScene = async (
  mujocoContainer: MujocoContainer,
  scene: THREE.Scene
): Promise<UpdateProps> => {

  // Remove the existing root object.
  const object = scene.getObjectByName(ROOT_OBJECT_NAME);
  if (object) {
    scene.remove(object);
  }
  const { raw: trajectory, T, N } = mujocoContainer.getTrajectory();
  const geo = new THREE.BufferGeometry();
  const positions = new Float32Array(N * 3);
  geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const mat = new THREE.PointsMaterial({ size: 0.04 });
  const points = new THREE.Points(geo, mat);
  // Create a new root object.
  const mujocoRoot = new THREE.Group();
  mujocoRoot.name = ROOT_OBJECT_NAME;
  mujocoRoot.add(points)
  scene.add(mujocoRoot);

  const bodies: { [key: number]: Body } = {};
  const meshes: { [key: number]: THREE.BufferGeometry } = {};
  const lights: THREE.Light[] = [];

  // Default material definition.
  let material = new THREE.MeshPhysicalMaterial();
  material.color = new THREE.Color(1, 1, 1);

  const simulation = mujocoContainer.getSimulation();
  const model = simulation.model();

  const names = decode(model.names.buffer, model.names.length, model.names.byteOffset, model.names.byteLength);
  // const paths = decode(model.paths.buffer, model.paths.length, model.paths.byteOffset, model.paths.byteLength);

  // Loop through the MuJoCo geometries and recreate them in three.js.
  for (let g = 0; g < model.ngeom; g++) {
    // Only visualize geom groups up to 2 (same default behavior as simulate).
    if (!(model.geom_group[g] < 3)) {
      continue;
    }

    // Get the body ID and type of the geom.
    const b = model.geom_bodyid[g];
    const type = model.geom_type[g];
    const size = [
      model.geom_size[g * 3 + 0],
      model.geom_size[g * 3 + 1],
      model.geom_size[g * 3 + 2]
    ];

    // Create the body if it doesn't exist.
    if (!(b in bodies)) {
      bodies[b] = new Body();
      bodies[b].name = names[model.name_bodyadr[b]];
      bodies[b].bodyID = b;
      bodies[b].has_custom_mesh = false;
    }

    // Set the default geometry. In MuJoCo, this is a sphere.
    let geometry: THREE.BufferGeometry = new THREE.SphereGeometry(size[0] * 0.5);

    if (type === mjtGeom.mjGEOM_PLANE) {
      // Special handling for plane later.
    } else if (type === mjtGeom.mjGEOM_HFIELD) {
      // TODO: Implement this.
    } else if (type === mjtGeom.mjGEOM_SPHERE) {
      geometry = new THREE.SphereGeometry(size[0]);
    } else if (type === mjtGeom.mjGEOM_CAPSULE) {
      geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
    } else if (type === mjtGeom.mjGEOM_ELLIPSOID) {
      geometry = new THREE.SphereGeometry(1); // Stretch this below
    } else if (type === mjtGeom.mjGEOM_CYLINDER) {
      geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
    } else if (type === mjtGeom.mjGEOM_BOX) {
      geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
    } else if (type === mjtGeom.mjGEOM_MESH) {
      const meshID = model.geom_dataid[g];

      if (!(meshID in meshes)) {
        geometry = new THREE.BufferGeometry(); // TODO: Populate the Buffer Geometry with Generic Mesh Data

        const vertex_buffer = model.mesh_vert.subarray(
          model.mesh_vertadr[meshID] * 3,
          (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3
        );
        for (let v = 0; v < vertex_buffer.length; v += 3) {
          //vertex_buffer[v + 0] =  vertex_buffer[v + 0];
          const temp = vertex_buffer[v + 1];
          vertex_buffer[v + 1] = vertex_buffer[v + 2];
          vertex_buffer[v + 2] = -temp;
        }
        geometry.setAttribute("position", new THREE.BufferAttribute(vertex_buffer, 3));

        const normal_buffer = model.mesh_normal.subarray(
          model.mesh_vertadr[meshID] * 3,
          (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3
        );
        for (let v = 0; v < normal_buffer.length; v += 3) {
          //normal_buffer[v + 0] =  normal_buffer[v + 0];
          const temp = normal_buffer[v + 1];
          normal_buffer[v + 1] = normal_buffer[v + 2];
          normal_buffer[v + 2] = -temp;
        }
        geometry.setAttribute("normal", new THREE.BufferAttribute(normal_buffer, 3));

        const uv_buffer = model.mesh_texcoord.subarray(
          model.mesh_texcoordadr[meshID] * 2,
          (model.mesh_texcoordadr[meshID] + model.mesh_vertnum[meshID]) * 2
        );
        geometry.setAttribute("uv", new THREE.BufferAttribute(uv_buffer, 2));

        const triangle_buffer = model.mesh_face.subarray(
          model.mesh_faceadr[meshID] * 3,
          (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3
        );
        geometry.setIndex(Array.from(triangle_buffer));

        // Step 1: Retrieve texture data from model
        // const textureAddress = model.tex_adr[meshID];
        // const texturePathAddress = model.tex_pathadr[meshID];
        // const textureData = model.tex_data[meshID];
        // const textureWidth = model.tex_width[meshID];
        // const textureHeight = model.tex_height[meshID];
        // const textureChannels = model.tex_nchannel[meshID];
        // const path = paths[meshID];

        // console.log(`path: ${path}, texturePathAddress: ${texturePathAddress}, textureWidth: ${textureWidth}, textureHeight: ${textureHeight}, textureChannels: ${textureChannels}`)

        meshes[meshID] = geometry;
      } else {
        geometry = meshes[meshID];
      }

      bodies[b].has_custom_mesh = true;
    }
    // Done with geometry creation.

    // Set the Material Properties of incoming bodies
    let texture = undefined;
    let color = [
      model.geom_rgba[g * 4 + 0],
      model.geom_rgba[g * 4 + 1],
      model.geom_rgba[g * 4 + 2],
      model.geom_rgba[g * 4 + 3]
    ];
    if (model.geom_matid[g] != -1) {
      const matId = model.geom_matid[g];
      color = [
        model.mat_rgba[matId * 4 + 0],
        model.mat_rgba[matId * 4 + 1],
        model.mat_rgba[matId * 4 + 2],
        model.mat_rgba[matId * 4 + 3]
      ];

      // Construct Texture from model.tex_data
      texture = undefined;
      const texId = model.mat_texid[matId];
      if (texId != -1) {
        const width = model.tex_width[texId];
        const height = model.tex_height[texId];
        const offset = model.tex_adr[texId];
        const rgbArray = model.tex_data;
        const rgbaArray = new Uint8Array(width * height * 4);

        for (let p = 0; p < width * height; p++) {
          rgbaArray[p * 4 + 0] = rgbArray[offset + (p * 3 + 0)];
          rgbaArray[p * 4 + 1] = rgbArray[offset + (p * 3 + 1)];
          rgbaArray[p * 4 + 2] = rgbArray[offset + (p * 3 + 2)];
          rgbaArray[p * 4 + 3] = 1.0;
        }
        texture = new THREE.DataTexture(
          rgbaArray,
          width,
          height,
          THREE.RGBAFormat,
          THREE.UnsignedByteType
        );
        if (texId == 2) {
          texture.repeat = new THREE.Vector2(50, 50);
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
        } else {
          texture.repeat = new THREE.Vector2(1, 1);
          texture.wrapS = THREE.RepeatWrapping;
          texture.wrapT = THREE.RepeatWrapping;
        }

        texture.needsUpdate = true;
      }
    }

    if (
      material.color.r != color[0] ||
      material.color.g != color[1] ||
      material.color.b != color[2] ||
      material.opacity != color[3] ||
      material.map != texture
    ) {
      material = new THREE.MeshPhysicalMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        transparent: color[3] < 1.0,
        opacity: color[3],
        specularIntensity:
          model.geom_matid[g] != -1
            ? model.mat_specular[model.geom_matid[g]] * 0.5
            : undefined,
        reflectivity:
          model.geom_matid[g] != -1
            ? model.mat_reflectance[model.geom_matid[g]]
            : undefined,
        roughness:
          model.geom_matid[g] != -1
            ? 1.0 - model.mat_shininess[model.geom_matid[g]]
            : undefined,
        metalness: model.geom_matid[g] != -1 ? 0.1 : undefined,
        map: texture
      });
    }

    let mesh = new BodyMesh();
    if (type == 0) {
      const groundMirror = createMirrotCheckerboard();
      groundMirror.rotateX(-Math.PI / 2);
      mesh = groundMirror;
    } else {
      mesh = new THREE.Mesh(geometry, material);
    }

    mesh.castShadow = g == 0 ? false : true;
    mesh.receiveShadow = type != 7;
    mesh.bodyID = b;
    bodies[b].add(mesh);
    getPosition(model.geom_pos, g, mesh.position);
    if (type != 0) {
      getQuaternion(model.geom_quat, g, mesh.quaternion);
    }
    if (type == 4) {
      mesh.scale.set(size[0], size[2], size[1]);
    } // Stretch the Ellipsoid
  }

  // Parse tendons

  // An InstancedMesh allows the same geometry to be reused multiple times
  // with minimal performance cost.

  const tendonMat = new THREE.MeshPhongMaterial();
  tendonMat.color = new THREE.Color(0.8, 0.3, 0.3);

  const cylinders = new THREE.InstancedMesh(
    new THREE.CylinderGeometry(1, 1, 1),
    tendonMat,
    1023 // The number of cylinders.
  );
  cylinders.receiveShadow = true;
  cylinders.castShadow = true;
  mujocoRoot.add(cylinders);

  const spheres = new THREE.InstancedMesh(
    new THREE.SphereGeometry(1, 10, 10),
    tendonMat,
    1023 // The number of spheres.
  );
  spheres.receiveShadow = true;
  spheres.castShadow = true;
  mujocoRoot.add(spheres);

  // Parse lights.
  for (let l = 0; l < model.nlight; l++) {
    let light: THREE.SpotLight | THREE.DirectionalLight;
    if (model.light_directional[l]) {
      light = new THREE.DirectionalLight();
    } else {
      light = new THREE.SpotLight();
      light.decay = model.light_attenuation[l] * 100;
      light.penumbra = 0.5;
    }
    light.castShadow = true; // default false
    light.shadow.mapSize.width = 1024; // default
    light.shadow.mapSize.height = 1024; // default
    light.shadow.camera.near = 1; // default
    light.shadow.camera.far = 10; // default
    //bodies[model.light_bodyid()].add(light);
    if (bodies[0]) {
      bodies[0].add(light);
    } else {
      mujocoRoot.add(light);
    }
    lights.push(light);
  }
  if (model.nlight == 0) {
    const light = new THREE.DirectionalLight();
    mujocoRoot.add(light);
  }

  for (let b = 0; b < model.nbody; b++) {
    //let parent_body = model.body_parentid()[b];
    if (b == 0 || !bodies[0]) {
      mujocoRoot.add(bodies[b]);
    } else if (bodies[b]) {
      bodies[0].add(bodies[b]);
    } else {
      console.log("Body without Geometry detected; adding to bodies", b, bodies[b]);
      bodies[b] = new THREE.Group();
      bodies[b].name = names[b + 1];
      bodies[b].bodyID = b;
      bodies[b].has_custom_mesh = false;
      bodies[0].add(bodies[b]);
    }
  }
  const updateProps = new UpdateProps(bodies, lights, cylinders, spheres)
  updateProps.setPointCloud(points, trajectory, T, N)
  return updateProps;
};

/**
 * Updates the Three.js scene based on the latest MuJoCo simulation state. This involves
 * updating the positions and orientations of bodies, lights, and tendons to reflect
 * the current state of the simulation.
 *
 * @param mujocoContainer The container holding the MuJoCo module and simulation.
 * @param updateProps An object containing references to bodies, lights, and tendon meshes.
 * @param tmpVec A temporary Vector3 used for calculations.
 * @returns This function does not return a value.
 * @throws Throws an error if the simulation fails to step.
 */
export const updateThreeScene = (
  mujocoContainer: MujocoContainer,
  updateProps: UpdateProps,
  tmpVec: THREE.Vector3
): void => {
  const simulation = mujocoContainer.getSimulation();
  const model = simulation.model();

  const bodies = updateProps.getBodies();
  const lights = updateProps.getLights();
  const cylinders = updateProps.getCylinders();
  const spheres = updateProps.getSpheres();

  // Update body transforms.
  for (let b = 0; b < model.nbody; b++) {
    if (bodies[b]) {
      getPosition(simulation.xpos, b, bodies[b].position);
      getQuaternion(simulation.xquat, b, bodies[b].quaternion);
      bodies[b].updateWorldMatrix(true, true);
    }
  }

  // Update light transforms.

  for (let l = 0; l < model.nlight; l++) {
    if (lights[l]) {
      getPosition(simulation.light_xpos, l, lights[l].position);
      getPosition(simulation.light_xdir, l, tmpVec);
      lights[l].lookAt(tmpVec.add(lights[l].position));
    }
  }

  // Update tendon transforms.

  let numWraps = 0;
  if (cylinders && spheres) {
    const mat = new THREE.Matrix4();
    for (let t = 0; t < model.ntendon; t++) {
      const startW = simulation.ten_wrapadr[t];
      const r = model.tendon_width[t];
      for (let w = startW; w < startW + simulation.ten_wrapnum[t] - 1; w++) {
        const tendonStart = getPosition(simulation.wrap_xpos, w, new THREE.Vector3());
        const tendonEnd = getPosition(simulation.wrap_xpos, w + 1, new THREE.Vector3());
        const tendonAvg = new THREE.Vector3()
          .addVectors(tendonStart, tendonEnd)
          .multiplyScalar(0.5);

        const validStart = tendonStart.length() > 0.01;
        const validEnd = tendonEnd.length() > 0.01;

        if (validStart) {
          spheres.setMatrixAt(
            numWraps,
            mat.compose(tendonStart, new THREE.Quaternion(), new THREE.Vector3(r, r, r))
          );
        }

        if (validEnd) {
          spheres.setMatrixAt(
            numWraps + 1,
            mat.compose(tendonEnd, new THREE.Quaternion(), new THREE.Vector3(r, r, r))
          );
        }

        if (validStart && validEnd) {
          mat.compose(
            tendonAvg,
            new THREE.Quaternion().setFromUnitVectors(
              new THREE.Vector3(0, 1, 0),
              tendonEnd.clone().sub(tendonStart).normalize()
            ),
            new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r)
          );
          cylinders.setMatrixAt(numWraps, mat);
          numWraps++;
        }
      }
    }
    cylinders.count = numWraps;
    spheres.count = numWraps > 0 ? numWraps + 1 : 0;
    cylinders.instanceMatrix.needsUpdate = true;
    spheres.instanceMatrix.needsUpdate = true;
  }

  simulation.step();
};
