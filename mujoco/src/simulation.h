#pragma once

#include <cstdio>
#include <cstring>
#include <string>

#include "mujoco/mujoco.h"

#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <emscripten/val.h>

#include <model.h>
#include <state.h>

#include <random>
#include <vector>
#include <iostream>

/**
 * @brief Manages the simulation by interfacing Model and State.
 *
 * The `Simulation` class facilitates interaction between a MuJoCo `Model` and
 * its corresponding `State`. It provides methods to apply forces and torques
 * to specific bodies within the simulation environment.
 * @param model The MuJoCo model associated with the simulation.
 * @param state The current simulation state.
 *
 * Usage Example:
 *
 * ```cpp
 * Model model = Model::load_from_xml("model.xml");
 * State state(&model);
 * Simulation sim(&model, &state);
 * sim.applyForce(0, 0, -9.81, 0, 0, 0, 0, 0, 0, 1);
 * sim.free();
 * ```
 */
std::vector<float> generateRandomPointCloud(int T, int N) {
    // 预分配空间
    std::vector<float> data;
    data.reserve(static_cast<size_t>(T) * N * 3);

    // 随机数引擎（用 std::random_device 做种子）
    static std::mt19937 rng{ std::random_device{}() };
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    // 按帧、按点填充 x,y,z
    for (int t = 0; t < T; ++t) {
        for (int i = 0; i < N; ++i) {
            data.push_back(dist(rng)); // x
            data.push_back(dist(rng)); // y
            data.push_back(dist(rng)); // z
        }
    }
    std::cout << "Create" << std::endl;
    std::cout << "T " << T << std::endl;
    std::cout << "N " << N << std::endl;
    for (int t = 0; t < T; ++t) {
    std::cout << "Frame " << t << ":" << std::endl;
    for (int i = 0; i < N; ++i) {
        int idx = (t * N + i) * 3;
        float x = data[idx];
        float y = data[idx + 1];
        float z = data[idx + 2];
        std::cout << "  Point " << i << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
    }
    }
    return data;
}

class Simulation {
public:
  Simulation(Model *m, State *s) {
    _model = m;
    _state = s;
  }

  State *state() { return _state; }
  Model *model() { return _model; }
  void free() {
    mju_free(_state);
    mju_free(_model);
  }

  void applyForce(mjtNum fx, mjtNum fy, mjtNum fz, mjtNum tx, mjtNum ty,
                  mjtNum tz, mjtNum px, mjtNum py, mjtNum pz, int body) {
    mjtNum force[3] = {fx, fy, fz};
    mjtNum torque[3] = {tx, ty, tz};
    mjtNum point[3] = {px, py, pz};
    mj_applyFT(_model->ptr(), _state->ptr(), force, torque, point, body,
               _state->ptr()->qfrc_applied);
  }

  // copied from the source of mjv_applyPerturbPose
  // sets perturb pos,quat in d->mocap when selected body is mocap, and in
  // d->qpos otherwise
  //  d->qpos written only if flg_paused and subtree root for selected body has
  //  free joint
  void applyPose(int bodyID, mjtNum refPosX, mjtNum refPosY, mjtNum refPosZ,
                 mjtNum refQuat1, mjtNum refQuat2, mjtNum refQuat3,
                 mjtNum refQuat4, int flg_paused) {
    int rootid = 0, sel = bodyID; // pert->select;
    mjtNum pos1[3], quat1[4], pos2[3], quat2[4], refpos[3], refquat[4];
    mjtNum *Rpos, *Rquat, *Cpos, *Cquat;
    mjtNum inrefpos[3] = {refPosX, refPosY, refPosZ};
    mjtNum inrefquat[4] = {refQuat1, refQuat2, refQuat3, refQuat4};
    mjModel *m = _model->ptr();
    mjData *d = _state->ptr();

    // exit if nothing to do
    // if (sel<=0 || sel>=m->nbody || !(pert->active | pert->active2)) { return;
    // }

    // get rootid above selected body
    rootid = m->body_rootid[sel];

    // transform refpos,refquat from I-frame to X-frame of body[sel]
    mju_negPose(pos1, quat1, m->body_ipos + 3 * sel, m->body_iquat + 4 * sel);
    mju_mulPose(refpos, refquat, inrefpos, inrefquat, pos1, quat1);

    // mocap body
    if (m->body_mocapid[sel] >= 0) {
      // copy ref pose into mocap pose
      mju_copy3(d->mocap_pos + 3 * m->body_mocapid[sel], refpos);
      mju_copy4(d->mocap_quat + 4 * m->body_mocapid[sel], refquat);
    }

    // floating body, paused
    else if (flg_paused && m->body_jntnum[sel] == 1 &&
             m->jnt_type[m->body_jntadr[sel]] == mjJNT_FREE) {
      // copy ref pose into qpos
      mju_copy3(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]], refpos);
      mju_copy4(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]] + 3, refquat);
    }

    // child of floating body, paused
    else if (flg_paused && m->body_jntnum[rootid] == 1 &&
             m->jnt_type[m->body_jntadr[rootid]] == mjJNT_FREE) {
      // get pointers to root
      Rpos = d->qpos + m->jnt_qposadr[m->body_jntadr[rootid]];
      Rquat = Rpos + 3;

      // get pointers to child
      Cpos = d->xpos + 3 * sel;
      Cquat = d->xquat + 4 * sel;

      // set root <- ref*neg(child)*root
      mju_negPose(pos1, quat1, Cpos, Cquat);              // neg(child)
      mju_mulPose(pos2, quat2, pos1, quat1, Rpos, Rquat); // neg(child)*root
      mju_mulPose(Rpos, Rquat, refpos, refquat, pos2,
                  quat2); // ref*neg(child)*root
    }
  }
  // function added

  // clang-format off

  // DEFINITIONS FROM MJDATA.H

  // State
  val qpos() const { return val(typed_memory_view(_model->ptr()->nq * 1 , _state->ptr()->qpos )); }
  val qvel() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qvel )); }
  val act() const { return val(typed_memory_view(_model->ptr()->na * 1 , _state->ptr()->act )); }
  val qacc_warmstart() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qacc_warmstart )); }
  val plugin_state() const { return val(typed_memory_view(_model->ptr()->npluginstate    * 1 , _state->ptr()->plugin_state )); }
  
  // Control
  val ctrl() const { return val(typed_memory_view(_model->ptr()->nu * 1 , _state->ptr()->ctrl )); }
  val qfrc_applied() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_applied )); }
  val xfrc_applied() const { return val(typed_memory_view(_model->ptr()->nbody * 6 , _state->ptr()->xfrc_applied )); }
  val eq_active() const { return val(typed_memory_view(_model->ptr()->neq * 1    , _state->ptr()->eq_active )); }  

  // mocap data
  val mocap_pos() const { return val(typed_memory_view(_model->ptr()->nmocap * 3 , _state->ptr()->mocap_pos )); }
  val mocap_quat() const { return val(typed_memory_view(_model->ptr()->nmocap * 4 , _state->ptr()->mocap_quat )); }
  
  // dynamics
  val qacc() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qacc )); }
  val act_dot() const { return val(typed_memory_view(_model->ptr()->na * 1 , _state->ptr()->act_dot )); }
  
  // User data
  val userdata() const { return val(typed_memory_view(_model->ptr()->nuserdata * 1 , _state->ptr()->userdata )); }
 
  // Sensor data
  val sensordata() const { return val(typed_memory_view(_model->ptr()->nsensordata * 1 , _state->ptr()->sensordata )); }
  
  // Plugins
  val plugin() const { return val(typed_memory_view(_model->ptr()->nplugin * 1 , _state->ptr()->plugin )); }
  val plugin_data() const { return val(typed_memory_view(_model->ptr()->nplugin * 1 , _state->ptr()->plugin_data )); }
  
  //-------------------- POSITION dependent

  // computed by mj_fwdPosition/mj_kinematics
  val xpos() const { return val(typed_memory_view(_model->ptr()->nbody * 3 , _state->ptr()->xpos )); }
  val xquat() const { return val(typed_memory_view(_model->ptr()->nbody * 4 , _state->ptr()->xquat )); }
  val xmat() const { return val(typed_memory_view(_model->ptr()->nbody * 9 , _state->ptr()->xmat )); }
  val xipos() const { return val(typed_memory_view(_model->ptr()->nbody * 3 , _state->ptr()->xipos )); }
  val ximat() const { return val(typed_memory_view(_model->ptr()->nbody * 9 , _state->ptr()->ximat )); }
  val xanchor() const { return val(typed_memory_view(_model->ptr()->njnt * 3 , _state->ptr()->xanchor )); }
  val xaxis() const { return val(typed_memory_view(_model->ptr()->njnt * 3 , _state->ptr()->xaxis )); }
  val geom_xpos() const { return val(typed_memory_view(_model->ptr()->ngeom * 3 , _state->ptr()->geom_xpos )); }
  val geom_xmat() const { return val(typed_memory_view(_model->ptr()->ngeom * 9 , _state->ptr()->geom_xmat )); }
  val site_xpos() const { return val(typed_memory_view(_model->ptr()->nsite * 3 , _state->ptr()->site_xpos )); }
  val site_xmat() const { return val(typed_memory_view(_model->ptr()->nsite * 9 , _state->ptr()->site_xmat )); }
  val cam_xpos() const { return val(typed_memory_view(_model->ptr()->ncam * 3 , _state->ptr()->cam_xpos )); }
  val cam_xmat() const { return val(typed_memory_view(_model->ptr()->ncam * 9 , _state->ptr()->cam_xmat )); }
  val light_xpos() const { return val(typed_memory_view(_model->ptr()->nlight * 3 , _state->ptr()->light_xpos )); }
  val light_xdir() const { return val(typed_memory_view(_model->ptr()->nlight * 3 , _state->ptr()->light_xdir )); }
   
  // computed by mj_fwdPosition/mj_comPos
  val subtree_com() const { return val(typed_memory_view(_model->ptr()->nbody * 3 , _state->ptr()->subtree_com )); }
  val cdof() const { return val(typed_memory_view(_model->ptr()->nv * 6 , _state->ptr()->cdof )); }
  val cinert() const { return val(typed_memory_view(_model->ptr()->nbody * 10 , _state->ptr()->cinert )); }
 
  // computed by mj_fwdPosition/mj_flex
  val flexvert_xpos() const { return val(typed_memory_view(_model->ptr()->nflexvert * 3, _state->ptr()->flexvert_xpos)); }    
  val flexelem_aabb() const { return val(typed_memory_view(_model->ptr()->nflexelem * 6, _state->ptr()->flexelem_aabb));} 
  val flexedge_J_rownnz() const { return val(typed_memory_view(_model->ptr()->nflexelem * 1, _state->ptr()->flexedge_J_rownnz));} 
  val flexedge_J_rowadr() const { return val(typed_memory_view(_model->ptr()->nflexelem * 1, _state->ptr()->flexedge_J_rowadr));} 
  val flexedge_J_colind() const { return val(typed_memory_view(_model->ptr()->nflexelem * _model->ptr()->nv, _state->ptr()->flexedge_J_colind));} 
  val flexedge_J() const { return val(typed_memory_view(_model->ptr()->nflexelem * _model->ptr()->nv, _state->ptr()->flexedge_J));} 
  val flexedge_length() const { return val(typed_memory_view(_model->ptr()->nflexelem * 1, _state->ptr()->flexedge_length));} 

  // computed by mj_fwdPosition/mj_tendon 
  val ten_wrapadr() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_wrapadr )); }
  val ten_wrapnum() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_wrapnum )); }
  val ten_J_rownnz() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_J_rownnz )); }
  val ten_J_rowadr() const { return val(typed_memory_view(_model->ptr()->ntendon * 1, _state->ptr()->ten_J_rowadr )); }
  val ten_J_colind() const { return val(typed_memory_view(_model->ptr()->ntendon * _model->ptr()->nv, _state->ptr()->ten_J_colind )); }
  val ten_J() const { return val(typed_memory_view(_model->ptr()->ntendon * _model->ptr()->nv, _state->ptr()->ten_J )); }
  val ten_length() const { return val(typed_memory_view(_model->ptr()->ntendon * 1 , _state->ptr()->ten_length )); }
  val wrap_obj() const { return val(typed_memory_view(_model->ptr()->nwrap * 2 , _state->ptr()->wrap_obj )); }
  val wrap_xpos() const { return val(typed_memory_view(_model->ptr()->nwrap * 6 , _state->ptr()->wrap_xpos )); }
  
  // computed by mj_fwdPosition/mj_transmission
  val actuator_length() const { return val(typed_memory_view(_model->ptr()->nu * 1 , _state->ptr()->actuator_length )); }
  val actuator_moment() const { return val(typed_memory_view(_model->ptr()->nu * _model->ptr()->nv, _state->ptr()->actuator_moment )); }
  
  // computed by mj_fwdPosition/mj_crb
  val crb() const { return val(typed_memory_view(_model->ptr()->nbody * 10 , _state->ptr()->crb )); }
  val qM() const { return val(typed_memory_view(_model->ptr()->nM * 1 , _state->ptr()->qM )); }
 
  // computed by mj_fwdPosition/mj_factorM
  val qLD() const { return val(typed_memory_view(_model->ptr()->nM * 1 , _state->ptr()->qLD )); }
  val qLDiagInv() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qLDiagInv )); }
  val qLDiagSqrtInv() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qLDiagSqrtInv )); }
  
  // computed by mj_collisionTree
  val bvh_aabb_dyn() const { return val(typed_memory_view(_model->ptr()->nbvhdynamic * 6 , _state->ptr()->bvh_aabb_dyn )); }
  val bvh_active() const { return val(typed_memory_view(_model->ptr()->nbvh * 1 , _state->ptr()->bvh_active )); }

  // computed by mj_fwdVelocity
  val flexedge_velocity() const { return val(typed_memory_view(_model->ptr()->nflexedge * 1 , _state->ptr()->flexedge_velocity )); }
  val ten_velocity() const { return val(typed_memory_view(_model->ptr()->ntendon * 1 , _state->ptr()->ten_velocity )); }
  val actuator_velocity() const { return val(typed_memory_view(_model->ptr()->nu * 1 , _state->ptr()->actuator_velocity )); }
 
  // computed by mj_fwdVelocity/mj_comVel
  val cvel() const { return val(typed_memory_view(_model->ptr()->nbody * 6 , _state->ptr()->cvel )); }
  val cdof_dot() const { return val(typed_memory_view(_model->ptr()->nv * 6 , _state->ptr()->cdof_dot )); }
  
  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  val qfrc_bias() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_bias )); }
  
  // computed by mj_fwdVelocity/mj_passive
  val qfrc_spring() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_spring )); }
  val qfrc_damper() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_damper )); }
  val qfrc_gravcomp() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_gravcomp )); }
  val qfrc_fluid() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_fluid )); }
  val qfrc_passive() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_passive )); }
  
  // computed by mj_sensorVel/mj_subtreeVel if needed
  val subtree_linvel() const { return val(typed_memory_view(_model->ptr()->nbody * 3 , _state->ptr()->subtree_linvel )); }
  val subtree_angmom() const { return val(typed_memory_view(_model->ptr()->nbody * 3 , _state->ptr()->subtree_angmom )); }
  
  // computed by mj_Euler or mj_implicit
  val qH() const { return val(typed_memory_view(_model->ptr()->nM * 1 , _state->ptr()->qH )); }
  val qHDiagInv() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qHDiagInv )); }
  
  // computed by mj_resetData
  val B_rownnz() const { return val(typed_memory_view(_model->ptr()->nbody * 1 , _state->ptr()->B_rownnz )); }
  val B_rowadr() const { return val(typed_memory_view(_model->ptr()->nbody * 1 , _state->ptr()->B_rowadr )); }
  val B_colind() const { return val(typed_memory_view(_model->ptr()->nB * 1 , _state->ptr()->B_colind )); }
  val C_rownnz() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->C_rownnz )); }
  val C_rowadr() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->C_rowadr )); }
  val C_colind() const { return val(typed_memory_view(_model->ptr()->nC * 1 , _state->ptr()->C_colind )); }
  val mapM2C() const { return val(typed_memory_view(_model->ptr()->nC * 1 , _state->ptr()->mapM2C )); }
  val D_rownnz() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->D_rownnz )); }
  val D_rowadr() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->D_rowadr )); }
  val D_colind() const { return val(typed_memory_view(_model->ptr()->nD * 1 , _state->ptr()->D_colind )); }
  val mapM2D() const { return val(typed_memory_view(_model->ptr()->nD * 1 , _state->ptr()->mapM2D )); }
  val mapD2M() const { return val(typed_memory_view(_model->ptr()->nM * 1 , _state->ptr()->mapD2M )); }

  // computed by mj_implicit/mj_derivative
  val qDeriv() const { return val(typed_memory_view(_model->ptr()->nD * 1 , _state->ptr()->qDeriv )); }
  
  // computed by mj_implicit/mju_factorLUSparse
  val qLU() const { return val(typed_memory_view(_model->ptr()->nD * 1 , _state->ptr()->qLU )); }
  
  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  val actuator_force() const { return val(typed_memory_view(_model->ptr()->nu * 1 , _state->ptr()->actuator_force )); }
  val qfrc_actuator() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_actuator )); }
  
  // computed by mj_fwdAcceleration
  val qfrc_smooth() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_smooth )); }
  val qacc_smooth() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qacc_smooth )); }
  
  // computed by mj_fwdAcceleration
  val qfrc_constraint() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_constraint )); }
   
  // computed by mj_inverse
  val qfrc_inverse() const { return val(typed_memory_view(_model->ptr()->nv * 1 , _state->ptr()->qfrc_inverse )); }
  
  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  val cacc() const { return val(typed_memory_view(_model->ptr()->nbody * 6 , _state->ptr()->cacc )); }
  val cfrc_int() const { return val(typed_memory_view(_model->ptr()->nbody * 6 , _state->ptr()->cfrc_int )); }
  val cfrc_ext() const { return val(typed_memory_view(_model->ptr()->nbody * 6 , _state->ptr()->cfrc_ext )); }
  
  // DEFINITIONS FROM XML API

  void   freeLastXML () { return mj_freeLastXML (); }

  // DEFINITIONS FROM ENGINE FORWARD API

  void step () { return mj_step (_model->ptr(), _state->ptr()); }
  void step1 () { return mj_step1 (_model->ptr(), _state->ptr()); }
  void step2 () { return mj_step2 (_model->ptr(), _state->ptr()); }
  void forward () { return mj_forward (_model->ptr(), _state->ptr()); }
  void forwardSkip (int skipstage, int skipsensor) { return mj_forwardSkip (_model->ptr(), _state->ptr(), skipstage, skipsensor); }
  void fwdPosition () { return mj_fwdPosition (_model->ptr(), _state->ptr()); }
  void fwdVelocity () { return mj_fwdVelocity (_model->ptr(), _state->ptr()); }
  void fwdActuation () { return mj_fwdActuation (_model->ptr(), _state->ptr()); }
  void fwdAcceleration () { return mj_fwdAcceleration (_model->ptr(), _state->ptr()); }
  void fwdConstraint () { return mj_fwdConstraint (_model->ptr(), _state->ptr()); }
  void Euler () { return mj_Euler (_model->ptr(), _state->ptr()); }
  void RungeKutta (int N ) { return mj_RungeKutta (_model->ptr(), _state->ptr(), N); }
  void checkPos () { return mj_checkPos (_model->ptr(), _state->ptr()); }
  void checkVel () { return mj_checkVel (_model->ptr(), _state->ptr()); }
  void checkAcc () { return mj_checkAcc (_model->ptr(), _state->ptr()); }

  // DEFINITIONS FROM ENGINE INVERSE API

  void inverse () { return mj_inverse (_model->ptr(), _state->ptr()); }
  void inverseSkip (int skipstage, int skipsensor) { return mj_inverseSkip (_model->ptr(), _state->ptr(), skipstage, skipsensor); }
  void invPosition () { return mj_invPosition (_model->ptr(), _state->ptr()); }
  void invVelocity () { return mj_invVelocity (_model->ptr(), _state->ptr()); }
  void invConstraint () { return mj_invConstraint (_model->ptr(), _state->ptr()); }
  void compareFwdInv () { return mj_compareFwdInv (_model->ptr(), _state->ptr()); }

  // DEFINITIONS FROM ENGINE IO API

  void defaultSolRefImp    (val solref, val solimp) { return mj_defaultSolRefImp (reinterpret_cast<mjtNum*>(solref["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(solimp["byteOffset"].as<int>())); }
  int sizeModel () { return mj_sizeModel (_model->ptr() ); }
  void resetData () { return mj_resetData (_model->ptr(), _state->ptr()); }
  void resetDataDebug (unsigned char debug_value) { return mj_resetDataDebug (_model->ptr(), _state->ptr(), debug_value); }
  void resetDataKeyframe   (int key ) { return mj_resetDataKeyframe (_model->ptr(), _state->ptr(), key); }
  void deleteData () { return mj_deleteData (_state->ptr() ); }

  // DEFINITIONS FROM ENGINE CALLBACK API

  void resetCallbacks () { return mj_resetCallbacks (); }

  // ENGINE PRINT API

  void printFormattedModel (std::string filename, std::string float_format) { return mj_printFormattedModel (_model->ptr(), filename.c_str(), float_format.c_str()); }
  void printModel (std::string filename) { return mj_printModel (_model->ptr(), filename.c_str()); }
  void printFormattedData  (std::string filename, std::string float_format) { return mj_printFormattedData (_model->ptr(), _state->ptr(), filename.c_str(), float_format.c_str()); }
  void printData (std::string filename) { return mj_printData (_model->ptr(), _state->ptr(), filename.c_str()); }
   
  // DEFINITIONS FROM ENGINE UTIL API

  void printMat (val mat, int nr, int nc) { return mju_printMat (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), nr, nc); }
   
  // DEFINITIONS FROM ENGINE SENSORS API

  void sensorPos () { return mj_sensorPos (_model->ptr(), _state->ptr()); }
  void sensorVel () { return mj_sensorVel (_model->ptr(), _state->ptr()); }
  void sensorAcc () { return mj_sensorAcc (_model->ptr(), _state->ptr()); }
  void energyPos () { return mj_energyPos (_model->ptr(), _state->ptr()); }
  void energyVel () { return mj_energyVel (_model->ptr(), _state->ptr()); }

  // DEFINITIONS FROM ENGINE CORE SMOOTH API

  void kinematics () { return mj_kinematics (_model->ptr(), _state->ptr()); }
  void comPos () { return mj_comPos (_model->ptr(), _state->ptr()); }
  void camlight () { return mj_camlight (_model->ptr(), _state->ptr()); }
  void tendon () { return mj_tendon (_model->ptr(), _state->ptr()); }
  void transmission () { return mj_transmission (_model->ptr(), _state->ptr()); }
  void crbCalculate () { return mj_crb (_model->ptr(), _state->ptr()); }
  void factorM () { return mj_factorM (_model->ptr(), _state->ptr()); }
  void solveM (val x, val y, int n ) { return mj_solveM (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(y["byteOffset"].as<int>()), n); }
  void solveM2 (val x, val y, int n ) { return mj_solveM2 (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(y["byteOffset"].as<int>()), n); }
  void comVel () { return mj_comVel (_model->ptr(), _state->ptr()); }
  void subtreeVel () { return mj_subtreeVel (_model->ptr(), _state->ptr()); }
  void rne (int flg_acc, val result) { return mj_rne (_model->ptr(), _state->ptr(), flg_acc, reinterpret_cast<mjtNum*>(result["byteOffset"].as<int>())); }
  void rnePostConstraint   () { return mj_rnePostConstraint (_model->ptr(), _state->ptr()); }

  // DEFINITIONS FROM ENGINE PASSIVE API

  void passive () { return mj_passive (_model->ptr(), _state->ptr()); }

  // DEFINITIONS FROM ENGINE COLLISION DRIVER API

  void collision () { return mj_collision (_model->ptr(), _state->ptr()); }

  // DEFINITIONS FROM ENGINE CORE CONSTRAINTS API

  void makeConstraint () { return mj_makeConstraint (_model->ptr(), _state->ptr()); }
  void projectConstraint   () { return mj_projectConstraint (_model->ptr(), _state->ptr()); }
  void referenceConstraint () { return mj_referenceConstraint (_model->ptr(), _state->ptr()); }
  int isPyramidal () { return mj_isPyramidal (_model->ptr() ); }
  int isSparse () { return mj_isSparse (_model->ptr() ); }
  int isDual () { return mj_isDual (_model->ptr() ); }
  void mulJacVec (val res, val vec    ) { return mj_mulJacVec (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>())); }
  void mulJacTVec (val res, val vec    ) { return mj_mulJacTVec (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>())); }
  
  // DEFINITIONS FROM ENGINE SUPPORT API

  void jacSubtreeCom (val jacp, int body  ) { return mj_jacSubtreeCom (_model->ptr(), _state->ptr(), reinterpret_cast<mjtNum*>(jacp["byteOffset"].as<int>()), body); }
  void differentiatePos    (val qvel, mjtNum dt, val qpos1, val qpos2) { return mj_differentiatePos (_model->ptr(), reinterpret_cast<mjtNum*>(qvel["byteOffset"].as<int>()), dt, reinterpret_cast<mjtNum*>(qpos1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(qpos2["byteOffset"].as<int>())); }
  void integratePos (val qpos, val qvel, mjtNum dt) { return mj_integratePos (_model->ptr(), reinterpret_cast<mjtNum*>(qpos["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(qvel["byteOffset"].as<int>()), dt); }
  void normalizeQuat (val qpos ) { return mj_normalizeQuat (_model->ptr(), reinterpret_cast<mjtNum*>(qpos["byteOffset"].as<int>())); }
  mjtNum getTotalmass () { return mj_getTotalmass (_model->ptr() ); }
  void fullM (val dst, val M ) { return mj_fullM (_model->ptr(), reinterpret_cast<mjtNum*>(dst["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(M["byteOffset"].as<int>())); }
  int version () { return mj_version (); }
  std::string versionString () { return std::string(mj_versionString ()); }

  // DEFINITIONS FROM ENGINE NAME API

  int name2id (int type, std::string name) { return mj_name2id (_model->ptr(), type, name.c_str()); }
  std::string id2name (int type, int id    ) { return std::string(mj_id2name (_model->ptr(), type, id)); }

  // DEFINITIONS FROM ENGINE PLUGIN API

  std::string getPluginConfig (int plugin_id, std::string attrib) { return std::string(mj_getPluginConfig (_model->ptr(), plugin_id, attrib.c_str())); }
  void loadPluginLibrary   (std::string path    ) { return mj_loadPluginLibrary (path.c_str() ); }
  
  // DEFINITIONS FROM RENDER API

  void rectangle (mjrRect viewport, float r, float g, float b, float a) { return mjr_rectangle (viewport, r, g, b, a); }
  void finish () { return mjr_finish (); }
  int getError () { return mjr_getError (); }

  // DEFINITIONS FROM UI API

  mjuiThemeSpacing themeSpacing (int ind ) { return mjui_themeSpacing (ind ); }
  mjuiThemeColor themeColor (int ind ) { return mjui_themeColor (ind ); }
  
  // DEFINITIONS FROM ENGINE ERROR AND MEMORY API

  void error (std::string msg ) { return mju_error (msg.c_str() ); }
  void error_i (std::string msg, int i) { return mju_error_i (msg.c_str(), i ); }
  void error_s (std::string msg, std::string text) { return mju_error_s (msg.c_str(), text.c_str()); }
  void warning (std::string msg ) { return mju_warning (msg.c_str() ); }
  void warning_i (std::string msg, int i) { return mju_warning_i (msg.c_str(), i ); }
  void warning_s (std::string msg, std::string text) { return mju_warning_s (msg.c_str(), text.c_str()); }
  void clearHandlers () { return mju_clearHandlers (); }
  void writeLog (std::string type, std::string msg) { return mju_writeLog (type.c_str(), msg.c_str()); }

  // DEFINITIONS MUJOCO API

  void zero (val res, int n ) { return mju_zero (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  void fill (val res, mjtNum val, int n) { return mju_fill (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), val, n); }
  void copy (val res, val data, int n) { return mju_copy (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(data["byteOffset"].as<int>()), n); }
  mjtNum sum (val vec, int n ) { return mju_sum (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum L1 (val vec, int n ) { return mju_L1 (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void scl (val res, val vec, mjtNum scl, int n) { return mju_scl (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), scl, n); }
  void add (val res, val vec1, val vec2, int n) { return mju_add (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void sub (val res, val vec1, val vec2, int n) { return mju_sub (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void addTo (val res, val vec, int n) { return mju_addTo (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void subFrom (val res, val vec, int n) { return mju_subFrom (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  void addToScl (val res, val vec, mjtNum scl, int n) { return mju_addToScl (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), scl, n); }
  void addScl (val res, val vec1, val vec2, mjtNum scl, int n) { return mju_addScl (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), scl, n); }
  mjtNum normalize (val res, int n ) { return mju_normalize (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  mjtNum norm (val res, int n ) { return mju_norm (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), n); }
  mjtNum dot (val vec1, val vec2, int n) { return mju_dot (reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void mulMatVec (val res, val mat, val vec, int nr, int nc) { return mju_mulMatVec (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), nr, nc); }
  void mulMatTVec (val res, val mat, val vec, int nr, int nc) { return mju_mulMatTVec (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), nr, nc); }
  mjtNum mulVecMatVec (val vec1, val mat, val vec2, int n) { return mju_mulVecMatVec (reinterpret_cast<mjtNum*>(vec1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec2["byteOffset"].as<int>()), n); }
  void transpose (val res, val mat, int nr, int nc) { return mju_transpose (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), nr, nc); }
  void symmetrize (val res, val mat, int n) { return mju_symmetrize (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n); }
  void eye (val mat, int n ) { return mju_eye (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n); }
  void mulMatMat (val res, val mat1, val mat2, int r1, int c1, int c2) { return mju_mulMatMat (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, c2); }
  void mulMatMatT (val res, val mat1, val mat2, int r1, int c1, int r2) { return mju_mulMatMatT (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, r2); }
  void mulMatTMat (val res, val mat1, val mat2, int r1, int c1, int c2) { return mju_mulMatTMat (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat1["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat2["byteOffset"].as<int>()), r1, c1, c2); }
  void sqrMatTD (val res, val mat, val diag, int nr, int nc) { return mju_sqrMatTD (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(diag["byteOffset"].as<int>()), nr, nc); }
  int  cholFactor (val mat, int n, mjtNum mindiag) { return mju_cholFactor (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), n, mindiag); }
  void cholSolve (val res, val mat, val vec, int n) { return mju_cholSolve (reinterpret_cast<mjtNum*>(res["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  int  cholUpdate (val mat, val x, int n, int flg_plus) { return mju_cholUpdate (reinterpret_cast<mjtNum*>(mat["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(x["byteOffset"].as<int>()), n, flg_plus); }
  void encodePyramid (val pyramid, val force, val mu, int dim) { return mju_encodePyramid (reinterpret_cast<mjtNum*>(pyramid["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(force["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mu["byteOffset"].as<int>()), dim); }
  void decodePyramid (val force, val pyramid, val mu, int dim) { return mju_decodePyramid (reinterpret_cast<mjtNum*>(force["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(pyramid["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(mu["byteOffset"].as<int>()), dim); }
  mjtNum springDamper (mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt) { return mju_springDamper (pos0, vel0, Kp, Kv, dt); }
  mjtNum min (mjtNum a, mjtNum b  ) { return mju_min (a, b ); }
  mjtNum max (mjtNum a, mjtNum b  ) { return mju_max (a, b ); }
  mjtNum clip (mjtNum x, mjtNum min, mjtNum max) { return mju_clip (x, min, max ); }
  mjtNum sign (mjtNum x ) { return mju_sign (x ); }
  int round (mjtNum x ) { return mju_round (x ); }
  std::string type2Str (int type ) { return std::string(mju_type2Str (type )); }
  int str2Type (std::string str ) { return mju_str2Type (str.c_str() ); }
  std::string writeNumBytes (size_t nbytes ) { return std::string(mju_writeNumBytes (nbytes )); }
  std::string warningText (int warning, size_t info) { return std::string(mju_warningText (warning, info )); }
  int isBad (mjtNum x ) { return mju_isBad (x ); }
  int isZero (val vec, int n ) { return mju_isZero (reinterpret_cast<mjtNum*>(vec["byteOffset"].as<int>()), n); }
  mjtNum standardNormal (val num2 ) { return mju_standardNormal (reinterpret_cast<mjtNum*>(num2["byteOffset"].as<int>())); }
  void insertionSort (val list, int n ) { return mju_insertionSort (reinterpret_cast<mjtNum*>(list["byteOffset"].as<int>()), n); }
  mjtNum Halton (int index, int base ) { return mju_Halton (index, base ); }
  mjtNum sigmoid (mjtNum x ) { return mju_sigmoid (x ); }
  void transitionFD (mjtNum eps, mjtByte centered, val A, val B, val C, val D) { return mjd_transitionFD (_model->ptr(), _state->ptr(), eps, centered, reinterpret_cast<mjtNum*>(A["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(B["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(C["byteOffset"].as<int>()), reinterpret_cast<mjtNum*>(D["byteOffset"].as<int>())); }
  int pluginCount () { return mjp_pluginCount (); }

  // clang-format on

private:
  Model *_model;
  State *_state;
};
