// camera.h
#pragma once
#include <mujoco/mujoco.h>
#include <emscripten/bind.h>

/**
 * 简单封装 mjvCamera，只管距离缩放。
 * ● setZoom(d)   —— 把相机距离设为绝对值 d（米）
 * ● zoom(f)      —— 按倍数缩放，f<1 放大，f>1 缩小
 * ● move(act,dx,dy,model,scn) —— 调用官方 mjv_moveCamera，
 *     act 对应 mjMOUSE_*，dx/dy 是像素位移
 */
class Camera {
public:
  Camera() { mjv_defaultCamera(&cam_); }

  // 绝对距离
  void setZoom(float dist)           { cam_.distance = dist; }
  float getZoom()              const { return cam_.distance; }

  // 相对缩放
  void zoom(float factor)            { cam_.distance *= factor; }

  // 完整鼠标操作（可选）
  void move(int action, float dx, float dy,
            mjModel* m, mjvScene* scn) {
    mjv_moveCamera(m, static_cast<mjtMouse>(action),
                   dx, dy, scn, &cam_);
  }

  // 供 JS 侧拿指针
  mjvCamera* ptr()                   { return &cam_; }

private:
  mjvCamera cam_;
};
