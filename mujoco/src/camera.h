// camera.h
#pragma once
#include <mujoco/mujoco.h>
#include <emscripten/bind.h>

class Camera {
public:
  Camera() { mjv_defaultCamera(&cam_); }

  void setZoom(float dist)           { cam_.distance = dist; }
  float getZoom()              const { return cam_.distance; }

  void zoom(float factor)            { cam_.distance *= factor; }

  void move(int action, float dx, float dy,
            mjModel* m, mjvScene* scn) {
    mjv_moveCamera(m, static_cast<mjtMouse>(action),
                   dx, dy, scn, &cam_);
  }

  mjvCamera* ptr()                   { return &cam_; }

private:
  mjvCamera cam_;
};
