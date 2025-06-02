// 在 main.cpp 最顶部，现有的 includes 之后，追加下面内容：

#pragma once

#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <emscripten/val.h>

#include "mujoco/mujoco.h"

#include <iostream>
#include <vector>
#include <array>
#include <cmath>            // 用到 cos(), sin() 之类函数
#include <emscripten/emscripten.h>  // EMSCRIPTEN_KEEPALIVE
using namespace emscripten;


// ----------------------------------------------------------------------------
// 全局目标序列容器：一个二维数组
// g_target_sequence[t] 就是第 t 帧，会存若干个三维点 (x,y,z)
// ----------------------------------------------------------------------------
static std::vector< std::vector< std::array<double,3> > > g_target_sequence;
extern "C" {

// --------------------------------------------------------------------------------
// generate_target_sequence(int num_steps):
//   在后端创建一个由 num_steps 帧组成的“点云序列”。
//   每一帧我们随便生成 10 个在 XY 平面围绕原点转一圈的点，
//   只是示范如何填充 g_target_sequence。
// --------------------------------------------------------------------------------
EMSCRIPTEN_KEEPALIVE
void generate_target_sequence(int num_steps) {
  // 清空旧数据
  g_target_sequence.clear();

  // 如果 num_steps <= 0，就不生成
  if (num_steps <= 0) { 
    return;
  }

  // 预分配一级 vector 空间
  g_target_sequence.reserve(num_steps);

  // 举例：让每一帧上都画 10 个点，点在 XY 平面上以正方形为轨迹，z 全部为 0
  const int points_per_frame = 10;
  for (int t = 0; t < num_steps; ++t) {
    // 本帧的点容器
    std::vector<std::array<double,3>> frame_pts;
    frame_pts.reserve(points_per_frame);

    // 这里我们给每个点一个 “角度偏移 = t * 0.1 + j * 2π/points_per_frame”，
    // x = cos(angle), y = sin(angle), z = 0
    for (int j = 0; j < points_per_frame; ++j) {
      double angle = (double)j * (2.0 * M_PI / points_per_frame) + (double)t * 0.1;
      double x = std::cos(angle);
      double y = std::sin(angle);
      double z = 0.0;
      frame_pts.push_back({ x, y, z });
    }

    // 保存到全局序列
    g_target_sequence.push_back(std::move(frame_pts));
  }
}

// --------------------------------------------------------------------------------
// get_sequence_length():
//   返回 g_target_sequence 的长度（也就是 T）。
// --------------------------------------------------------------------------------
EMSCRIPTEN_KEEPALIVE
int get_sequence_length() {
  return (int)g_target_sequence.size();
}

}  // extern "C"
