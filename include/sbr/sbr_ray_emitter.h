// sbr_ray_emitter.h — 射线发射器 (Phase 1 实现)
// TODO: Phase 1 — Fibonacci Sphere / 方向图重要性采样
#pragma once
#include "sbr_types.h"
#include <vector>

namespace sbr {

std::vector<Vec3> GenerateFibonacciRays(int N);

} // namespace sbr
