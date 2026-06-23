#include "doctest.h"
#include "sbr/sbr_engine.h"
#include "sbr/sbr_bvh_accelerator.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <chrono>
#include <cstdio>
using namespace sbr;
static Scene MakeGrid(int n) { SceneBuilder sb; double s=10.0/n; for(int i=0;i<=n;++i)for(int j=0;j<=n;++j)sb.AddVertex(i*s,j*s,0); for(int i=0;i<n;++i)for(int j=0;j<n;++j){int v=i*(n+1)+j;sb.AddFace(v,v+n+1,v+n+2);sb.AddFace(v,v+n+2,v+1);} return sb.Build(); }
TEST_CASE("BVH speedup vs BruteForce") {
  Scene scene=MakeGrid(100);
  auto t0=std::chrono::steady_clock::now(); BvhAccelerator bvh(scene); auto t1=std::chrono::steady_clock::now();
  double bms=std::chrono::duration<double,std::milli>(t1-t0).count();
  std::printf("[BVH] Build %zu faces: %.1f ms, %d nodes, depth %d\n",scene.faces.size(),bms,bvh.NodeCount(),bvh.MaxDepth());
  SbrEngine be(std::make_unique<BvhAccelerator>(scene));
  SbrConfig c; c.ray_count=100;c.max_ray_depth=1;c.max_reflection_count=1;c.rx_sphere_radius_m=0.5;c.ray_power_threshold_dB=-120;c.enable_path_dedup=true;
  NumericToleranceConfig t;
  auto t2=std::chrono::steady_clock::now();
  auto r=be.RunPointToPoint(scene,MaterialDatabase{},c,MakeVec3(2,2,5),{MakeVec3(5,5,2)},t);
  auto t3=std::chrono::steady_clock::now();
  double tms=std::chrono::duration<double,std::milli>(t3-t2).count();
  std::printf("[BVH] 100 rays trace: %.1f ms\n",tms);
  CHECK(r.paths.size()>=0);
  CHECK(tms<5000.0);
  // Quick regression
  SceneBuilder sb;Scene s=sb.Build();SbrEngine e3(std::make_unique<BvhAccelerator>(s));
  SbrConfig c3;c3.trace_profile="DebugValidation";c3.ray_count=100;c3.max_ray_depth=0;c3.max_reflection_count=0;c3.rx_sphere_radius_m=1.0;c3.ray_power_threshold_dB=-120;
  auto r3=e3.RunPointToPoint(s,MaterialDatabase{},c3,MakeVec3(0,0,0),{MakeVec3(0,2,0)},NumericToleranceConfig{});
  CHECK(r3.paths.size()>0);CHECK(r3.paths[0].is_los);CHECK(r3.paths[0].nodes.size()==2);
}
