// EleSim binding; upstream algorithm and MIT notices remain under upstream/.
// One translation unit avoids upstream non-inline header definition duplication.
#include "see_core/src/see_core.cpp"
#include <cmath>
#include <stdexcept>

using ori::see::core::SeeCore;

// Input: per-observation XYZ in one fixed frame, counts, sensor XYZ + optical Z.
// Output rows: camera XYZ, direction XYZ, associated frontier XYZ.
extern "C" int elesim_see_proposals(const float* points, const int* counts,
    const float* views, int frames, int width, int height, float fov_x,
    float fov_y, float radius, int density, float distance,
    float* output, int capacity) noexcept {
  try {
    if (frames < 1 || frames > 24 || capacity < 1 || capacity > 4096 ||
        width < 1 || height < 1 || !std::isfinite(radius) || radius <= 0 ||
        !std::isfinite(distance) || distance <= 0 || density < 1)
      return -1;
    omp_set_num_threads(1); // Deterministic classification; no control-thread starvation.
    SensorParams sensor{};
    sensor.pix_x=width; sensor.pix_y=height;
    sensor.fov_x=fov_x; sensor.fov_y=fov_y;
    sensor.f_axis=2; sensor.fps=1; sensor.world_frame="scan";
    SeeParams params{};
    params.r=radius; params.rho=density; params.d=distance;
    params.tau=100; params.psi=distance; params.ups=radius;
    SeeCore core(params, sensor);
    if (core.IsDone()) return -2;
    int offset=0;
    for (int frame=0; frame<frames; ++frame) {
      if (counts[frame]<3 || counts[frame]>20000) return -1;
      SeePointCloudPtr cloud(new SeePointCloud);
      cloud->resize(counts[frame]);
      SeeView view{}; view.data[3]=1.f;
      for (int j=0;j<3;++j) {
        view.getVector3fMap()[j]=views[frame*6+j];
        view.getViewVector3fMap()[j]=views[frame*6+3+j];
      }
      for (int i=0;i<counts[frame];++i) {
        auto &p=cloud->points[i]; p=SeePoint{}; p.data[3]=1.f;
        for(int j=0;j<3;++j) {
          float value=points[3*(offset+i)+j];
          if(!std::isfinite(value)) return -1;
          p.getVector3fMap()[j]=value;
        }
      }
      core.UpdatePointCloud(cloud,view);
      offset+=counts[frame];
      // Frontier exhaustion is explicitly an empty proposal result, not success.
      if(core.IsDone()) return 0;
    }
    auto cloud=core.GetPointCloud(); auto proposals=core.GetViewCloud();
    int count=0;
    for(size_t i=0;i<cloud->size();++i) {
      if(cloud->points[i].label!=frontier_pt) continue;
      if(count==capacity) return -3; // Never silently discard the tail of proposals.
      const auto &v=proposals->points[i]; const auto &p=cloud->points[i];
      for(int j=0;j<3;++j) {
        output[count*9+j]=v.getVector3fMap()[j];
        output[count*9+3+j]=v.getViewVector3fMap()[j];
        output[count*9+6+j]=p.getVector3fMap()[j];
      }
      ++count;
    }
    return count;
  } catch (...) { return -4; }
}
