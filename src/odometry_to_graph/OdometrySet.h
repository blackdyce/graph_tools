#pragma once

#include "OdometryFrame.h"

namespace odometry_to_graph
{
class OdometrySet {
public:
  OdometrySet(const float resolution, const float delta_x, const float delta_angle, const float inf_pos, const float inf_rot);

  void load(const std::string& directory);

  bool save(const std::string& directory);

private:
  bool save_graph(const std::string& filename) const;

  bool save_keyframes(const std::string& directory) const;

private:
  const float resolution;
  const float delta_x;
  const float delta_angle;
  const float inf_pos;
  const float inf_rot;

  std::vector<OdometryFrame::Ptr> keyframes;
};
}