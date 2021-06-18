/*
MIT License

Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <gtest/gtest.h>
#include <mav_trajectory_generation/vertex.h>

#include "fm_trajectories/circle_trajectory.h"

using namespace fm_trajectories;
using namespace mav_trajectory_generation;

TEST(CircleTest, Basic) {
  const double kCenterX = 0.0;
  const double kCenterY = 0.0;
  const double kAltitude = 0.0;
  const double kRadius = 1.0;
  const double kVelocity = 1.0;
  const double kStartHeading = 0.0;
  const double kArcLength = 360.0;
  const double kOffsetHeading = 0.0;

  auto settings = std::make_shared<CircleTrajectory::Settings>(
      kCenterX, kCenterY, kRadius, kStartHeading, kArcLength, kAltitude,
      kVelocity, kOffsetHeading, std::weak_ptr<BaseTrajectory>(),
      InputConstraints());
  CircleTrajectory circle(settings);
  Trajectory trajectory;
  EXPECT_TRUE(circle.planTrajectory(&trajectory));
  Vertex::Vector computed_position_vertices, computed_yaw_vertices;
  std::vector<double> computed_segment_times;
  circle.getVertices(&computed_position_vertices, &computed_yaw_vertices,
                     &computed_segment_times);

  EXPECT_GE(computed_position_vertices.size(), 2);
  EXPECT_GE(computed_segment_times.size(), 1);
  EXPECT_EQ(computed_position_vertices.size(), computed_yaw_vertices.size());
  EXPECT_EQ(computed_segment_times.size() + 1,
            computed_position_vertices.size());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
