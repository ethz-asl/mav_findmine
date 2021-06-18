/*
MIT License

Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland

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

#include <Eigen/Geometry>

using namespace Eigen;

TEST(TfTest, Rotation) {
  // 90deg yaw.
  Quaterniond q_WB;
  q_WB = AngleAxisd(90 * M_PI / 180.0, Vector3d::UnitZ());
  // 3 m/s**2 acceleration in x direction in body frame.
  Vector3d a_B = Vector3d(3.0, 0.0, 0.0);
  // Expected acceleration in world frame is 3 m/s**2 in y direction.
  Vector3d a_W = Vector3d(0.0, 3.0, 0.0);
  EXPECT_TRUE(a_W.isApprox(q_WB.toRotationMatrix() * a_B, 1.0e-9));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
