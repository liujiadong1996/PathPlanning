#include "common/math/vec2d.hpp"

#include <cmath>
#include <gtest/gtest.h>

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST(Vec2dTest, NomralCases) {
  common::math::Vec2d pt(2, 3);
  EXPECT_NEAR(pt.Length(), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(pt.LengthSquare(), 13.0, 1e-5);
  EXPECT_NEAR(pt.DistanceTo({0, 0}), std::sqrt(13.0), 1e-5);
  EXPECT_NEAR(pt.DistanceSquareTo({0, 0}), 13.0, 1e-5);
  EXPECT_NEAR(pt.DistanceTo({0, 2}), std::sqrt(5.0), 1e-5);
  EXPECT_NEAR(pt.DistanceSquareTo({0, 2}), 5.0, 1e-5);
  EXPECT_NEAR(pt.Angle(), std::atan2(3, 2), 1e-5);
  EXPECT_NEAR(pt.CrossProd({4, 5}), -2, 1e-5);
  EXPECT_NEAR(pt.InnerProd({4, 5}), 23, 1e-5);
  EXPECT_EQ(pt.DebugString(), "vec2d ( x = 2  y = 3 )");
  pt.set_x(4);
  pt.set_y(5);
  EXPECT_NEAR(pt.Length(), std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt.LengthSquare(), 41.0, 1e-5);
  pt.Normalize();
  EXPECT_NEAR(pt.x(), 4.0 / std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt.y(), 5.0 / std::sqrt(41.0), 1e-5);
  EXPECT_NEAR(pt.Length(), 1.0, 1e-5);

  const common::math::Vec2d d =
      common::math::Vec2d(0.5, 1.5) + common::math::Vec2d(2.5, 3.5);
  EXPECT_NEAR(d.x(), 3.0, 1e-5);
  EXPECT_NEAR(d.y(), 5.0, 1e-5);
  const common::math::Vec2d e =
      common::math::Vec2d(0.5, 1.5) - common::math::Vec2d(2.5, 3.5);
  EXPECT_NEAR(e.x(), -2.0, 1e-5);
  EXPECT_NEAR(e.y(), -2.0, 1e-5);
  const common::math::Vec2d f = d / 2.0;
  EXPECT_NEAR(f.x(), 1.5, 1e-5);
  EXPECT_NEAR(f.y(), 2.5, 1e-5);
  const common::math::Vec2d g = e * (-3.0);
  EXPECT_NEAR(g.x(), 6.0, 1e-5);
  EXPECT_NEAR(g.y(), 6.0, 1e-5);

  const common::math::Vec2d unit_pt =
      common::math::Vec2d::CreateUnitVec2d(M_PI_4);
  EXPECT_NEAR(unit_pt.x(), std::sqrt(2.0) / 2.0, 1e-5);
  EXPECT_NEAR(unit_pt.y(), std::sqrt(2.0) / 2.0, 1e-5);
  EXPECT_NEAR(unit_pt.Angle(), M_PI_4, 1e-5);
}

TEST(Vec2dTest, rotate) {
  common::math::Vec2d pt(4, 0);
  auto p1 = pt.rotate(M_PI / 2.0);
  EXPECT_NEAR(p1.x(), 0.0, 1e-5);
  EXPECT_NEAR(p1.y(), 4.0, 1e-5);
  auto p2 = pt.rotate(M_PI);
  EXPECT_NEAR(p2.x(), -4.0, 1e-5);
  EXPECT_NEAR(p2.y(), 0.0, 1e-5);
  auto p3 = pt.rotate(-M_PI / 2.0);
  EXPECT_NEAR(p3.x(), 0.0, 1e-5);
  EXPECT_NEAR(p3.y(), -4.0, 1e-5);
  auto p4 = pt.rotate(-M_PI);
  EXPECT_NEAR(p4.x(), -4.0, 1e-5);
  EXPECT_NEAR(p4.y(), 0.0, 1e-5);
}

TEST(Vec2dTest, selfrotate) {
  common::math::Vec2d p1(4, 0);
  p1.SelfRotate(M_PI / 2.0);
  EXPECT_NEAR(p1.x(), 0.0, 1e-5);
  EXPECT_NEAR(p1.y(), 4.0, 1e-5);
  common::math::Vec2d p2(4, 0);
  p2.SelfRotate(M_PI);
  EXPECT_NEAR(p2.x(), -4.0, 1e-5);
  EXPECT_NEAR(p2.y(), 0.0, 1e-5);
  common::math::Vec2d p3(4, 0);
  p3.SelfRotate(-M_PI / 2.0);
  EXPECT_NEAR(p3.x(), 0.0, 1e-5);
  EXPECT_NEAR(p3.y(), -4.0, 1e-5);
  common::math::Vec2d p4(4, 0);
  p4.SelfRotate(-M_PI);
  EXPECT_NEAR(p4.x(), -4.0, 1e-5);
  EXPECT_NEAR(p4.y(), 0.0, 1e-5);
}