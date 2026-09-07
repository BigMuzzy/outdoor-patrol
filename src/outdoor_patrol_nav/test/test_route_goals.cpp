// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// The route reader and the station subsampler, without a ROS graph.
//
// The rejections matter more than the happy path. A route file that parses
// but is wrong -- one schema version off, recorded at the antenna, three
// samples long -- would be driven, and the failure would show up as the robot
// tracking a line 0.42 m to the right of the road. Each rejection here has a
// counterpart in outdoor_patrol_route/test/test_route_file.py; if one side
// changes and the other does not, the same file is valid for the recorder and
// invalid for the mission.

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "outdoor_patrol_nav/route_goals.hpp"

using outdoor_patrol_nav::parse_route;
using outdoor_patrol_nav::read_route;
using outdoor_patrol_nav::subsample_indices;

namespace
{

/// A minimal valid route: four samples is the floor route_file.py enforces.
std::string minimal_route()
{
  return std::string(
    "version: 1\n"
    "source: odometry_global\n"
    "loop: false\n"
    "datum: {latitude: -41.28646, longitude: 174.776236, altitude: 10.0}\n"
    "samples:\n"
    "  - {lat: -41.286460, lon: 174.776236, yaw: 0.0}\n"
    "  - {lat: -41.286460, lon: 174.776356, yaw: 0.0}\n"
    "  - {lat: -41.286460, lon: 174.776475, yaw: 0.1}\n"
    "  - {lat: -41.286370, lon: 174.776475, yaw: 1.5}\n");
}

/// A straight polyline of `count` points spaced 1 m apart along +x.
std::vector<std::array<double, 2>> straight_line(std::size_t count)
{
  std::vector<std::array<double, 2>> xy;
  for (std::size_t i = 0; i < count; ++i) {
    xy.push_back({static_cast<double>(i), 0.0});
  }
  return xy;
}

}  // namespace

TEST(ReadRoute, ReadsTheFixture)
{
  const auto route = read_route(std::string(FIXTURE_DIR) + "/route_square.yaml");
  EXPECT_EQ(route.samples.size(), 8u);
  EXPECT_TRUE(route.loop);
  EXPECT_EQ(route.source, "odometry_global");
  EXPECT_TRUE(route.is_base_link());
}

TEST(ReadRoute, YawIsPassedThroughUntouched)
{
  // Not renormalised, not unwrapped: the recorder already wrote REP-103, and
  // the negative yaws on the fixture's west side must survive as negatives.
  const auto route = read_route(std::string(FIXTURE_DIR) + "/route_square.yaml");
  EXPECT_NEAR(route.samples[0].yaw, 0.0, 1e-9);
  EXPECT_NEAR(route.samples[2].yaw, 1.570796, 1e-9);
  EXPECT_NEAR(route.samples[6].yaw, -1.570796, 1e-9);
}

TEST(ReadRoute, MissingFileNamesThePath)
{
  EXPECT_THROW(read_route("/nonexistent/route.yaml"), std::runtime_error);
}

TEST(ParseRoute, AcceptsAMinimalRoute)
{
  const auto route = parse_route(minimal_route());
  EXPECT_EQ(route.samples.size(), 4u);
  EXPECT_FALSE(route.loop);
}

TEST(ParseRoute, RejectsAnotherSchemaVersion)
{
  // Version 2 adds safe_spots (plan Phase 4). Reading one with this reader
  // would silently drop them, so it is refused outright.
  auto text = minimal_route();
  text.replace(text.find("version: 1"), 10, "version: 2");
  EXPECT_THROW(parse_route(text), std::runtime_error);
}

TEST(ParseRoute, RejectsAMissingDatum)
{
  auto text = minimal_route();
  const auto start = text.find("datum:");
  text.erase(start, text.find('\n', start) - start + 1);
  EXPECT_THROW(parse_route(text), std::runtime_error);
}

TEST(ParseRoute, RejectsTooFewSamples)
{
  auto text = minimal_route();
  const auto start = text.rfind("  - {");
  text.erase(start);
  EXPECT_THROW(parse_route(text), std::runtime_error);
}

TEST(ParseRoute, RejectsANonNumericCoordinate)
{
  auto text = minimal_route();
  text.replace(text.find("lat: -41.286460"), 15, "lat: not_a_number");
  EXPECT_THROW(parse_route(text), std::runtime_error);
}

TEST(ParseRoute, RejectsAMissingYaw)
{
  auto text = minimal_route();
  text.replace(text.find(", yaw: 0.0}"), 11, "}");
  EXPECT_THROW(parse_route(text), std::runtime_error);
}

TEST(ParseRoute, RejectsTextThatIsNotAMapping)
{
  EXPECT_THROW(parse_route("- one\n- two\n"), std::runtime_error);
}

TEST(ParseRoute, AnAntennaRouteParsesButIsNotBaseLink)
{
  // The file is well-formed; it is the *pose* that is wrong, 0.42 m right of
  // base_link. parse_route is not the place to refuse it -- patrol_mission is,
  // with an error a human can act on -- but is_base_link() has to say so.
  auto text = minimal_route();
  text.replace(text.find("source: odometry_global"), 23, "source: raw_antenna");
  const auto route = parse_route(text);
  EXPECT_EQ(route.samples.size(), 4u);
  EXPECT_FALSE(route.is_base_link());
}

TEST(ParseRoute, DefaultsSourceToOdometryGlobal)
{
  auto text = minimal_route();
  const auto start = text.find("source:");
  text.erase(start, text.find('\n', start) - start + 1);
  EXPECT_TRUE(parse_route(text).is_base_link());
}

TEST(Subsample, TakesEveryNthMetreAndKeepsTheEnd)
{
  // 41 points, 1 m apart, 10 m spacing: stations at 0, 10, 20, 30, 40.
  const auto indices = subsample_indices(straight_line(41), 10.0, false);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0, 10, 20, 30, 40}));
}

TEST(Subsample, AnOpenRouteAlwaysEndsAtTheLastSample)
{
  // 15 m spacing lands on 15 and 30 and would otherwise stop 10 m short of
  // the actual goal.
  const auto indices = subsample_indices(straight_line(41), 15.0, false);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0, 15, 30, 40}));
}

TEST(Subsample, ALoopDoesNotAppendTheLastSample)
{
  // The caller closes the lap by repeating station 0. Emitting sample 40 as
  // well would put a goal pose a few centimetres before it, and Hybrid-A*
  // would plan a hairpin between the two.
  const auto indices = subsample_indices(straight_line(41), 15.0, true);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0, 15, 30}));
}

TEST(Subsample, ZeroSpacingKeepsEverySample)
{
  const auto indices = subsample_indices(straight_line(5), 0.0, false);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0, 1, 2, 3, 4}));
}

TEST(Subsample, SpacingLongerThanTheRouteStillGivesStartAndEnd)
{
  const auto indices = subsample_indices(straight_line(5), 1000.0, false);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0, 4}));
}

TEST(Subsample, EmptyInputGivesNoStations)
{
  EXPECT_TRUE(subsample_indices({}, 10.0, false).empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
