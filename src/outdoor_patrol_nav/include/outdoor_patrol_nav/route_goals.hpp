// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0
//
// Route file -> Nav2 goal poses.
//
// The two pure functions patrol_mission needs, kept out of the node so they
// can be unit-tested without a ROS graph.
//
// This is a deliberately MINIMAL reader for the schema written by
// outdoor_patrol_route/route_file.py. It reads only what a Nav2 mission
// needs -- version, loop, source, and each sample's lat/lon/yaw -- and it
// cannot write. The Python module remains the full implementation and the
// owner of the format; the recorder, the follower and score_route.py all
// still use it. Keep the version check and the rejections below in step with
// route_file.loads(), because "the file parsed" must mean the same thing on
// both sides.

#ifndef OUTDOOR_PATROL_NAV__ROUTE_GOALS_HPP_
#define OUTDOOR_PATROL_NAV__ROUTE_GOALS_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace outdoor_patrol_nav
{

/// The only schema version this reader accepts (route_file.SCHEMA_VERSION).
constexpr int kRouteSchemaVersion = 1;

/// A route recorded at the ANTENNA, not at base_link. Diagnostics only.
extern const char kSourceRawAntenna[];

struct RouteSample
{
  double lat {0.0};
  double lon {0.0};
  double yaw {0.0};   //< REP-103: ENU, radians, CCW from east.
};

struct RouteData
{
  bool loop {false};
  std::string source;
  std::vector<RouteSample> samples;

  /// False for a diagnostics-only file recorded at the antenna phase centre.
  bool is_base_link() const;
};

/// Parse route file text. Throws std::runtime_error on anything a mission
/// would otherwise act on silently and wrongly.
RouteData parse_route(const std::string & text);

/// Read and parse a route file from disk.
RouteData read_route(const std::string & path);

/// Indices of `xy` spaced at least `spacing_m` apart along the polyline.
///
/// Index 0 is always included. On an OPEN route the final index is too --
/// it is the actual goal. On a loop it is not: the caller closes the lap by
/// repeating station 0, and emitting a point a few centimetres before it
/// would make Nav2 plan a hairpin.
std::vector<std::size_t> subsample_indices(
  const std::vector<std::array<double, 2>> & xy, double spacing_m, bool loop);

}  // namespace outdoor_patrol_nav

#endif  // OUTDOOR_PATROL_NAV__ROUTE_GOALS_HPP_
