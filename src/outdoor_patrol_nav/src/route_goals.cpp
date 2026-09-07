// Copyright 2026 Outdoor Patrol Team
// SPDX-License-Identifier: Apache-2.0

#include "outdoor_patrol_nav/route_goals.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace outdoor_patrol_nav
{

const char kSourceRawAntenna[] = "raw_antenna";

bool RouteData::is_base_link() const
{
  return source != kSourceRawAntenna;
}

namespace
{

double require_double(const YAML::Node & node, const char * key, std::size_t index)
{
  if (!node[key]) {
    std::ostringstream out;
    out << "sample " << index << " is missing '" << key << "'";
    throw std::runtime_error(out.str());
  }
  try {
    return node[key].as<double>();
  } catch (const YAML::Exception &) {
    std::ostringstream out;
    out << "sample " << index << " has a non-numeric '" << key << "'";
    throw std::runtime_error(out.str());
  }
}

}  // namespace

RouteData parse_route(const std::string & text)
{
  YAML::Node raw;
  try {
    raw = YAML::Load(text);
  } catch (const YAML::Exception & exc) {
    throw std::runtime_error(std::string("route file is not valid YAML: ") + exc.what());
  }
  if (!raw.IsMap()) {
    throw std::runtime_error("route file is not a mapping");
  }

  // A file from a future schema may carry safe_spots or a changed sample
  // layout (plan Phase 4), so refuse it outright rather than silently
  // ignoring the parts this reader does not know about.
  const int version = raw["version"] ? raw["version"].as<int>(-1) : -1;
  if (version != kRouteSchemaVersion) {
    std::ostringstream out;
    out << "route schema version " << version << ", expected " << kRouteSchemaVersion;
    throw std::runtime_error(out.str());
  }

  // The datum is not used here -- /fromLL projects against whatever datum
  // navsat_transform has in force -- but its absence means the file was not
  // written by the recorder, and route_file.loads() rejects that too.
  const YAML::Node datum = raw["datum"];
  if (!datum || !datum.IsMap() || !datum["latitude"] || !datum["longitude"]) {
    throw std::runtime_error("route datum is missing 'latitude' or 'longitude'");
  }

  RouteData route;
  route.loop = raw["loop"] ? raw["loop"].as<bool>(false) : false;
  route.source = raw["source"] ? raw["source"].as<std::string>("") : "odometry_global";

  const YAML::Node samples = raw["samples"];
  if (samples && samples.IsSequence()) {
    route.samples.reserve(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      RouteSample sample;
      sample.lat = require_double(samples[i], "lat", i);
      sample.lon = require_double(samples[i], "lon", i);
      sample.yaw = require_double(samples[i], "yaw", i);
      route.samples.push_back(sample);
    }
  }

  // Four is route_file.py's floor: fewer cannot fit a spline. Nav2 does its
  // own smoothing, but keeping the same floor keeps a file that one loader
  // accepts from being rejected by the other.
  if (route.samples.size() < 4) {
    std::ostringstream out;
    out << "route has " << route.samples.size() << " samples; at least 4 are needed";
    throw std::runtime_error(out.str());
  }
  return route;
}

RouteData read_route(const std::string & path)
{
  std::ifstream handle(path);
  if (!handle) {
    throw std::runtime_error("cannot open route file: " + path);
  }
  std::ostringstream buffer;
  buffer << handle.rdbuf();
  return parse_route(buffer.str());
}

std::vector<std::size_t> subsample_indices(
  const std::vector<std::array<double, 2>> & xy, double spacing_m, bool loop)
{
  std::vector<std::size_t> out;
  if (xy.empty()) {
    return out;
  }
  out.push_back(0);
  if (spacing_m <= 0.0) {
    for (std::size_t i = 1; i < xy.size(); ++i) {
      out.push_back(i);
    }
    return out;
  }

  double since_last = 0.0;
  for (std::size_t i = 1; i < xy.size(); ++i) {
    since_last += std::hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]);
    if (since_last >= spacing_m) {
      out.push_back(i);
      since_last = 0.0;
    }
  }
  if (!loop && out.back() != xy.size() - 1) {
    out.push_back(xy.size() - 1);
  }
  return out;
}

}  // namespace outdoor_patrol_nav
