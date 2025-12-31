#include "nav2_coverage/poses_creator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>

namespace nav2_coverage
{

PosesCreator::PosesCreator(const Options & opts, const rclcpp::Logger & logger)
: opts_(opts), logger_(logger)
{
}

GridMeta PosesCreator::metaFromGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  GridMeta m;
  m.frame_id = grid.header.frame_id.empty() ? "map" : grid.header.frame_id;
  m.resolution = static_cast<double>(grid.info.resolution);
  m.width = static_cast<int>(grid.info.width);
  m.height = static_cast<int>(grid.info.height);
  m.origin_x = static_cast<double>(grid.info.origin.position.x);
  m.origin_y = static_cast<double>(grid.info.origin.position.y);
  return m;
}

bool PosesCreator::worldToGrid(double wx, double wy, const GridMeta & meta, int & mx, int & my)
{
  mx = static_cast<int>(std::floor((wx - meta.origin_x) / meta.resolution));
  my = static_cast<int>(std::floor((wy - meta.origin_y) / meta.resolution));
  if (mx < 0 || my < 0 || mx >= meta.width || my >= meta.height) {
    return false;
  }
  return true;
}

std::vector<geometry_msgs::msg::Pose> PosesCreator::orderSerpentine(
  const std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> & nodes,
  int dsx_cells,
  int dsy_cells,
  const std::string & order_mode,
  bool columns_left_to_right,
  bool rows_bottom_to_top)
{
  // nodes tuples are (mx,my,pose)
  std::vector<geometry_msgs::msg::Pose> ordered;

  auto mode = order_mode;
  std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
  if (mode != "columns") {
    mode = "rows";
  }

  if (mode == "columns") {
    // group by bx
    std::unordered_map<int, std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>>> cols;
    cols.reserve(nodes.size());
    for (const auto & t : nodes) {
      const int mx = std::get<0>(t);
      const int bx = mx / std::max(1, dsx_cells);
      cols[bx].push_back(t);
    }

    std::vector<int> keys;
    keys.reserve(cols.size());
    for (const auto & kv : cols) {
      keys.push_back(kv.first);
    }
    std::sort(keys.begin(), keys.end(), [&](int a, int b) {
      return columns_left_to_right ? (a < b) : (a > b);
    });

    ordered.reserve(nodes.size());
    for (size_t i = 0; i < keys.size(); ++i) {
      auto & col = cols[keys[i]];
      std::sort(col.begin(), col.end(), [&](const auto & a, const auto & b) {
        const int my_a = std::get<1>(a);
        const int my_b = std::get<1>(b);
        const int by_a = my_a / std::max(1, dsy_cells);
        const int by_b = my_b / std::max(1, dsy_cells);
        if (by_a != by_b) return by_a < by_b;
        if (my_a != my_b) return my_a < my_b;
        return std::get<0>(a) < std::get<0>(b);
      });

      // even columns reversed (1-based)
      if (((i + 1) % 2) == 0) {
        std::reverse(col.begin(), col.end());
      }

      for (const auto & t : col) {
        ordered.push_back(std::get<2>(t));
      }
    }

    return ordered;
  }

  // mode == "rows": group by by
  std::unordered_map<int, std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>>> rows;
  rows.reserve(nodes.size());
  for (const auto & t : nodes) {
    const int my = std::get<1>(t);
    const int by = my / std::max(1, dsy_cells);
    rows[by].push_back(t);
  }

  std::vector<int> keys;
  keys.reserve(rows.size());
  for (const auto & kv : rows) {
    keys.push_back(kv.first);
  }
  std::sort(keys.begin(), keys.end(), [&](int a, int b) {
    return rows_bottom_to_top ? (a < b) : (a > b);
  });

  ordered.reserve(nodes.size());
  for (size_t i = 0; i < keys.size(); ++i) {
    auto & row = rows[keys[i]];
    std::sort(row.begin(), row.end(), [&](const auto & a, const auto & b) {
      const int mx_a = std::get<0>(a);
      const int mx_b = std::get<0>(b);
      const int bx_a = mx_a / std::max(1, dsx_cells);
      const int bx_b = mx_b / std::max(1, dsx_cells);
      if (bx_a != bx_b) return bx_a < bx_b;
      if (mx_a != mx_b) return mx_a < mx_b;
      return std::get<1>(a) < std::get<1>(b);
    });

    // even rows reversed (1-based)
    if (((i + 1) % 2) == 0) {
      std::reverse(row.begin(), row.end());
    }

    for (const auto & t : row) {
      ordered.push_back(std::get<2>(t));
    }
  }

  return ordered;
}

geometry_msgs::msg::PoseArray PosesCreator::create(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & costmap,
  const std::string & order_mode,
  const bool enable_serpentine,
  const bool columns_left_to_right,
  const bool rows_bottom_to_top,
  const float downsample_step_x,
  const float downsample_step_y,
  const int map_occ_threshold,
  const int costmap_occ_threshold)
{
  const auto map_meta = metaFromGrid(map);
  const auto cost_meta = metaFromGrid(costmap);

  const auto & map_data = map.data;
  const auto & cost_data = costmap.data;

  const int step_x_cells = std::max(1, static_cast<int>(std::lround(opts_.grid_step_x / map_meta.resolution)));
  const int step_y_cells = std::max(1, static_cast<int>(std::lround(opts_.grid_step_y / map_meta.resolution)));

  const int dsx_cells = (downsample_step_x > 0.0) ?
    std::max(1, static_cast<int>(std::lround(downsample_step_x / map_meta.resolution))) : 1;
  const int dsy_cells = (downsample_step_y > 0.0) ?
    std::max(1, static_cast<int>(std::lround(downsample_step_y / map_meta.resolution))) : 1;

  const int half_dsx = dsx_cells / 2;
  const int half_dsy = dsy_cells / 2;

  // bins[(bx,by)] = (mx,my,pose,d2)
  struct BinVal { int mx; int my; geometry_msgs::msg::Pose pose; int d2; };
  std::unordered_map<std::uint64_t, BinVal> bins;
  bins.reserve(static_cast<size_t>(map_meta.width * map_meta.height / (dsx_cells * dsy_cells + 1)));

  auto pack_key = [](int bx, int by) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(bx)) << 32) |
           static_cast<std::uint32_t>(by);
  };

  for (int my = 0; my < map_meta.height; my += step_y_cells) {
    const int row_base = my * map_meta.width;
    for (int mx = 0; mx < map_meta.width; mx += step_x_cells) {
      const int idx = row_base + mx;
      const int8_t occ = map_data[static_cast<size_t>(idx)];

      // map filter
      if (occ < 0) {
        if (!opts_.allow_unknown_map) continue;
      } else {
        if (occ >= map_occ_threshold) continue;
      }

      const double wx = map_meta.origin_x + (static_cast<double>(mx) + 0.5) * map_meta.resolution;
      const double wy = map_meta.origin_y + (static_cast<double>(my) + 0.5) * map_meta.resolution;

      // costmap filter
      int cx = 0, cy = 0;
      if (!worldToGrid(wx, wy, cost_meta, cx, cy)) {
        if (opts_.skip_outside_costmap) continue;
      } else {
        const int cidx = cy * cost_meta.width + cx;
        const int8_t c = cost_data[static_cast<size_t>(cidx)];
        if (c < 0) {
          if (!opts_.allow_unknown_costmap) continue;
        } else {
          if (c >= costmap_occ_threshold) continue;
        }
      }

      const int bx = mx / dsx_cells;
      const int by = my / dsy_cells;

      const int center_mx = bx * dsx_cells + half_dsx;
      const int center_my = by * dsy_cells + half_dsy;
      const int dx = mx - center_mx;
      const int dy = my - center_my;
      const int d2 = dx * dx + dy * dy;

      geometry_msgs::msg::Pose p;
      p.position.x = wx;
      p.position.y = wy;
      p.position.z = 0.0;
      p.orientation.w = 1.0;

      const std::uint64_t key = pack_key(bx, by);
      auto it = bins.find(key);
      if (it == bins.end() || d2 < it->second.d2) {
        bins[key] = BinVal{mx, my, p, d2};
      }
    }
  }

  std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> nodes;
  nodes.reserve(bins.size());
  for (const auto & kv : bins) {
    const auto & v = kv.second;
    nodes.emplace_back(v.mx, v.my, v.pose);
  }

  geometry_msgs::msg::PoseArray out;
  out.header.frame_id = map_meta.frame_id;

  if (!enable_serpentine) {
    std::sort(nodes.begin(), nodes.end(), [](const auto & a, const auto & b) {
      const int ax = std::get<0>(a), ay = std::get<1>(a);
      const int bx = std::get<0>(b), by = std::get<1>(b);
      if (ax != bx) return ax < bx;
      return ay < by;
    });
    out.poses.reserve(nodes.size());
    for (const auto & t : nodes) {
      out.poses.push_back(std::get<2>(t));
    }
    return out;
  }

  out.poses = orderSerpentine(nodes, dsx_cells, dsy_cells, order_mode,
                              columns_left_to_right, rows_bottom_to_top);
  return out;
}

}  // namespace nav2_coverage
