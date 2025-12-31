#ifndef NAV2_COVERAGE__POSES_CREATOR_HPP_
#define NAV2_COVERAGE__POSES_CREATOR_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/logger.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace nav2_coverage
{

struct GridMeta
{
  std::string frame_id;
  double resolution{0.05};
  int width{0};
  int height{0};
  double origin_x{0.0};
  double origin_y{0.0};
};

class PosesCreator
{
public:
  struct Options
  {
    // Raw scan stride (meters)
    double grid_step_x{0.05};
    double grid_step_y{0.05};

    // Filters
    bool allow_unknown_map{false};
    bool allow_unknown_costmap{false};
    bool skip_outside_costmap{true};
  };

  PosesCreator(const Options & opts, const rclcpp::Logger & logger);

  geometry_msgs::msg::PoseArray create(
    const nav_msgs::msg::OccupancyGrid & map,
    const nav_msgs::msg::OccupancyGrid & costmap,
    const std::string & order_mode /* "rows" or "columns" */,
    const bool enable_serpentine,
    const bool columns_left_to_right,
    const bool rows_bottom_to_top,
    const float downsample_step_x,
    const float downsample_step_y,
    const int map_occ_threshold,
    const int costmap_occ_threshold);

private:
  Options opts_;
  rclcpp::Logger logger_;

  static GridMeta metaFromGrid(const nav_msgs::msg::OccupancyGrid & grid);

  static bool worldToGrid(double wx, double wy, const GridMeta & meta, int & mx, int & my);

  static std::vector<geometry_msgs::msg::Pose> orderSerpentine(
    const std::vector<std::tuple<int, int, geometry_msgs::msg::Pose>> & nodes,
    int dsx_cells,
    int dsy_cells,
    const std::string & order_mode,
    bool columns_left_to_right,
    bool rows_bottom_to_top);
};

}  // namespace nav2_coverage

#endif  // NAV2_COVERAGE__POSES_CREATOR_HPP_
