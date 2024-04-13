#include <memory>
#include "stereo_image_proc/disparity_fpga.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <vitis_common/common/xf_headers.hpp>
#include <vitis_common/common/utilities.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto disparity_node =
    std::make_shared<stereo_image_proc::DisparityNodeFPGA>(options);
  exec.add_node(disparity_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

