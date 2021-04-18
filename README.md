# Xilinx ROS 2 minimal examples

#### Basic ROS in embedded
| package | description | acceleration kernel |
|---------|-------------|---------------------|
|[`publisher_xilinx`](publisher_xilinx) | This package contains a minimalistic publisher using a member function for evaluation purposes which subclasses`rclcpp::Node` and sets up an `rclcpp::timer` to periodically call functions which publish messages. | No |

#### Simple acceleration
| package | description | acceleration kernel |
|---------|-------------|---------------------|
|[`vadd_publisher`](vadd_publisher)| A a trivial vector-add ROS 2 publisher. Adds two inputs to a vector in a loop and publishes on the go at 10 Hz. An emulation of the target hardware shows that it's not able to meet the rate target and stays in between 2-4 Hz. *The objective of this package is to generate a computationally expensive baseline when executed in a general purpose embedded CPU. See "accelerated_vadd_publisher" package for an optimized and accelerated_vadd_publisher version of the same package which offloads the vector operations into an FPGA*| No |
|[`accelerated_vadd_publisher`](accelerated_vadd_publisher)| A trivial vector-add ROS 2 publisher that adds two inputs to a vector in a loop and publishes them on the go at 10 Hz. Vector add operations are offloaded into to the FPGA. The offloading operation into the FPGA allows the publisher to meet its targeted rate (10 Hz). *See "vadd_publisher" for a version running purely on the CPUs which fails to maintain the publication rate*| [Yes](accelerated_vadd_publisher/src/vadd.cpp) |
|[`accelerated_vadd_publisher_once`](accelerated_vadd_publisher_once)| Similar to `accelerated_vadd_publisher`, a trivial vector-add ROS 2 publisher that adds two inputs and relies on the same acceleration kernel yet *it publishes only once*. | [Yes](accelerated_vadd_publisher_once/src/vadd.cpp) |