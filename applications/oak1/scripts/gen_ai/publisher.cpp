#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Include necessary DepthAI headers
#include <depthai/depthai.hpp>

using namespace std::chrono_literals;

class FeatureTrackingNode : public rclcpp::Node
{
public:
    FeatureTrackingNode() : Node("feature_tracking_node")
    {
        // Create a publisher for feature points
        feature_points_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "tracked_features", 10);

        // Initialize DepthAI pipeline
        initialize_depthai_pipeline();
    }

private:
    void initialize_depthai_pipeline()
    {
        try
        {
            // Create the pipeline
            dai::Pipeline pipeline;

            // Define input source
            auto mono = pipeline.create<dai::node::MonoCamera>();
            mono->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
            mono->setFps(30);

            // Create the spatial feature tracker node
            auto sf = pipeline.create<dai::node::SpatialFeatureTracker>();

            // Link nodes
            mono->out.link(sf.inputMonoLeft);
            sf.out.link(feature_points_publisher_->get_subscription());

            // Start pipeline
            device_ = dai::Device(pipeline);

            RCLCPP_INFO(this->get_logger(), "DepthAI pipeline initialized and running.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error initializing DepthAI pipeline: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void publish_feature_points(const sensor_msgs::msg::PointCloud2::SharedPtr &message)
    {
        feature_points_publisher_->publish(*message);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr feature_points_publisher_;
    dai::Device device_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FeatureTrackingNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}