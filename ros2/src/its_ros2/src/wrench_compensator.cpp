//# TO DO: node for fingertip compensation
//# - get_initial fingertip orientation
//# - estimate initial wrench included in initial bias b0
//# - get_current fingertip orientation
//# - estimate current wrench w
//# - correct with f = f_m + b0 - w
//# - broadcast corrected f
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unordered_map>
#include <string>
#include <vector>
#include <stdexcept>

// Structure to hold state and communication objects for each sensor independently
struct SensorContext {
    std::string frame_id;
    bool b0_computed{false};
    Eigen::Matrix<double, 6, 1> b0;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscriber;
};

class FingertipCompensationNode : public rclcpp::Node {
public:
    FingertipCompensationNode() : Node("fingertip_compensation_node") {

        // --- 1. Declare Parameters ---
        this->declare_parameter<std::vector<std::string>>("sensors.id", std::vector<std::string>());
        this->declare_parameter<std::vector<std::string>>("fingertips.id", std::vector<std::string>());

        this->declare_parameter<std::string>("input_topic_name", "netft_data");
        this->declare_parameter<std::string>("world_frame", "palm_link");

        // Additional Rotation (Fingertip to Sensor)
        this->declare_parameter<double>("rotation.roll", 0.0);
        this->declare_parameter<double>("rotation.pitch", 0.0);
        this->declare_parameter<double>("rotation.yaw", 0.0);


        this->declare_parameter<double>("mass", 0.0);
        this->declare_parameter<std::vector<double>>("com", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("Ib", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        // --- 2. Load Parameters ---
        std::vector<std::string> sensor_ids = this->get_parameter("sensors.id").as_string_array();
        std::vector<std::string> fingertip_ids = this->get_parameter("fingertips.id").as_string_array();

        std::string input_topic = this->get_parameter("input_topic_name").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        mass_ = this->get_parameter("mass").as_double();

        std::vector<double> com_vec = this->get_parameter("com").as_double_array();
        if (com_vec.size() == 3) {
            com_ = Eigen::Vector3d(com_vec[0], com_vec[1], com_vec[2]);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'com' must have exactly 3 elements.");
        }

        std::vector<double> ib_vec = this->get_parameter("Ib").as_double_array();
        if (ib_vec.size() == 6) {
            Ib_ << ib_vec[0], ib_vec[1], ib_vec[2],
                   ib_vec[1], ib_vec[3], ib_vec[4],
                   ib_vec[2], ib_vec[4], ib_vec[5];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'Ib' must have exactly 6 elements.");
        }

        // --- 3. Safety Check on Parallel Arrays ---
        if (sensor_ids.size() != fingertip_ids.size()) {
            RCLCPP_FATAL(this->get_logger(),
                "CRITICAL ERROR: 'sensors.id' size (%zu) does not match 'fingertips.id' size (%zu). Check your YAML!",
                sensor_ids.size(), fingertip_ids.size());
            throw std::runtime_error("Mismatched parameter arrays.");
        }

        // Sensor offset
        double roll = this->get_parameter("rotation.roll").as_double();
        double pitch = this->get_parameter("rotation.pitch").as_double();
        double yaw = this->get_parameter("rotation.yaw").as_double();

        R_offset_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        // --- 4. TF2 Setup ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- 5. Initialize Multi-Sensor Pubs/Subs ---
        if (sensor_ids.empty()) {
            RCLCPP_WARN(this->get_logger(), "No sensors found in parameter list!");
        }

        for (size_t i = 0; i < sensor_ids.size(); ++i) {
            std::string id = sensor_ids[i];

            // Map the corresponding frame explicitly from the YAML array index
            sensors_[id].frame_id = fingertip_ids[i];

            // Build dynamic topics: e.g., /thumb_sensor/netft_data
            std::string sub_topic = "/" + id + "/" + input_topic;
            std::string pub_topic = "/" + id + "_compensated/" + input_topic;

            sensors_[id].publisher = this->create_publisher<geometry_msgs::msg::WrenchStamped>(pub_topic, 10);

            sensors_[id].subscriber = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
                sub_topic, 10,
                [this, id](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                    this->wrench_callback(msg, id);
                }
            );

            RCLCPP_INFO(this->get_logger(), "Initialized compensator for sensor '%s' tracking frame '%s'",
                        id.c_str(), sensors_[id].frame_id.c_str());
        }
    }

private:
    std::string world_frame_;
    double mass_;
    Eigen::Vector3d com_;
    Eigen::Matrix3d Ib_;
    Eigen::Matrix3d R_offset_; // Rotation from Fingertip to Sensor

    // Maps the custom sensor string name (e.g., "thumb_sensor") to its internal state/topics
    std::unordered_map<std::string, SensorContext> sensors_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    Eigen::Matrix<double, 6, 1> compute_gravity_wrench(const geometry_msgs::msg::TransformStamped& transform) {
        // Construct quaternion (Eigen order: w, x, y, z)
        Eigen::Quaterniond q(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z
        );

        // Rotation matrices
        Eigen::Matrix3d R_world_sensor = q.toRotationMatrix();
        Eigen::Matrix3d R_sensor_world = R_world_sensor.transpose();

        // Gravity vector in world frame (Z-down)
        Eigen::Vector3d g_world(0.0, 0.0, -9.81);

        // Rotate gravity vector into the specific sensor frame
        Eigen::Vector3d g_sensor = R_sensor_world * g_world;

        // Force and Torque from mass & CoM
        Eigen::Vector3d f_g = R_offset_ * mass_ * g_sensor;
        Eigen::Vector3d tau_g = R_offset_ * com_.cross(f_g);

        Eigen::Matrix<double, 6, 1> wrench;
        wrench << f_g, tau_g;
        return wrench;
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg, const std::string& id) {
        geometry_msgs::msg::TransformStamped t;
        try {
            // Explicitly look up the frame assigned to this specific sensor name
            t = tf_buffer_->lookupTransform(world_frame_, sensors_[id].frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF Error for %s (%s): %s",
                id.c_str(), sensors_[id].frame_id.c_str(), ex.what());
            return;
        }

        // 1. Estimate current wrench w
        Eigen::Matrix<double, 6, 1> w_current = compute_gravity_wrench(t);

        // 2. Estimate initial wrench b0 (execute only once per sensor at startup)
        if (!sensors_[id].b0_computed) {
            sensors_[id].b0_computed = true;
            sensors_[id].b0 = w_current;
            RCLCPP_INFO(this->get_logger(), "Initial orientation acquired for '%s'. Bias b0 computed.", id.c_str());
        }

        // Parse measured wrench (f_m)
        Eigen::Matrix<double, 6, 1> f_m;
        f_m << msg->wrench.force.x,
               msg->wrench.force.y,
               msg->wrench.force.z,
               msg->wrench.torque.x,
               msg->wrench.torque.y,
               msg->wrench.torque.z;

        // 3. Correct with: f = f_m + b0 - w
        Eigen::Matrix<double, 6, 1> f_compensated = f_m + sensors_[id].b0 - w_current;

        // 4. Broadcast corrected f
        geometry_msgs::msg::WrenchStamped comp_msg;
        comp_msg.header = msg->header;

        comp_msg.wrench.force.x = f_compensated(0);
        comp_msg.wrench.force.y = f_compensated(1);
        comp_msg.wrench.force.z = f_compensated(2);

        comp_msg.wrench.torque.x = f_compensated(3);
        comp_msg.wrench.torque.y = f_compensated(4);
        comp_msg.wrench.torque.z = f_compensated(5);

        sensors_[id].publisher->publish(comp_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FingertipCompensationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
