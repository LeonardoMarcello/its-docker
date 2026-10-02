#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "its_msgs/msg/soft_contact_sensing_problem_solution.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "its_ros2/IntrinsicTactileSensing.hpp"
#include "its_ros2/SoftIntrinsicTactileSensing.hpp"

using namespace soft_its;
using namespace std::chrono_literals;

class SoftITSNode : public rclcpp::Node {
public:
    SoftITSNode() : Node("soft_its_node") {
        RCLCPP_INFO(get_logger(), "Hi from soft_its_node");

        // ---- parameters -----------------------------------------------------
        declare_parameter("sensor.id",                              "mySensor");
        declare_parameter("fingertip.id",                           "myFinger");
        declare_parameter("fingertip.displacement.x",               0.0);
        declare_parameter("fingertip.displacement.y",               0.0);
        declare_parameter("fingertip.displacement.z",               0.0);
        declare_parameter("fingertip.orientation.roll",             0.0);
        declare_parameter("fingertip.orientation.pitch",            0.0);
        declare_parameter("fingertip.orientation.yaw",              0.0);
        declare_parameter("fingertip.principalSemiAxis.a",          1.0);
        declare_parameter("fingertip.principalSemiAxis.b",          1.0);
        declare_parameter("fingertip.principalSemiAxis.c",          1.0);
        declare_parameter("fingertip.stiffnessType.a",              0.0);
        declare_parameter("fingertip.stiffnessType.b",              0.0);
        // mesh / convex-hull (optional)
        declare_parameter("fingertip.mesh.filepath",                "");
        declare_parameter("fingertip.mesh.type",                    "obj");

        declare_parameter("soft_its.algorithm.verbose",             false);
        declare_parameter("soft_its.algorithm.force_threshold",     0.0);
        declare_parameter("soft_its.algorithm.method.name",         "Levenberg-Marquardt");
        declare_parameter("soft_its.algorithm.method.params.count_max",     100);
        declare_parameter("soft_its.algorithm.method.params.stop_threshold",0.005);
        declare_parameter("soft_its.algorithm.method.params.epsilon",       0.01);
        declare_parameter("soft_its.rate",                          0.5);

        const auto sensor_id     = get_parameter("sensor.id").as_string();
        const auto finger_id     = get_parameter("fingertip.id").as_string();
        const double dispX       = get_parameter("fingertip.displacement.x").as_double();
        const double dispY       = get_parameter("fingertip.displacement.y").as_double();
        const double dispZ       = get_parameter("fingertip.displacement.z").as_double();
        const double roll        = get_parameter("fingertip.orientation.roll").as_double();
        const double pitch       = get_parameter("fingertip.orientation.pitch").as_double();
        const double yaw         = get_parameter("fingertip.orientation.yaw").as_double();
        const double a           = get_parameter("fingertip.principalSemiAxis.a").as_double();
        const double b           = get_parameter("fingertip.principalSemiAxis.b").as_double();
        const double c           = get_parameter("fingertip.principalSemiAxis.c").as_double();
        const double stiff_a     = get_parameter("fingertip.stiffnessType.a").as_double();
        const double stiff_b     = get_parameter("fingertip.stiffnessType.b").as_double();
        const auto mesh_file     = get_parameter("fingertip.mesh.filepath").as_string();
        const auto mesh_type     = get_parameter("fingertip.mesh.type").as_string();

        verbose_    = get_parameter("soft_its.algorithm.verbose").as_bool();
        force_th_   = get_parameter("soft_its.algorithm.force_threshold").as_double();
        solver_name_= get_parameter("soft_its.algorithm.method.name").as_string();
        count_max_  = get_parameter("soft_its.algorithm.method.params.count_max").as_int();
        stop_th_    = get_parameter("soft_its.algorithm.method.params.stop_threshold").as_double();
        eps_        = get_parameter("soft_its.algorithm.method.params.epsilon").as_double();
        rate_hz_    = get_parameter("soft_its.rate").as_double();

        // ---- configure SITS -------------------------------------------------
        SITS_.sensor_id = sensor_id;
        psa_at_rest_    = {a, b, c};

        if (!mesh_file.empty()) {
            if (mesh_type == "obj"){
                SITS_.setFingertipSurfaceMesh(finger_id, mesh_file);
                RCLCPP_INFO(get_logger(), "\033[1;32mMesh: %s\033[0m", mesh_file.c_str());
            } else{
                SITS_.setFingertipSurfaceConvexHull(finger_id, mesh_file);
                RCLCPP_INFO(get_logger(), "\033[1;32mMesh (Convex Hull): %s\033[0m", mesh_file.c_str());
            }
        } else {
            SITS_.setFingertipSurface(finger_id, a, b, c);
        }
        SITS_.setFingertipDisplacement(dispX, dispY, dispZ);
        SITS_.setFingertipOrientation(roll, pitch, yaw);
        SITS_.setFingertipStiffness(stiff_a, stiff_b);

        // rigid ITS helper for CF-based initial guess (Method B, currently commented)
        ITS_.setFingertipSurface(finger_id, a, b, c);
        ITS_.setFingertipDisplacement(dispX, dispY, dispZ);
        ITS_.setFingertipOrientation(roll, pitch, yaw);

        // ---- solver ---------------------------------------------------------
        if (solver_name_ == "Levenberg-Marquardt") {
            solver_ = its::ContactSensingProblemMethod::Levenberg_Marquardt;
            SITS_.setLMParameters(force_th_, count_max_, stop_th_, eps_, verbose_);
        } else if (solver_name_ == "Gauss-Newton") {
            solver_ = its::ContactSensingProblemMethod::Gauss_Newton;
        } else if (solver_name_ == "Closed-Form") {
            solver_ = its::ContactSensingProblemMethod::Closed_Form;
        } else {
            RCLCPP_WARN(get_logger(), "Unknown solver '%s'. Using Levenberg-Marquardt.", solver_name_.c_str());
            solver_ = its::ContactSensingProblemMethod::Levenberg_Marquardt;
            SITS_.setLMParameters(force_th_, count_max_, stop_th_, eps_, verbose_);
        }
        RCLCPP_INFO(get_logger(), "\033[1;32mSolver: %s\033[0m", solver_name_.c_str());

        // ---- pub/sub --------------------------------------------------------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        ft_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            sensor_id + "/netft_data", 100,
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { ftCallback(msg); });

        ig_sub_ = create_subscription<its_msgs::msg::SoftContactSensingProblemSolution>(
            "soft_csp/initial_guess", 100,
            [this](const its_msgs::msg::SoftContactSensingProblemSolution::SharedPtr msg) { igCallback(msg); });

        solution_pub_ = create_publisher<its_msgs::msg::SoftContactSensingProblemSolution>(
            "soft_csp/solution", 100);

        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(period, [this]() { timerCallback(); });
    }

private:
    void ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        f_(0) = msg->wrench.force.x;
        f_(1) = msg->wrench.force.y;
        f_(2) = msg->wrench.force.z;
        m_(0) = msg->wrench.torque.x / 1000.0;
        m_(1) = msg->wrench.torque.y / 1000.0;
        m_(2) = msg->wrench.torque.z / 1000.0;
        new_measure_ = true;
    }

    void igCallback(const its_msgs::msg::SoftContactSensingProblemSolution::SharedPtr msg) {
        // Method A: initial guess from TacTip
        X0_.c  = {msg->poc.x, msg->poc.y, msg->poc.z};
        X0_.Dd = msg->d;
        Eigen::Vector3d n = SITS_.fingertip.model.getNormal(
            X0_.c(0), X0_.c(1), X0_.c(2), X0_.Dd);
        X0_.K = msg->t / n.norm();
    }

    void timerCallback() {
        if (!new_measure_) return;

        const auto t0   = now();
        const int  step = SITS_.solveContactSensingProblem(
            f_, m_, force_th_, solver_, X0_, count_max_, stop_th_, eps_, verbose_);
        const auto t1   = now();
        new_measure_ = false;

        const double elapsed_ms = (t1 - t0).nanoseconds() / 1e6;

        its_msgs::msg::SoftContactSensingProblemSolution sol_msg;
        sol_msg.header.frame_id = SITS_.fingertip.id;
        sol_msg.header.stamp    = t1;

        if (step > 0) {
            auto sol = SITS_.getExtendedSolution();
            Eigen::Vector3d n = SITS_.fingertip.model.getNormal(
                sol.PoC(0), sol.PoC(1), sol.PoC(2), sol.Dd).normalized();

            RCLCPP_INFO(get_logger(), "%s", std::string(40, '-').c_str());
            RCLCPP_INFO(get_logger(),
                "SITS input from %s: F=(%.3f,%.3f,%.3f) M=(%.3f,%.3f,%.3f)",
                SITS_.sensor_id.c_str(),
                SITS_.f(0), SITS_.f(1), SITS_.f(2),
                SITS_.m(0), SITS_.m(1), SITS_.m(2));
            RCLCPP_INFO(get_logger(),
                "SITS solution X=[x,y,z,k,Dd]=[%.2f,%.2f,%.2f,%.2f,%.2f]",
                SITS_.X.c(0), SITS_.X.c(1), SITS_.X.c(2), SITS_.X.K, SITS_.X.Dd);
            RCLCPP_INFO(get_logger(),
                "SITS Extended [x,y,z,fn,t,Dd]=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                sol.PoC(0), sol.PoC(1), sol.PoC(2), sol.fn, sol.t, sol.Dd);
            if (step < count_max_)
                RCLCPP_INFO(get_logger(), "Converged in %i steps (%.3f ms)", step, elapsed_ms);
            else
                RCLCPP_WARN(get_logger(), "Did not converge in %i steps (%.3f ms)", count_max_, elapsed_ms);
            RCLCPP_INFO(get_logger(), "%s", std::string(40, '-').c_str());

            sol_msg.poc.x = sol.PoC(0); sol_msg.poc.y = sol.PoC(1); sol_msg.poc.z = sol.PoC(2);
            sol_msg.n.x   = n(0);       sol_msg.n.y   = n(1);       sol_msg.n.z   = n(2);
            sol_msg.fn    = sol.fn;
            sol_msg.ft.x  = sol.ft(0);  sol_msg.ft.y  = sol.ft(1);  sol_msg.ft.z  = sol.ft(2);
            sol_msg.t     = sol.t;
            sol_msg.d     = sol.Dd;
            sol_msg.convergence_time = elapsed_ms;

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp    = t1;
            tf.header.frame_id = SITS_.fingertip.id;
            tf.child_frame_id  = "PoC";
            tf.transform.translation.x = sol.PoC(0) / 1000.0;
            tf.transform.translation.y = sol.PoC(1) / 1000.0;
            tf.transform.translation.z = sol.PoC(2) / 1000.0;
            tf2::Quaternion q; q.setRPY(0, 0, 0);
            tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf);
        }
        solution_pub_->publish(sol_msg);
    }

    SoftIntrinsicTactileSensing           SITS_;
    its::IntrinsicTactileSensing          ITS_;   // for CF initial-guess (Method B)
    ContactSensingProblemSolution         X0_;
    its::ContactSensingProblemMethod      solver_ {its::ContactSensingProblemMethod::Levenberg_Marquardt};
    std::vector<double>                   psa_at_rest_ {1.0, 1.0, 1.0};

    Eigen::Vector3d f_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3d m_ {Eigen::Vector3d::Zero()};
    bool new_measure_  {false};

    std::string solver_name_;
    bool        verbose_   {false};
    double      force_th_  {0.0};
    int         count_max_ {100};
    double      stop_th_   {0.005};
    double      eps_       {0.01};
    double      rate_hz_   {0.5};

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr                        ft_sub_;
    rclcpp::Subscription<its_msgs::msg::SoftContactSensingProblemSolution>::SharedPtr         ig_sub_;
    rclcpp::Publisher<its_msgs::msg::SoftContactSensingProblemSolution>::SharedPtr            solution_pub_;
    rclcpp::TimerBase::SharedPtr                                                              timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                            tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoftITSNode>());
    rclcpp::shutdown();
    return 0;
}
