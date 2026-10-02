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

using namespace its;
using namespace std::chrono_literals;

class ITSNode : public rclcpp::Node {
public:
    ITSNode() : Node("its_node") {

        // ---- declare & get parameters ----------------------------------------
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

        // mesh / convex-hull (optional; if set, overrides the analytic surface)
        declare_parameter("fingertip.mesh.filepath",                "");
        declare_parameter("fingertip.mesh.type",                    "obj"); // "obj" | "convex_hull"
        declare_parameter("fingertip.mesh.scale",                   1.0);
        declare_parameter("fingertip.mesh.proxy.displacement.x", 0.0);
        declare_parameter("fingertip.mesh.proxy.displacement.y", 0.0);
        declare_parameter("fingertip.mesh.proxy.displacement.z", 0.0);
        declare_parameter("fingertip.mesh.proxy.orientation.roll", 0.0);
        declare_parameter("fingertip.mesh.proxy.orientation.pitch", 0.0);
        declare_parameter("fingertip.mesh.proxy.orientation.yaw", 0.0);

        declare_parameter("soft_its.algorithm.verbose",             false);
        declare_parameter("soft_its.algorithm.force_threshold",     0.0);
        declare_parameter("soft_its.algorithm.method.name",         "Levenberg-Marquardt");
        declare_parameter("soft_its.algorithm.method.params.count_max",     100);
        declare_parameter("soft_its.algorithm.method.params.stop_threshold",0.005);
        declare_parameter("soft_its.algorithm.method.params.epsilon",       0.01);
        declare_parameter("soft_its.rate",                          0.5);

        declare_parameter("soft_its.wrench_msgs",                   false); // Broadcast the contact via a tf and standard wrench message for compatibility

        const auto sensor_id   = get_parameter("sensor.id").as_string();
        const auto finger_id   = get_parameter("fingertip.id").as_string();
        const double dispX     = get_parameter("fingertip.displacement.x").as_double();
        const double dispY     = get_parameter("fingertip.displacement.y").as_double();
        const double dispZ     = get_parameter("fingertip.displacement.z").as_double();
        const double roll      = get_parameter("fingertip.orientation.roll").as_double();
        const double pitch     = get_parameter("fingertip.orientation.pitch").as_double();
        const double yaw       = get_parameter("fingertip.orientation.yaw").as_double();
        const double a         = get_parameter("fingertip.principalSemiAxis.a").as_double();
        const double b         = get_parameter("fingertip.principalSemiAxis.b").as_double();
        const double c         = get_parameter("fingertip.principalSemiAxis.c").as_double();
        const auto mesh_file   = get_parameter("fingertip.mesh.filepath").as_string();
        const auto mesh_type   = get_parameter("fingertip.mesh.type").as_string();
        const auto mesh_scale   = get_parameter("fingertip.mesh.scale").as_double();
        const double proxyDispX     = get_parameter("fingertip.mesh.proxy.displacement.x").as_double();
        const double proxyDispY     = get_parameter("fingertip.mesh.proxy.displacement.y").as_double();
        const double proxyDispZ     = get_parameter("fingertip.mesh.proxy.displacement.z").as_double();
        const double proxyRoll      = get_parameter("fingertip.mesh.proxy.orientation.roll").as_double();
        const double proxyPitch     = get_parameter("fingertip.mesh.proxy.orientation.pitch").as_double();
        const double proxyYaw       = get_parameter("fingertip.mesh.proxy.orientation.yaw").as_double();

        verbose_    = get_parameter("soft_its.algorithm.verbose").as_bool();
        force_th_   = get_parameter("soft_its.algorithm.force_threshold").as_double();
        solver_name_= get_parameter("soft_its.algorithm.method.name").as_string();
        count_max_  = get_parameter("soft_its.algorithm.method.params.count_max").as_int();
        stop_th_    = get_parameter("soft_its.algorithm.method.params.stop_threshold").as_double();
        eps_        = get_parameter("soft_its.algorithm.method.params.epsilon").as_double();
        rate_hz_    = get_parameter("soft_its.rate").as_double();

        wrench_msgs_compatibility = get_parameter("soft_its.wrench_msgs").as_bool();

        // ---- configure ITS ---------------------------------------------------
        ITS_.sensor_id = sensor_id;
        psa_at_rest_   = {a, b, c};
        ITS_.setFingertipSurface(finger_id, a, b, c); // This is still used with elliptical approximation
        if (!mesh_file.empty()) {
            std::vector<double> proxy_transform = {proxyDispX, proxyDispY, proxyDispZ,
                                                    proxyRoll, proxyPitch, proxyYaw};
            if (mesh_type == "obj"){
                ITS_.setFingertipSurfaceMesh(finger_id, mesh_file, mesh_scale, proxy_transform);
                RCLCPP_INFO(get_logger(), "\033[1;32mMesh: %s\033[0m", mesh_file.c_str());
            } else{
                // Auto computation of convex hull (BUGGATO)
                ITS_.setFingertipSurfaceConvexHull(finger_id, mesh_file, mesh_scale, proxy_transform);
                RCLCPP_INFO(get_logger(), "\033[1;32mMesh (Convex Hull): %s\033[0m", mesh_file.c_str());
            }
        }
        ITS_.setFingertipDisplacement(dispX, dispY, dispZ);
        ITS_.setFingertipOrientation(roll, pitch, yaw);

        // ---- set solver ----------------------------------------------------------
        if (solver_name_ == "Levenberg-Marquardt") {
            solver_ = ContactSensingProblemMethod::Levenberg_Marquardt;
            ITS_.setLMParameters(force_th_, count_max_, stop_th_, eps_, verbose_);
        } else if (solver_name_ == "Gauss-Newton") {
            solver_ = ContactSensingProblemMethod::Gauss_Newton;
        } else if (solver_name_ == "Closed-Form") {
            solver_ = ContactSensingProblemMethod::Closed_Form;
        } else if (solver_name_ == "Custom") {
            solver_ = ContactSensingProblemMethod::Custom;
        } else {
            RCLCPP_WARN(get_logger(), "Unknown solver '%s'. Using Levenberg-Marquardt.", solver_name_.c_str());
            solver_ = ContactSensingProblemMethod::Levenberg_Marquardt;
            ITS_.setLMParameters(force_th_, count_max_, stop_th_, eps_, verbose_);
        }
        RCLCPP_INFO(get_logger(), "\033[1;32mSolver: %s\033[0m", solver_name_.c_str());

        // ---- initial guess ---------------------------------------------------
        X0_.c = {0.0, 0.0, psa_at_rest_[2] / 2.0};
        X0_.K = 0.0;

        // ---- pub/sub ---------------------------------------------------------
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);                                 // Broadcast contact location

        ft_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            sensor_id + "/netft_data", 100,
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { ftCallback(msg); });                 // raw measurment

        ig_sub_ = create_subscription<its_msgs::msg::SoftContactSensingProblemSolution>(
            "its_" + finger_id + "/initial_guess", 100,
            [this](const its_msgs::msg::SoftContactSensingProblemSolution::SharedPtr msg) { igCallback(msg); });  // ITS initial guess

        solution_pub_ = create_publisher<its_msgs::msg::SoftContactSensingProblemSolution>(
            "soft_csp_" + finger_id + "/solution", 100);                                                                            // ITS Solution guess

        if (wrench_msgs_compatibility){
            RCLCPP_INFO(get_logger(), "Publishing wrench message on topic: 'its_%s/wrench'", finger_id.c_str());
            wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
                "its_" + finger_id + "/wrench", 100);                                                                                 // ITS Solution guess as wrench
        }


        const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
        timer_ = create_wall_timer(period, [this]() { timerCallback(); });
    }

private:
    // ---- F/T callback -------------------------------------------------------
    void ftCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        f_(0) = msg->wrench.force.x;              // [N]
        f_(1) = msg->wrench.force.y;              // [N]
        f_(2) = msg->wrench.force.z;              // [N]
        m_(0) = msg->wrench.torque.x;             // [N mm]
        m_(1) = msg->wrench.torque.y;             // [N mm]
        m_(2) = msg->wrench.torque.z;             // [N mm]
        new_measure_ = true;
    }

    // ---- Initial-guess callback ---------------------------------------------
    void igCallback(const its_msgs::msg::SoftContactSensingProblemSolution::SharedPtr msg) {
        X0_.c = {msg->poc.x, msg->poc.y, msg->poc.z};
        const std::string& finger_id = ITS_.fingertip.id;
        const double Dd = msg->d;
        ITS_.setFingertipSurface(finger_id,
                                 psa_at_rest_[0] - Dd,
                                 psa_at_rest_[1] - Dd,
                                 psa_at_rest_[2] - Dd);
        Eigen::Vector3d n = ITS_.fingertip.model.getNormal(X0_.c(0), X0_.c(1), X0_.c(2));
        X0_.K = msg->t / n.norm();
    }

    // ---- Timer --------------------------------------------------------------
    void timerCallback() {
        if (!new_measure_) return;

        const auto t0 = now();

        // Compute initial guess with Closed Form on mesh approximating ellipsoid --------------------------------------------
        if (solver_ != ContactSensingProblemMethod::Closed_Form && solver_ != ContactSensingProblemMethod::Custom ) {
            ITS_.solveContactSensingProblemInitialGuess(f_, m_, force_th_);
            X0_.c = ITS_.X.c;
            X0_.K = ITS_.X.K;
        }
        // -----------------------------------------------------------------



        const int step = ITS_.solveContactSensingProblem(
            f_, m_, force_th_, solver_, X0_, count_max_, stop_th_, eps_, verbose_);
        const auto t1 = now();
        new_measure_ = false;

        const double elapsed_ms = (t1 - t0).nanoseconds() / 1e6;
        its_msgs::msg::SoftContactSensingProblemSolution sol_msg;
        sol_msg.header.frame_id = ITS_.fingertip.id;
        sol_msg.header.stamp    = t1;

        if (step > 0) {
            auto sol = ITS_.getExtendedSolution();
            Eigen::Vector3d n = ITS_.fingertip.model.getNormal(
                sol.PoC(0), sol.PoC(1), sol.PoC(2)).normalized();

            if (verbose_){
                RCLCPP_INFO(get_logger(), "%s", std::string(40, '-').c_str());
                RCLCPP_INFO(get_logger(),
                    "ITS input from %s: F=(%.3f,%.3f,%.3f) [N], M=(%.3f,%.3f,%.3f) [N mm]",
                    ITS_.sensor_id.c_str(),
                    ITS_.f(0), ITS_.f(1), ITS_.f(2),
                    ITS_.m(0), ITS_.m(1), ITS_.m(2));
                RCLCPP_INFO(get_logger(),
                    "ITS solution X=[x,y,z,k]=[%.2f (mm),%.2f (mm),%.2f (mm),%.2f (N mm)]",
                    ITS_.X.c(0), ITS_.X.c(1), ITS_.X.c(2), ITS_.X.K);
                RCLCPP_INFO(get_logger(),
                    "ITS Extended [x,y,z,fn,t]=[%.2f (mm),%.2f (mm),%.2f (mm),%.2f (N),%.2f (N mm)]",
                    sol.PoC(0), sol.PoC(1), sol.PoC(2), sol.fn, sol.t);
                if (step < count_max_)
                    RCLCPP_INFO(get_logger(), "Converged in %i steps (%.3f ms)", step, elapsed_ms);
                else
                    RCLCPP_WARN(get_logger(), "Did not converge in %i steps (%.3f ms)", count_max_, elapsed_ms);
                RCLCPP_INFO(get_logger(), "%s", std::string(40, '-').c_str());
            }
            sol_msg.poc.x = sol.PoC(0); sol_msg.poc.y = sol.PoC(1); sol_msg.poc.z = sol.PoC(2);
            sol_msg.n.x   = n(0);       sol_msg.n.y   = n(1);       sol_msg.n.z   = n(2);
            sol_msg.fn    = sol.fn;
            sol_msg.ft.x  = sol.ft(0);  sol_msg.ft.y  = sol.ft(1);  sol_msg.ft.z  = sol.ft(2);
            sol_msg.t     = sol.t;
            sol_msg.d     = psa_at_rest_[0] - ITS_.fingertip.model.principalAxisCoeff[0];
            sol_msg.convergence_time = elapsed_ms;

            // broadcast TF from fingertip to contact frame
            std::string poc_frame_id = ITS_.fingertip.id + "_PoC";
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp    = t1;
            tf.header.frame_id = ITS_.fingertip.id;
            tf.child_frame_id  = poc_frame_id;
            tf.transform.translation.x = sol.PoC(0) / 1000.0;   // [m]
            tf.transform.translation.y = sol.PoC(1) / 1000.0;   // [m]
            tf.transform.translation.z = sol.PoC(2) / 1000.0;   // [m]

            // tf2::Quaternion q; q.setRPY(0, 0, 0);
            // tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
            // tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
            Eigen::Quaterniond q_poc = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), sol.n);
            tf.transform.rotation.x = q_poc.x();
            tf.transform.rotation.y = q_poc.y();
            tf.transform.rotation.z = q_poc.z();
            tf.transform.rotation.w = q_poc.w();

            tf_broadcaster_->sendTransform(tf);

        if (wrench_msgs_compatibility) {
                // broadcast wrench at contact point
                geometry_msgs::msg::WrenchStamped w; // Note the capital 'W'
                w.header.stamp    = t1;
                w.header.frame_id = poc_frame_id;

                // 3. Calculate Force in the local PoC Frame
                // Reconstruct the total force vector in the fingertip frame
                Eigen::Vector3d f_tip = (sol.fn * sol.n) + sol.ft;

                // Rotate the force vector backwards into our new PoC frame
                Eigen::Vector3d f_local = q_poc.inverse() * f_tip;

                w.wrench.force.x = f_local.x(); // [N]
                w.wrench.force.y = f_local.y(); // [N]
                w.wrench.force.z = f_local.z(); // [N]

                // 4. Calculate Torque in the local PoC Frame
                // Since the Z-axis is the normal, the torsional friction acts only around Z
                w.wrench.torque.x = 0.0;            // [N m]
                w.wrench.torque.y = 0.0;            // [N m]
                w.wrench.torque.z = sol.t/1000.0;   // [N m]

                // Publish it!
                wrench_pub_->publish(w);
            }
        }
        // (publish zero-msg even when step <= 0)
        solution_pub_->publish(sol_msg);
    }

    // ---- members ------------------------------------------------------------
    IntrinsicTactileSensing           ITS_;
    ContactSensingProblemSolution     X0_;
    ContactSensingProblemMethod       solver_ {ContactSensingProblemMethod::Levenberg_Marquardt};
    std::vector<double>               psa_at_rest_ {1.0, 1.0, 1.0};

    Eigen::Vector3d f_ {Eigen::Vector3d::Zero()};
    Eigen::Vector3d m_ {Eigen::Vector3d::Zero()};
    bool new_measure_  {false};

    std::string  solver_name_;
    bool wrench_msgs_compatibility {false};
    bool         verbose_   {false};
    double       force_th_  {0.0};
    int          count_max_ {100};
    double       stop_th_   {0.005};
    double       eps_       {0.01};
    double       rate_hz_   {0.5};

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr                         ft_sub_;
    rclcpp::Subscription<its_msgs::msg::SoftContactSensingProblemSolution>::SharedPtr          ig_sub_;
    rclcpp::Publisher<its_msgs::msg::SoftContactSensingProblemSolution>::SharedPtr             solution_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr                            wrench_pub_;

    rclcpp::TimerBase::SharedPtr                                                               timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                                             tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ITSNode>());
    rclcpp::shutdown();
    return 0;
}
