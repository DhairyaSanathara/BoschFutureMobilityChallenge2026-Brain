#ifndef BFMC_CONTROL__TRAJECTORY_CONTROLLER_NODE_HPP_
#define BFMC_CONTROL__TRAJECTORY_CONTROLLER_NODE_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <bfmc_msgs/msg/control_command.hpp>
#include <bfmc_msgs/msg/vehicle_state.hpp>
#include <bfmc_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace bfmc_control
{

static constexpr int NX = 4;           
static constexpr int NU = 2;           
static constexpr int NH = 10;          
static constexpr double DT = 0.05;     
static constexpr int NZ = NH * NU;     
static constexpr int MAX_REF = 64;     

using StateVec  = Eigen::Matrix<double, NX, 1>;
using CtrlVec   = Eigen::Matrix<double, NU, 1>;
using StateJac  = Eigen::Matrix<double, NX, NX>;  
using InputJac  = Eigen::Matrix<double, NX, NU>;   
using QpVec     = Eigen::Matrix<double, NZ, 1>;    
using QpMat     = Eigen::Matrix<double, NZ, NZ>;   
using SensMat   = Eigen::Matrix<double, NH * NX, NZ>;  

struct RefPoint
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double curvature{0.0};
  double velocity{0.0};
  double time_from_start{0.0};
};

class TrajectoryControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(TrajectoryControllerNode)

  explicit TrajectoryControllerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  double L_{0.26};              
  double a_max_{1.2}; // SYNCED WITH PLANNING LAYER
  double delta_max_{0.436};     
  double v_max_{1.2};           
  double steer_rate_lim_{1.5}; // Slightly opened to allow sharper BFMC turns
  int    max_iter_{3};          
  double eps_vel_{0.01};        

  // OPTIMIZED NMPC WEIGHTS
  double w_xy_{5.0};    // Cartesian Position Tracking (Fixes the blind axis)
  double w_psi_{3.0};   // Heading 
  double w_v_{2.0};     // Velocity
  double w_delta_{0.5}; // Steering effort
  double w_a_{0.1};     // Throttle effort
  double w_ddelta_{15.0}; // MASSIVE PENALTY: Kills Steering Jitters
  double w_da_{5.0};      // MASSIVE PENALTY: Kills Throttle Stutter

  double traj_timeout_s_{0.25};

  // HARDWARE LIMITS (Upgraded to allow physical stopping)
  double min_throttle_{0.05};
  double max_throttle_{0.25};
  double max_brake_{0.45}; // Upgraded so the NMPC can actually stop the car!

  // Hard publish-time clamps (race-day P3): throttle in [0, 0.35], brake in
  // [0, 1]. Throttle floor is 0 — never send negative throttle, ever. This
  // physically forbids reverse motion regardless of NMPC solver output.
  double abs_max_throttle_{0.35};
  double abs_max_brake_   {1.0};

  // Goal-reached neutral (race-day P3). When the vehicle is within
  // goal_radius_m_ of the final reference point AND the planner has flagged
  // a terminal zero-velocity reference, publish strict neutral
  // (throttle=brake=steering=0) for goal_hold_s_ before going to neutral idle.
  double goal_radius_m_   {0.25};
  double goal_v_eps_      {0.05};
  bool   goal_reached_latched_{false};
  double veh_x_{0.0};
  double veh_y_{0.0};

  std::array<StateVec, NH + 1> x_pred_;
  std::array<CtrlVec, NH> u_seq_;
  CtrlVec u_prev_;
  std::array<StateJac, NH> A_;
  std::array<InputJac, NH> B_;
  SensMat G_;   
  QpMat   H_;   
  QpVec   g_;   
  QpVec   du_;  

  std::array<RefPoint, MAX_REF> ref_raw_;
  int ref_count_{0};
  std::array<RefPoint, NH + 1> ref_interp_;
  rclcpp::Time last_traj_stamp_;

  double veh_speed_{0.0};
  double veh_confidence_{0.0};
  bool   veh_valid_{false};
  // Race-day: IMU accel confidence (from /fusion/imu_confidence). Used to
  // gate how much the target trajectory speed influences the open-loop
  // speed estimate — when IMU is unhealthy, we trust the previous estimate
  // more and target less. Clamped to [0,1]; default 1.0 means "fully trust".
  double imu_accel_confidence_{1.0};

  rclcpp::Subscription<bfmc_msgs::msg::Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<bfmc_msgs::msg::VehicleState>::SharedPtr sub_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_imu_conf_;
  bool emergency_stop_active_{false};

  rclcpp_lifecycle::LifecyclePublisher<bfmc_msgs::msg::ControlCommand>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cb_trajectory(const bfmc_msgs::msg::Trajectory::SharedPtr msg);
  void cb_vehicle_state(const bfmc_msgs::msg::VehicleState::SharedPtr msg);
  void cb_estop(const std_msgs::msg::Bool::SharedPtr msg);

  void control_tick();
  void interpolate_reference();
  void warm_start();
  StateVec bicycle_predict(const StateVec & x, const CtrlVec & u) const;
  StateJac bicycle_jacobian_A(const StateVec & x, const CtrlVec & u) const;
  InputJac bicycle_jacobian_B(const StateVec & x, const CtrlVec & u) const;
  void forward_simulate();
  void compute_jacobians();
  void build_qp();
  void solve_qp();
  void project_constraints();
  void publish_command();
  void publish_neutral();
  static double wrap_angle(double a);
};

}  // namespace bfmc_control

#endif  // BFMC_CONTROL__TRAJECTORY_CONTROLLER_NODE_HPP_
