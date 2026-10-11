#include "sura_manipulation_actions/plan_joint_trajectory_action_server.hpp"

#include <gtest/gtest.h>
#include <thread>

namespace sura_manipulation_actions
{
class ControllerSwitchTest : public ::testing::Test
{
protected:
  using Server = PlanJointTrajectoryActionServer;
  using List = controller_manager_msgs::srv::ListControllers;
  using Switch = controller_manager_msgs::srv::SwitchController;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    server_ = std::make_shared<Server>();
    fake_ = std::make_shared<rclcpp::Node>("fake_split_managers");
    server_->controller_list_service_ = "/test_alpha/list_controllers";
    server_->controller_switch_service_ = "/test_alpha/switch_controller";
    server_->task_priority_controller_ = "task_priority_controller";
    server_->body_velocity_controller_ = "body_velocity";
    server_->left_velocity_controller_ = "alpha_left_forward_velocity_controller";
    server_->right_velocity_controller_ = "alpha_right_forward_velocity_controller";
    server_->left_trajectory_controller_ = "alpha_left_joint_trajectory_controller";
    server_->right_trajectory_controller_ = "alpha_right_joint_trajectory_controller";
    server_->list_controllers_client_ = server_->create_client<List>(server_->controller_list_service_);
    server_->switch_controller_client_ = server_->create_client<Switch>(server_->controller_switch_service_);
    server_->vehicle_list_client_ = server_->create_client<List>("/test_vehicle/list_controllers");
    server_->vehicle_switch_client_ = server_->create_client<Switch>("/test_vehicle/switch_controller");
    alpha_state_ = {{"task_priority_controller", "inactive"}, {"alpha_left_cartesian_velocity_controller", "active"}};
    vehicle_state_ = {{"body_velocity", "inactive"}, {"stabilize", "active"}};
    alpha_list_ = fake_->create_service<List>("/test_alpha/list_controllers",
      [this](const std::shared_ptr<List::Request>, std::shared_ptr<List::Response> response) {
        list(alpha_state_, *response);
      });
    vehicle_list_ = fake_->create_service<List>("/test_vehicle/list_controllers",
      [this](const std::shared_ptr<List::Request>, std::shared_ptr<List::Response> response) {
        list(vehicle_state_, *response);
      });
    alpha_switch_ = fake_->create_service<Switch>("/test_alpha/switch_controller",
      [this](const std::shared_ptr<Switch::Request> request, std::shared_ptr<Switch::Response> response) {
        order_.push_back("alpha");
        alpha_requests_.push_back(*request);
        if (delay_alpha_ && alpha_requests_.size() == 1U) {
          std::this_thread::sleep_for(std::chrono::milliseconds(800));
        }
        response->ok = !reject_alpha_;
        if (response->ok) {apply(alpha_state_, *request);}
      });
    vehicle_switch_ = fake_->create_service<Switch>("/test_vehicle/switch_controller",
      [this](const std::shared_ptr<Switch::Request> request, std::shared_ptr<Switch::Response> response) {
        order_.push_back("vehicle");
        vehicle_requests_.push_back(*request);
        response->ok = !reject_vehicle_;
        if (response->ok) {apply(vehicle_state_, *request);}
      });
    executor_.add_node(server_->get_node_base_interface());
    executor_.add_node(fake_);
    spin_ = std::thread([this]() {executor_.spin();});
  }

  void TearDown() override
  {
    executor_.cancel();
    spin_.join();
    executor_.remove_node(server_->get_node_base_interface());
    executor_.remove_node(fake_);
    server_.reset();
    fake_.reset();
  }

  static void list(const std::map<std::string, std::string> & states, List::Response & response)
  {
    for (const auto & entry : states) {
      controller_manager_msgs::msg::ControllerState state;
      state.name = entry.first;
      state.state = entry.second;
      response.controller.push_back(state);
    }
  }

  static void apply(std::map<std::string, std::string> & states, const Switch::Request & request)
  {
    for (const auto & name : request.deactivate_controllers) {states[name] = "inactive";}
    for (const auto & name : request.activate_controllers) {states[name] = "active";}
  }

  bool execute(std::string & error) {return server_->switchControllersForTaskExecution(error);}
  void setTimeout(double timeout) {server_->controller_switch_timeout_ = timeout;}
  std::shared_ptr<Server> server_;
  rclcpp::Node::SharedPtr fake_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_;
  rclcpp::Service<List>::SharedPtr alpha_list_, vehicle_list_;
  rclcpp::Service<Switch>::SharedPtr alpha_switch_, vehicle_switch_;
  std::map<std::string, std::string> alpha_state_, vehicle_state_;
  std::vector<Switch::Request> alpha_requests_, vehicle_requests_;
  std::vector<std::string> order_;
  bool delay_alpha_{false};
  bool reject_alpha_{false};
  bool reject_vehicle_{false};
};

TEST_F(ControllerSwitchTest, ActivatesVehicleBeforeAlpha)
{
  std::string error;
  ASSERT_TRUE(execute(error)) << error;
  EXPECT_EQ(order_, (std::vector<std::string>{"vehicle", "alpha"}));
  EXPECT_EQ(vehicle_state_["body_velocity"], "active");
  EXPECT_EQ(vehicle_state_["stabilize"], "inactive");
  EXPECT_EQ(alpha_state_["task_priority_controller"], "active");
  EXPECT_EQ(alpha_state_["alpha_left_cartesian_velocity_controller"], "inactive");
}

TEST_F(ControllerSwitchTest, RestoresVehicleWhenAlphaRejectsActivation)
{
  reject_alpha_ = true;
  std::string error;
  ASSERT_FALSE(execute(error));
  EXPECT_EQ(order_, (std::vector<std::string>{"vehicle", "alpha", "vehicle"}));
  EXPECT_EQ(vehicle_state_["body_velocity"], "inactive");
  EXPECT_EQ(vehicle_state_["stabilize"], "active");
  EXPECT_EQ(alpha_state_["task_priority_controller"], "inactive");
}

TEST_F(ControllerSwitchTest, DoesNotActivateAlphaIfVehicleRejectsSetup)
{
  reject_vehicle_ = true;
  std::string error;
  ASSERT_FALSE(execute(error));
  EXPECT_EQ(order_, (std::vector<std::string>{"vehicle"}));
  EXPECT_EQ(alpha_state_["task_priority_controller"], "inactive");
}

TEST_F(ControllerSwitchTest, StopsLateAlphaActivationBeforeRestoringVehicle)
{
  delay_alpha_ = true;
  setTimeout(0.05);
  std::string error;
  ASSERT_FALSE(execute(error));
  EXPECT_EQ(order_, (std::vector<std::string>{"vehicle", "alpha", "alpha", "vehicle"}));
  EXPECT_EQ(alpha_state_["task_priority_controller"], "inactive");
  EXPECT_EQ(vehicle_state_["body_velocity"], "inactive");
  EXPECT_EQ(vehicle_state_["stabilize"], "active");
}

TEST_F(ControllerSwitchTest, DoesNotSwitchManagersAlreadyInTaskMode)
{
  alpha_state_ = {{"task_priority_controller", "active"}};
  vehicle_state_ = {{"body_velocity", "active"}};
  std::string error;
  ASSERT_TRUE(execute(error)) << error;
  EXPECT_TRUE(order_.empty());
}
}  // namespace sura_manipulation_actions
