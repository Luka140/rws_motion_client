#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <abb_robot_msgs/srv/set_io_signal.hpp>
#include <abb_robot_msgs/srv/get_rapid_bool.hpp>
#include <abb_robot_msgs/msg/rapid_symbol_path.hpp>


class RWSMotionClient : public rclcpp::Node {
public:
    RWSMotionClient() : Node("rws_motion_client"), success_(true), check_in_progress_(false) {
        start_move_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/start_move",
            std::bind(&RWSMotionClient::startMoveCallback, this, std::placeholders::_1, std::placeholders::_2));

        rws_set_io_client_ = this->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal");
        rws_get_bool_client_ = this->create_client<abb_robot_msgs::srv::GetRAPIDBool>("/rws_client/get_rapid_bool");

        bool_symbol_req_.path.task = "TROB1";
        bool_symbol_req_.path.module = "Grinding";
        bool_symbol_req_.path.symbol = "run_status";

        // Initialize the timer for periodic BOOL checks
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RWSMotionClient::checkBoolValue, this));

        RCLCPP_INFO(this->get_logger(), "RWS Motion Client initialized");
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_move_service_;
    rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr rws_set_io_client_;
    rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedPtr rws_get_bool_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    abb_robot_msgs::srv::GetRAPIDBool::Request bool_symbol_req_;
    bool success_;
    bool check_in_progress_;
    std::string message_;
    std::shared_ptr<std_srvs::srv::Trigger::Response> response_;

    void startMoveCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Start Move request received");
        success_ = true;
        message_.clear();
        response_ = response;

        // Set IO signal to 1
        auto io_req = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
        io_req->signal = "startRoutineSignal";
        io_req->value = "1";

        auto future = rws_set_io_client_->async_send_request(io_req,
            [this, response](rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future) {
                this->handleIOSet1Response(future, response);
            });
    }

    void handleIOSet1Response(rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
            auto result = future.get();
            if (result->result_code == 1) {
                RCLCPP_INFO(this->get_logger(), "IO signal set to 1 successfully");
                check_in_progress_ = true; // Start periodic BOOL check
            } else {
                success_ = false;
                message_ += "Failed to set IO signal to 1. ";
                finalizeResponse(response);
            }
        } catch (const std::exception &e) {
            success_ = false;
            message_ += "Error setting IO signal: " + std::string(e.what());
            finalizeResponse(response);
        }
    }

    void checkBoolValue() {
        if (!check_in_progress_) return;

        RCLCPP_INFO(this->get_logger(), "Checking BOOL value");

        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>(bool_symbol_req_);

        rws_get_bool_client_->async_send_request(bool_req,
            [this](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
                try {
                    auto result = future.get();
                    if (result ->result_code != 1){
                      check_in_progress_ = false;
                      response_->success = false;
                      response_->message = "Failed to check bool value";
                      RCLCPP_INFO(this->get_logger(), "Failed to check bool value - return code: %s", result->message.c_str());
                      finalizeResponse(response_);
                      return;

                    }
                    if (result->value) {
                        RCLCPP_INFO(this->get_logger(), "BOOL value is true");
                        check_in_progress_ = false;
                        response_->success = true;
                        
                        resetIO(response_);

                    } else {
                        RCLCPP_WARN(this->get_logger(), "BOOL value is set to false: -- rechecking");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Error checking BOOL value: %s", e.what());
                    check_in_progress_ = false;
                    response_->success = false;
                    response_->message = "Exception occurred while checking BOOL value";
                    finalizeResponse(response_);
                }
            });
    }


    void resetIO(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Resetting IO");

        auto io_req = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
        io_req->signal = "startRoutineSignal";
        io_req->value = "0";

        rws_set_io_client_->async_send_request(io_req,
            [this, response](rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future) {
                this->handleIOResetResponse(future, response);
            });
    }

    void handleIOResetResponse(
        rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
            auto result = future.get();
            if (result->result_code == 1) { // Assuming 'result_code' is the field
                RCLCPP_INFO(this->get_logger(), "IO reset successfully");
                response->success = true;
                response->message = "IO reset successfully";
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to reset IO");
                response->success = false;
                response->message = "Failed to reset IO";
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error resetting IO: %s", e.what());
            response->success = false;
            response->message = "Exception occurred while resetting IO";
        }
    }

    

    void finalizeResponse(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Response: success=%d, message='%s'", response->success, response->message.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RWSMotionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
