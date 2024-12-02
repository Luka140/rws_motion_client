#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <abb_robot_msgs/srv/set_io_signal.hpp>
#include <abb_robot_msgs/srv/get_rapid_bool.hpp>
#include <abb_robot_msgs/srv/set_rapid_bool.hpp>
#include <abb_robot_msgs/msg/rapid_symbol_path.hpp>
#include <abb_robot_msgs/srv/trigger_with_result_code.hpp>

#include <stamped_std_msgs/msg/float32_stamped.hpp>
#include <data_gathering_msgs/srv/start_grind_test.hpp>
#include <data_gathering_msgs/srv/start_grinder.hpp>
#include <data_gathering_msgs/srv/stop_grinder.hpp>

#include <string>


class RWSMotionClient : public rclcpp::Node {
public:
    RWSMotionClient() : Node("rws_motion_client"){
        start_move_service_ = this->create_service<data_gathering_msgs::srv::StartGrindTest>(
            "~/start_grind_move",
            std::bind(&RWSMotionClient::startMoveCallback, this, std::placeholders::_1, std::placeholders::_2));

        rws_set_bool_client_        = this->create_client<abb_robot_msgs::srv::SetRAPIDBool>("/rws_client/set_rapid_bool");
        rws_get_bool_client_        = this->create_client<abb_robot_msgs::srv::GetRAPIDBool>("/rws_client/get_rapid_bool");
        rws_pp_to_main_client_      = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/pp_to_main");
        rws_pp_start_rapid_client_  = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/start_rapid");

        start_grinder_client_       = this->create_client<data_gathering_msgs::srv::StartGrinder>("/grinder_node/enable_grinder");
        stop_grinder_client_        = this->create_client<data_gathering_msgs::srv::StopGrinder>("/grinder_node/disable_grinder");
        acf_force_publisher_        = this->create_publisher<stamped_std_msgs::msg::Float32Stamped>("/acf/force", 10);

        // bool_symbol_req_default.path.task = "T_ROB1";
        // bool_symbol_req_default.path.module = "Grinding";

        symbol_home_flag = "waiting_at_home";
        symbol_grind0_flag = "waiting_at_grind0";
        symbol_grind_done_flag = "finished_grind_pass";
        symbol_run_status_flag = "run_status";

        bool_timer_period_ = 250; // Milliseconds 
        flip_back_timer_busy = false;
        finished_grind_timer_busy = false;

        // Retract acf
        // TODO
        
        stamped_std_msgs::msg::Float32Stamped retract_message;
        retract_message.data = -5.0;
        retract_message.header.stamp = this->get_clock()->now(); // Ensure timestamp is set
        acf_force_publisher_->publish(retract_message);

        RCLCPP_INFO(this->get_logger(), "RWS Motion Client initialized");
    }

private:
    // Main service to perform a test and response
    rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>::SharedPtr start_move_service_;
    std::string message_;
    std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Request> test_request_;
    std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> test_response_;
    bool success_;

    // rws interfaces 
    rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_pp_to_main_client_;
    rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_pp_start_rapid_client_;
    // rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr rws_set_io_client_;
    rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedPtr rws_get_bool_client_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr rws_set_bool_client_;
    
    rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedPtr start_grinder_client_;
    rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedPtr stop_grinder_client_;

    rclcpp::Publisher<stamped_std_msgs::msg::Float32Stamped>::SharedPtr acf_force_publisher_;

    // Bool request with module and task preset
    abb_robot_msgs::srv::GetRAPIDBool::Request bool_symbol_req_default;

    rclcpp::TimerBase::SharedPtr timer_wait_home_;
    rclcpp::TimerBase::SharedPtr timer_wait_grind0_;
    rclcpp::TimerBase::SharedPtr timer_wait_grind_done_;
    rclcpp::TimerBase::SharedPtr timer_wait_script_done_;

    std::string symbol_home_flag;
    std::string symbol_grind0_flag;
    std::string symbol_grind_done_flag;
    std::string symbol_run_status_flag; 
    
    int bool_timer_period_;
    bool flip_back_timer_busy;
    bool finished_grind_timer_busy;

    void startMoveCallback(const data_gathering_msgs::srv::StartGrindTest::Request::SharedPtr request,
                           data_gathering_msgs::srv::StartGrindTest::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Start test request received");

        // CHECK WHETHER ABB IS ALREADY RUNNING RAPID 
        auto is_running_bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
        is_running_bool_req->path.task = "T_ROB1";
        is_running_bool_req->path.module = "Grinding";
        is_running_bool_req->path.symbol = symbol_run_status_flag;

        // Now use is_running_bool_req as the argument
        rws_get_bool_client_->async_send_request(is_running_bool_req, 
        [this, request, response](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
            this->handleRunStatusResponse(future, request, response);
        });

    }

    void handleRunStatusResponse(rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future, 
                                 std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Request> request,
                                 std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){

        // All conditions for early return 
        try {
            auto result = future.get();
            if (result->result_code == 1) {
                // There is a program currently runing. Abort 
                if (result->value){
                    RCLCPP_INFO(this->get_logger(), "There is a program running. 'run_status' is true");
                    response->success = false;
                    response->message += "There is a program running. 'run_status' is true";
                    finalizeResponse(response);
                    return; 
                }

            } else {
                response->success = false;
                response->message += "Failed to read run_status bool";
                finalizeResponse(response);
                return;
            }
            
        } catch (const std::exception &e) {
            response->success = false;
            response->message += "Error reading run_status bool: "  + std::string(e.what());
            finalizeResponse(response);
            return;
        }

        // There is no current program running -> Proceed 
        RCLCPP_INFO(this->get_logger(), "ABB Ready...");

        // Reset flags and save response 
        success_ = true;
        message_.clear();
        test_response_ = response;
        test_request_ =  request;

        RCLCPP_INFO(this->get_logger(), "Requesting PP to main...");
        // TODO
        // SET SPEED AND PASSES
        auto pp_to_main_req = std::make_shared<abb_robot_msgs::srv::TriggerWithResultCode::Request>();
        rws_pp_to_main_client_->async_send_request(pp_to_main_req,
            [this,response](rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future){this->handlePPToMainResponse(future, response);
        });
    }

    void handlePPToMainResponse(rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){

        // Conditions for early return 
        try {
            auto result = future.get();
            if (result->result_code != 1) {
                test_response_->success = false;
                test_response_->message += "Failed to set pp to main";
                finalizeResponse(test_response_);
                return;
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading pp to main response " + std::string(e.what());
            finalizeResponse(test_response_);
            return;

        }
        RCLCPP_INFO(this->get_logger(), "Requesting Start RAPID");
        // PP was successfully set to main. Start RAPID
        auto request = std::make_shared<abb_robot_msgs::srv::TriggerWithResultCode::Request>();
        rws_pp_start_rapid_client_->async_send_request(request,
                [this,response](rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future){this->handleStartRapidResponse(future, response);
        });

    }

    void handleStartRapidResponse(rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        // Conditions for early return 
        try {
            auto result = future.get();
            if (result->result_code != 1) {
                test_response_->success = false;
                test_response_->message += "Failed to start RAPID";
                finalizeResponse(test_response_);
                return; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error start RAPID response " + std::string(e.what());
            finalizeResponse(test_response_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Moving to home...");

        // RAPID Successfully started
        // Next wait until bool 'waiting_at_home' is set to True
        // Initialize the timer for periodic BOOL checks
        timer_wait_home_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, response]() {
                // Create weak_ptr from shared_ptr inside the callback
                auto weak_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_home_);

                // Try to lock the weak pointer
                if (auto timer = weak_timer.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_home_flag, 
                        weak_timer,             
                        std::bind(&RWSMotionClient::enableGrinder, this, response),
                        response                // Pass response
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Could not lock weak pointer");
                }
            }
        );

    }

    void checkBoolValueFlipBack(const std::string &value,  const std::weak_ptr<rclcpp::TimerBase> weak_timer, 
            std::function<void(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response>)> next_func, 
            std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response) {
        /*

        Function called by timer to periodically check a bool from RWS until it is found to be True.
        Once this is the case, it is flipped back to false, and the timer is stopped. 

        */

        // Check and set flag so that the timer won't call this function again while this one is still in progress.
        if (flip_back_timer_busy){
            RCLCPP_WARN(this->get_logger(), "Overlapping timer callback for checkBoolValueFlipBack. Ignoring...");    
            return;
        }

        flip_back_timer_busy= true;

        RCLCPP_INFO(this->get_logger(), "Checking BOOL value for symbol: %s", value.c_str());

        // Create a request to check the boolean value
        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
        bool_req->path.task = "T_ROB1";
        bool_req->path.module = "Grinding";
        bool_req->path.symbol = value; 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this, value, weak_timer, response, next_func](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
                try {
                    auto result = future.get();

                    // Handle response
                    if (result->result_code != 1) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to check BOOL value. Return code: %d", result->result_code);
                        if (auto timer = weak_timer.lock()) {
                            timer->cancel();
                            flip_back_timer_busy = false;
                        }
                        return;
                    }

                    if (result->value) {
                        RCLCPP_INFO(this->get_logger(), "BOOL value is true. Moving on to next function and resetting to false afterwards.");

                        /*
                        
                        Bool value was set to true. Trigger next action (turn on grinder, or acf etc.)
                        
                        */
                        // Stop the timer and reset flag for next run.
                        if (auto timer = weak_timer.lock()) {
                            timer->cancel();
                            flip_back_timer_busy = false;

                        next_func(response);
                        
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "BOOL value is false. Rechecking...");

                        // Re-enable the timer 
                        flip_back_timer_busy = false;
                    }
                } catch (const std::exception &e) {
                    // RCLCPP_ERROR(this->get_logger(), "Exception while checking BOOL value: %s", e.what());
                    if (auto timer = weak_timer.lock()) {
                        timer->cancel();
                        flip_back_timer_busy = false;
                    }
                }
            });
    }


    void enableGrinder(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        RCLCPP_INFO(this->get_logger(), "Enabling grinder");
        // Now at home position. Spin up the grinder.
        auto grinder_req = std::make_shared<data_gathering_msgs::srv::StartGrinder::Request>();
        grinder_req->rpm = test_request_->rpm;
        grinder_req->timeout_duration = 0.0;

        start_grinder_client_->async_send_request(grinder_req, 
                [this,response](rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedFuture future){this->handleStartGrinderResponse(future, response);
        });
    }

    void handleStartGrinderResponse(rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        // Conditions for early return 
        try {
            auto result = future.get();
            if (!result->success) {
                test_response_->success = false;
                test_response_->message += "\nFailed to start grinder" + std::string(result->message.c_str());
                finalizeResponse(test_response_);
                return; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading start grinder response" + std::string(e.what());
            finalizeResponse(test_response_);
            return;
        }

        // Grinder was started, now move on to the next position by reseting the waiting_at_home bool
        resetBoolValue(symbol_home_flag, response,
            [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response) {
                this->handleResetHomeBoolResponse(future, response);
        });


    }

    void resetBoolValue(const std::string &value, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response,
            std::function<void(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response>)> next_func) {
        RCLCPP_INFO(this->get_logger(), "Resetting BOOL value for symbol: %s", value.c_str());

        auto reset_req = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
        reset_req->path.task = "T_ROB1";
        reset_req->path.module = "Grinding";
        reset_req->path.symbol = value;
        reset_req->value = false;

        rws_set_bool_client_->async_send_request(reset_req,
                [this, response, next_func](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){next_func(future, response);
        });
    }

    void handleResetHomeBoolResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
       // Conditions for early return 
        try {
            auto result = future.get();
            if (result->result_code != 1) {
                test_response_->success = false;
                test_response_->message += "Failed to reset bool: " + symbol_home_flag;
                finalizeResponse(test_response_);
                return; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading set bool response " + std::string(e.what());
            finalizeResponse(test_response_);
            return;
        }

        /*
        Moving from home to grind position 0
        Next wait until bool 'waiting_at_grind0' is set to True
        Initialize the timer for periodic BOOL checks
        */
        RCLCPP_INFO(this->get_logger(), "Waiting at home bool reset. Waiting to reach grind0");

        timer_wait_grind0_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, response]() {
                // Create weak_ptr from shared_ptr inside the callback
                auto weak_grind0_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_);

                // Try to lock the weak pointer
                if (auto timer = weak_grind0_timer.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_grind0_flag, 
                        weak_grind0_timer,             
                        std::bind(&RWSMotionClient::enableACF, this, response),
                        response                // Pass response
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Could not lock weak pointer");
                }
            }
        );
        }

        // auto weak_grind0_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_);
        // timer_wait_grind0_ = this->create_wall_timer(
        //     std::chrono::milliseconds(bool_timer_period_),
        //     [this, weak_grind0_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_), response]() {
        //         if (auto timer = weak_grind0_timer.lock()) {
        //             this->checkBoolValueFlipBack(
        //                 this->symbol_grind0_flag,   // Access via `this`
        //                 weak_grind0_timer,          // Pass the weak_timer
        //                 std::bind(&RWSMotionClient::enableACF, this, response), // Use a placeholder or actual std::function if needed
        //                 response                    // Pass response
        //             );
        //         }
        //     }
        // );
        // }
    
    void enableACF(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        RCLCPP_INFO(this->get_logger(), "Extending ACF");
        // publish acf
        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.data = test_request_->force;
        acf_force_publisher_->publish(acf_req);
        
        // request bool reset 
        // ACF was started, now move on to the next position by reseting the waiting_at_home bool
        resetBoolValue(symbol_grind0_flag, response,
            [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response) {
                this->handleResetGrind0BoolResponse(future, response);
        });
    }

    void handleResetGrind0BoolResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){

        /*
        
        - The system is currently grinding 
        - Next wait until bool 'finished_grind_pass' is set to True
        -> Initialize the timer for periodic BOOL checks

        */

        // Timer to check periodically whether the grind is done.
        // The timer calls checkBoolValueFlipBack. This function checks whether symbol_grind_done_flag is true. If this is the case, it resets it and calls grindPassDone.
        // auto weak_timer_grind_done = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done_);
        // timer_wait_grind_done_ = this->create_wall_timer(
        //     std::chrono::milliseconds(bool_timer_period_),
        //     [this, weak_timer_grind_done = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done_), response]() {
        //         if (auto timer = weak_timer_grind_done.lock()) {
        //             this->checkBoolValueFlipBack(
        //                 this->symbol_grind_done_flag,   
        //                 weak_timer_grind_done,          
        //                 std::bind(&RWSMotionClient::grindPassDone, this, response), 
        //                 response                  
        //             );
        //         }
        //     }
        // );
        timer_wait_grind_done_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, response]() {
                // Create weak_ptr from shared_ptr inside the callback
                auto weak_timer_grind_done = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done_);

                // Try to lock the weak pointer
                if (auto timer = weak_timer_grind_done.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_grind_done_flag, 
                        weak_timer_grind_done,             
                        std::bind(&RWSMotionClient::grindPassDone, this, response), 
                        response                // Pass response
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Could not lock weak pointer");
                }
            }
        );


        }

    void grindPassDone(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        
        // TODO turn off acf and grinder    
        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.data = -5.;
        acf_force_publisher_->publish(acf_req);

        // Now at home position. Spin up the grinder.
        auto grinder_off_req = std::make_shared<data_gathering_msgs::srv::StopGrinder::Request>(); 

        stop_grinder_client_->async_send_request(grinder_off_req, 
                [this,response](rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){this->handleGrinderOffResponse(future, response);
        });
    }

    void handleGrinderOffResponse(rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        // Conditions for early return 
        auto result = future.get();
        try {
            if (!result->success) {
                test_response_->success = false;
                test_response_->message += "Failed to turn off grinder: " + std::string(result->message.c_str());
                finalizeResponse(test_response_);
                return; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading turn off grinder response " + std::string(e.what());
            finalizeResponse(test_response_);
            return;
        }

        // Something went wrong during grinding 
        if (!result->grind_successful){
            test_response_->success = false;
            test_response_->message += result->grind_message.c_str();
        }

        // Wait until the script is done to finish the test service request 
        timer_wait_script_done_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, response]() {
                this->checkFinishedBool(response);
            });

    }

    void checkFinishedBool(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        
        // Stop timer to prevent functions being called in parallel
        if (finished_grind_timer_busy){
            RCLCPP_WARN(this->get_logger(), "Overlapping checkFinishedBool timer callbacl. Ignoring...");
            return;
        }
        finished_grind_timer_busy = true;

        RCLCPP_INFO(this->get_logger(), "Checking BOOL value for symbol: %s", symbol_run_status_flag.c_str());

        // Create a request to check the boolean value
        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();

        bool_req->path.task = "T_ROB1";
        bool_req->path.module = "Grinding";
        bool_req->path.symbol = symbol_run_status_flag; 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
                try {
                    auto result = future.get();

                    // Handle response
                    if (result->result_code != 1) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to check BOOL value. Return code: %d", result->result_code);
                        timer_wait_script_done_->cancel();
                        finished_grind_timer_busy = false; 
                        return;
                    }

                    if (!result->value) {
                        RCLCPP_INFO(this->get_logger(), "BOOL value is false. Finished service.");
                        // Finished. Stop timer and reset flag 
                        timer_wait_script_done_->cancel();
                        finished_grind_timer_busy = false; 


                    } else {
                        RCLCPP_WARN(this->get_logger(), "BOOL value is true. Rechecking...");
                        // The flag shows that the test is still running. Restart the timer to check whether it is done later. 
                        finished_grind_timer_busy = false;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception while checking BOOL value: %s", e.what());
                    timer_wait_script_done_->cancel();
                    finished_grind_timer_busy = false; 
                }
            });

    }

    void finalizeResponse(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response) {
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
