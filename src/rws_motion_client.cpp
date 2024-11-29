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
        rws_pp_start_rapid_client_  = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/start_RAPID");

        start_grinder_client_       = this->create_client<data_gathering_msgs::srv::StartGrinder>("/grinder_node/start_grinder");
        stop_grinder_client_        = this->create_client<data_gathering_msgs::srv::StopGrinder>("/grinder_node/stop_grinder");
        acf_force_publisher_        = this->create_publisher<stamped_std_msgs::msg::Float32Stamped>("/acf/force", 10);

        bool_symbol_req_default.path.task = "TROB1";
        bool_symbol_req_default.path.module = "Grinding";

        symbol_home_flag = "waiting_at_home";
        symbol_grind0_flag = "waiting_at_grind0";
        symbol_grind_done_flag = "finished_grind_pass";
        symbol_run_status_flag = "run_status";

        bool_timer_period_ = 250; // Milliseconds 

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

    void startMoveCallback(const data_gathering_msgs::srv::StartGrindTest::Request::SharedPtr request,
                           data_gathering_msgs::srv::StartGrindTest::Response::SharedPtr response) {
        RCLCPP_INFO(this->get_logger(), "Start test request received");

        // CHECK WHETHER ABB IS ALREADY RUNNING RAPID 
        auto is_running_bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
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
            response->message += "Error reading run_status bool: "; //+ std::string(e.what());
            finalizeResponse(response);
            return;
        }

        // There is no current program running -> Proceed 

        // Reset flags and save response 
        success_ = true;
        message_.clear();
        test_request_ = std::make_shared<data_gathering_msgs::srv::StartGrindTest::Request>(request);
        test_response_ = std::make_shared<data_gathering_msgs::srv::StartGrindTest::Response>(response);


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

        // RAPID Successfully started
        // Next wait until bool 'waiting_at_home' is set to True
        // Initialize the timer for periodic BOOL checks
        auto weak_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_home_);
        timer_wait_home_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, weak_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_home_), response]() {
                if (auto timer = weak_timer.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_home_flag, // Access via `this`
                        weak_timer,             // Pass the weak_timer
                        std::bind(&RWSMotionClient::enableGrinder, this, response),    // Use a placeholder or actual std::function if needed
                        response                // Pass response
                    );
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

        RCLCPP_INFO(this->get_logger(), "Checking BOOL value for symbol: %s", value.c_str());

        // Create a request to check the boolean value
        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
        bool_req->path.task = "TROB1";
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
                        }
                        return;
                    }

                    if (result->value) {
                        RCLCPP_INFO(this->get_logger(), "BOOL value is true. Resetting to false.");

                        /*
                        
                        Bool value was set to true. Trigger next action (turn on grinder, or acf etc.)
                        
                        */
                        // Stop the timer
                        if (auto timer = weak_timer.lock()) {
                            timer->cancel();

                        next_func(response);
                        
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "BOOL value is false. Rechecking...");
                    }
                } catch (const std::exception &e) {
                    // RCLCPP_ERROR(this->get_logger(), "Exception while checking BOOL value: %s", e.what());
                    if (auto timer = weak_timer.lock()) {
                        timer->cancel();
                    }
                }
            });
    }


    void enableGrinder(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
            
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
        reset_req->path.task = "TROB1";
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


        // Moving from home to grind position 0
        // Next wait until bool 'waiting_at_grind0' is set to True
        // Initialize the timer for periodic BOOL checks
        // timer_wait_grind0_ = this->create_wall_timer(
        //     std::chrono::milliseconds(bool_timer_period_),
        //     std::bind(&RWSMotionClient::checkBoolValueFlipBack, 
        //       this, 
        //       symbol_grind0_flag, 
        //       std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_), 
        //       std::bind(&RWSMotionClient::enableACF, this),
        //       response));
        
        auto weak_grind0_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_);
        timer_wait_grind0_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, weak_grind0_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_), response]() {
                if (auto timer = weak_grind0_timer.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_grind0_flag,   // Access via `this`
                        weak_grind0_timer,          // Pass the weak_timer
                        std::bind(&RWSMotionClient::enableACF, this, response), // Use a placeholder or actual std::function if needed
                        response                    // Pass response
                    );
                }
            }
        );
        }
    
    void enableACF(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        
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

        // The system is currently grinding 
        // Next wait until bool 'finished_grind_pass' is set to True
        // Initialize the timer for periodic BOOL checks
    //     timer_wait_grind_done_ = this->create_wall_timer(
    //         std::chrono::milliseconds(bool_timer_period_),
    //         std::bind(&RWSMotionClient::checkBoolValueFlipBack, 
    //           this, 
    //           symbol_grind_done_flag, 
    //           std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done), 
    //           std::bind(&RWSMotionClient::grindPassDone, this),
    //           response));
    // }

        auto weak_timer_grind_done = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done_);
        timer_wait_grind_done_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this, weak_timer_grind_done = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done_), response]() {
                if (auto timer = weak_timer_grind_done.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_grind_done_flag,   // Access via `this`
                        weak_timer_grind_done,          // Pass the weak_timer
                        std::bind(&RWSMotionClient::grindPassDone, this, response), // Use a placeholder or actual std::function if needed
                        response                    // Pass response
                    );
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
        auto grinder_off_req = std::make_shared<data_gathering_msgs::srv::StopGrinder>(); 

        stop_grinder_client_->async_send_request(grinder_off_req, 
                [this,response](rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){this->handleGrinderOffResponse(future, response);
        });
    }

    void handleGrinderOffResponse(rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future, std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        // Conditions for early return 
        try {
            auto result = future.get();
            if (!result->success) {
                test_response_->success = false;
                test_response_->message += "Failed to turn off grinder: " + result->message.c_str();
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
        timer_wait_script_done = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            std::bind(&RWSMotionClient::checkFinishedBool, 
            this,
            response));

    }

    void checkFinishedBool(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response){
        
        RCLCPP_INFO(this->get_logger(), "Checking BOOL value for symbol: " + symbol_run_status_flag);

        // Create a request to check the boolean value
        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
        bool_req->path.task = "TROB1";
        bool_req->path.module = "Grinding";
        bool_req->symbol = symbol_run_status_flag; 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this, value](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
                try {
                    auto result = future.get();

                    // Handle response
                    if (result->result_code != 1) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to check BOOL value. Return code: %d", result->result_code);
                        timer_wait_script_done.cancel();
                        return;
                    }

                    if (!result->value) {
                        RCLCPP_INFO(this->get_logger(), "BOOL value is false. Finished service.");

                        timer_wait_script_done.cancel();


                    } else {
                        RCLCPP_WARN(this->get_logger(), "BOOL value is true. Rechecking...");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception while checking BOOL value: %s", e.what());
                    timer_wait_script_done.cancel();
                }
            });

    }

    void finalizeResponse(std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Response: success=%d, message='%s'", response->success, response->message.c_str());
    }



















//     void handleIOSet1Response(rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future,
//                               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//         try {
//             auto result = future.get();
//             if (result->result_code == 1) {
//                 RCLCPP_INFO(this->get_logger(), "IO signal set to 1 successfully");
//                 check_in_progress_ = true; // Start periodic BOOL check
//             } else {
//                 success_ = false;
//                 message_ += "Failed to set IO signal to 1. ";
//                 finalizeResponse(response);
//             }
//         } catch (const std::exception &e) {
//             success_ = false;
//             message_ += "Error setting IO signal: " + std::string(e.what());
//             finalizeResponse(response);
//         }
//     }

//     void checkBoolValue() {
//         if (!check_in_progress_) return;

//         RCLCPP_INFO(this->get_logger(), "Checking BOOL value");

//         auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>(bool_symbol_req_);

//         rws_get_bool_client_->async_send_request(bool_req,
//             [this](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
//                 try {
//                     auto result = future.get();
//                     if (result ->result_code != 1){
//                       check_in_progress_ = false;
//                       test_response_->success = false;
//                       test_response_->message = "Failed to check bool value";
//                       RCLCPP_INFO(this->get_logger(), "Failed to check bool value - return code: %s", result->message.c_str());
//                       finalizeResponse(test_response_);
//                       return;

//                     }
//                     if (result->value) {
//                         RCLCPP_INFO(this->get_logger(), "BOOL value is true");
//                         check_in_progress_ = false;
//                         test_response_->success = true;
                        
//                         resetIO(test_response_);

//                     } else {
//                         RCLCPP_WARN(this->get_logger(), "BOOL value is set to false: -- rechecking");
//                     }
//                 } catch (const std::exception &e) {
//                     RCLCPP_ERROR(this->get_logger(), "Error checking BOOL value: %s", e.what());
//                     check_in_progress_ = false;
//                     test_response_->success = false;
//                     test_response_->message = "Exception occurred while checking BOOL value";
//                     finalizeResponse(test_response_);
//                 }
//             });
//     }


//     void resetIO(std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//         RCLCPP_INFO(this->get_logger(), "Resetting IO");

//         auto io_req = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
//         io_req->signal = "startRoutineSignal";
//         io_req->value = "0";

//         rws_set_io_client_->async_send_request(io_req,
//             [this, response](rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future) {
//                 this->handleIOResetResponse(future, response);
//             });
//     }

//     void handleIOResetResponse(
//         rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedFuture future,
//         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
//         try {
//             auto result = future.get();
//             if (result->result_code == 1) { // Assuming 'result_code' is the field
//                 RCLCPP_INFO(this->get_logger(), "IO reset successfully");
//                 response->success = true;
//                 response->message = "IO reset successfully";
//             } else {
//                 RCLCPP_WARN(this->get_logger(), "Failed to reset IO");
//                 response->success = false;
//                 response->message = "Failed to reset IO";
//             }
//         } catch (const std::exception &e) {
//             RCLCPP_ERROR(this->get_logger(), "Error resetting IO: %s", e.what());
//             response->success = false;
//             response->message = "Exception occurred while resetting IO";
//         }
//     }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RWSMotionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
