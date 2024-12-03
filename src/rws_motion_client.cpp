#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <abb_robot_msgs/srv/set_io_signal.hpp>
#include <abb_robot_msgs/srv/get_rapid_bool.hpp>
#include <abb_robot_msgs/srv/set_rapid_bool.hpp>
#include <abb_robot_msgs/srv/set_rapid_num.hpp>
#include <abb_robot_msgs/msg/rapid_symbol_path.hpp>
#include <abb_robot_msgs/srv/trigger_with_result_code.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

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
            std::bind(&RWSMotionClient::startMoveCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            // rmw_qos_profile_services_default,  // Third argument: QoS profile
            // this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive) // Fourth argument: Callback group
        // );

        rws_set_bool_client_        = this->create_client<abb_robot_msgs::srv::SetRAPIDBool>("/rws_client/set_rapid_bool");
        rws_get_bool_client_        = this->create_client<abb_robot_msgs::srv::GetRAPIDBool>("/rws_client/get_rapid_bool");
        rws_pp_to_main_client_      = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/pp_to_main");
        rws_start_rapid_client_     = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/start_rapid");
        rws_set_num_client_         = this->create_client<abb_robot_msgs::srv::SetRAPIDNum>("/rws_client/set_rapid_num");
        rws_sys_states_sub_         = this->create_subscription<abb_robot_msgs::msg::SystemState>("/rws_client/system_states", 10,
                                                                                                  std::bind(&RWSMotionClient::systemStateCallback, this, std::placeholders::_1));

        start_grinder_client_       = this->create_client<data_gathering_msgs::srv::StartGrinder>("/grinder_node/enable_grinder");
        stop_grinder_client_        = this->create_client<data_gathering_msgs::srv::StopGrinder>("/grinder_node/disable_grinder");
        acf_force_publisher_        = this->create_publisher<stamped_std_msgs::msg::Float32Stamped>("/acf/force", 10);


        // bool_symbol_req_default.path.task = "T_ROB1";
        // bool_symbol_req_default.path.module = "Grinding";

        symbol_home_flag = "waiting_at_home";
        symbol_grind0_flag = "waiting_at_grind0";
        symbol_grind_done_flag = "finished_grind_pass";
        symbol_run_status_flag = "run_status";
        symbol_nr_passes = "num_pass";
        symbol_tcp_speed = "tcp_feedrate";

        estop_triggered_ = false;
        service_available = true; 
        bool_timer_period_ = 100; // Milliseconds 
        flip_back_timer_busy = false;
        finished_grind_timer_busy = false;
        max_tcp_speed = 30.; // TODO MAKE THIS A PARAMETER
        grinder_spinup_duration = 4; //seconds 

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
    bool success_;

    std::shared_ptr<rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>> test_service_;
    std::shared_ptr<rmw_request_id_t> test_request_header_;
    std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Request> test_request_;
    std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> test_response_;

    // rws interfaces 
    rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_pp_to_main_client_;
    rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_start_rapid_client_;
        // rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr rws_set_io_client_;
    rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedPtr rws_get_bool_client_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr rws_set_bool_client_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDNum>::SharedPtr rws_set_num_client_;
    rclcpp::Subscription<abb_robot_msgs::msg::SystemState>::SharedPtr rws_sys_states_sub_;
    
    rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedPtr start_grinder_client_;
    rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedPtr stop_grinder_client_;

    rclcpp::Publisher<stamped_std_msgs::msg::Float32Stamped>::SharedPtr acf_force_publisher_;

    // Bool request with module and task preset
    abb_robot_msgs::srv::GetRAPIDBool::Request bool_symbol_req_default;

    rclcpp::TimerBase::SharedPtr timer_wait_home_;
    rclcpp::TimerBase::SharedPtr timer_wait_grind0_;
    rclcpp::TimerBase::SharedPtr timer_wait_grind_done_;
    rclcpp::TimerBase::SharedPtr timer_wait_script_done_;
    rclcpp::TimerBase::SharedPtr timer_wait_grinder_spinup_;

    std::string symbol_home_flag;
    std::string symbol_grind0_flag;
    std::string symbol_grind_done_flag;
    std::string symbol_run_status_flag; 
    std::string symbol_nr_passes;
    std::string symbol_tcp_speed;
    
    bool estop_triggered_;
    bool service_available;
    int bool_timer_period_;
    bool flip_back_timer_busy;
    bool finished_grind_timer_busy;
    double max_tcp_speed;
    int grinder_spinup_duration;

    void startMoveCallback(const std::shared_ptr<rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>> service,
                           const std::shared_ptr<rmw_request_id_t> request_header,
                           const data_gathering_msgs::srv::StartGrindTest::Request::SharedPtr request) {

        // Create response
        auto response = std::make_shared<data_gathering_msgs::srv::StartGrindTest::Response>();
        response->success = true;  // To be set to false if anything goes wrong

        if (!service_available){
            RCLCPP_WARN(this->get_logger(), "The service is not available. It is either already running, or 'service_available' was not reset correctly");
            response->success = false;
            response->message = "The service was not available.";
            service->send_response(*request_header, *response);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Start test request received");
        if (request->tcp_speed > max_tcp_speed){
            RCLCPP_WARN(
                this->get_logger(),
                "The requested TCP feedrate of [%.2f] exceeds the maximum of [%.2f]",
                request->tcp_speed,
                max_tcp_speed);

            RCLCPP_WARN(this->get_logger(), "Ignoring request");
            response->success = false;
            response->message = "The requested TCP speed exceeds the maximum of " + std::to_string(max_tcp_speed);
            service->send_response(*request_header, *response);
            return;
        }

        service_available = false;

        // Check to ensure that ABB is not already running RAPID 
        auto is_running_bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
        is_running_bool_req->path.task = "T_ROB1";
        is_running_bool_req->path.module = "Grinding";
        is_running_bool_req->path.symbol = symbol_run_status_flag;

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        rws_get_bool_client_->async_send_request(is_running_bool_req, 
        [this, service, request_header, request, response](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {
            this->handleRunStatusResponse(future, service, request_header, request, response);
        });

    }

    void handleRunStatusResponse(rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future, 
                                 const std::shared_ptr<rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>> service,
                                 const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Request> request,
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
                    finalizeResponse(service, request_header, response);
                    return; 
                }

            } else {
                response->success = false;
                response->message += "Failed to read run_status bool";
                finalizeResponse(service, request_header, response);
                return;
            }
            
        } catch (const std::exception &e) {
            response->success = false;
            response->message += "Error reading run_status bool: "  + std::string(e.what());
            finalizeResponse(service, request_header, response);
            return;
        }

        // There is no current program running -> Proceed 
        RCLCPP_INFO(this->get_logger(), "ABB Ready...");

        // Reset flags and save the request params. 
        success_ = true;
        message_.clear();
        test_service_           = service;
        test_request_header_    = request_header;
        test_request_           = request;   
        test_response_          = response;

        RCLCPP_INFO(this->get_logger(), "Requesting PP to main...");

        auto pp_to_main_req = std::make_shared<abb_robot_msgs::srv::TriggerWithResultCode::Request>();
    
        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }
        rws_pp_to_main_client_->async_send_request(pp_to_main_req,
            [this](rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future){this->handlePPToMainResponse(future);
        });
    }

    void handlePPToMainResponse(rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future){

        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"request pp_to_main");
        if (!successful){return;}

        RCLCPP_INFO(this->get_logger(), "Setting number of passes");
        // PP was successfully set to main. Set speed and nr of passes

        auto req_passes = std::make_shared<abb_robot_msgs::srv::SetRAPIDNum::Request>();
        req_passes->path.task = "T_ROB1";
        req_passes->path.module = "Grinding";
        req_passes->path.symbol = symbol_nr_passes;
        req_passes->value = test_request_->passes;


        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        rws_set_num_client_->async_send_request(req_passes,
                [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDNum>::SharedFuture future){this->handleSetNrPassesResponse(future);}
        );
    }

    void handleSetNrPassesResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDNum>::SharedFuture future){

        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"setting number of passes");
        if (!successful){return;}

        // Set the number of passes, now set the tcp speed. 
        RCLCPP_INFO(this->get_logger(), "Setting TCP feedrate");
        auto req_passes = std::make_shared<abb_robot_msgs::srv::SetRAPIDNum::Request>();
        req_passes->path.task = "T_ROB1";
        req_passes->path.module = "Grinding";
        req_passes->path.symbol = symbol_tcp_speed;

        req_passes->value = test_request_->tcp_speed;


        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        rws_set_num_client_->async_send_request(req_passes,
                [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDNum>::SharedFuture future){this->handleSetTCPSpeedResponse(future);}
        );
    }
     void handleSetTCPSpeedResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDNum>::SharedFuture future){
        
        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"setting TCP speed");
        if (!successful){return;}

        // The variables are set, now the RAPID program can be started.
        
        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        startRAPID();
    }

    void startRAPID(){
        RCLCPP_INFO(this->get_logger(), "\n\nStarting RAPID\n");

        auto request = std::make_shared<abb_robot_msgs::srv::TriggerWithResultCode::Request>();

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        rws_start_rapid_client_->   async_send_request(request,
                [this](rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future){this->handleStartRapidResponse(future);
        });
     }
    
    void handleStartRapidResponse(rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedFuture future){

        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"requesting start RAPID");
        if (!successful){return;}

        RCLCPP_INFO(this->get_logger(), "Moving to home...");


        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        // RAPID Successfully started
        // Next wait until bool 'waiting_at_home' is set to True
        // Initialize the timer for periodic BOOL checks
        timer_wait_home_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this]() {
                // Create weak_ptr from shared_ptr inside the callback
                auto weak_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_home_);

                // Try to lock the weak pointer
                if (auto timer = weak_timer.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_home_flag, 
                        weak_timer,             
                        std::bind(&RWSMotionClient::enableGrinder, this)
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Could not lock weak pointer");
                }
            }
        );

    }

    void checkBoolValueFlipBack(const std::string &value,  const std::weak_ptr<rclcpp::TimerBase> weak_timer, 
            std::function<void()> next_func) {
        /*

        Function called by timer to periodically check a bool from RWS until it is found to be True.
        Once this is the case, it is flipped back to false, and the timer is stopped. 

        */


        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        // Check and set flag so that the timer won't call this function again while this one is still in progress.
        if (flip_back_timer_busy){
            RCLCPP_WARN(this->get_logger(), "Overlapping timer callback for checkBoolValueFlipBack. Ignoring...");    
            return;
        }

        flip_back_timer_busy= true;

        // RCLCPP_INFO(this->get_logger(), "Checking BOOL value for symbol: %s", value.c_str());

        // Create a request to check the boolean value
        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();
        bool_req->path.task = "T_ROB1";
        bool_req->path.module = "Grinding";
        bool_req->path.symbol = value; 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this, value, weak_timer, next_func](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {

                if (this->estop_triggered_) {
                    RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
                    return;
                }

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

                        next_func();                        
                        }
                    } else {
                        // RCLCPP_WARN(this->get_logger(), "BOOL value is false. Rechecking...");

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


    void enableGrinder(){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Enabling grinder");
        // Now at home position. Spin up the grinder.
        auto grinder_req = std::make_shared<data_gathering_msgs::srv::StartGrinder::Request>();
        grinder_req->rpm = test_request_->rpm;
        grinder_req->timeout_duration = 0.0;

        start_grinder_client_->async_send_request(grinder_req, 
                [this](rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedFuture future){this->handleStartGrinderResponse(future);
        });
    }

    void handleStartGrinderResponse(rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedFuture future){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }
        
        // Conditions for early return 
        try {
            auto result = future.get();
            if (!result->success) {
                test_response_->success = false;
                test_response_->message += "\nFailed to start grinder" + std::string(result->message.c_str());
                finalizeResponse(test_service_, test_request_header_, test_response_);
                return; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading start grinder response" + std::string(e.what());
            finalizeResponse(test_service_, test_request_header_, test_response_);
            return;
        }

        // Grinder was started, now move on to the next position by reseting the waiting_at_home bool
        // First wait for a couple of seconds for the grinder to spin up
        timer_wait_grinder_spinup_ = this->create_wall_timer(
            std::chrono::seconds(grinder_spinup_duration),
            [this]() {
                
                if (estop_triggered_) {
                    RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
                    return;
                }
                // Spinup duration elapsed. Cancel timer 
                this->timer_wait_grinder_spinup_->cancel();

                RCLCPP_INFO(this->get_logger(), "Grinder spin-up duration elapsed. Moving to first grind position.");

                // Reset the wait at home flag, to move the grinder to the next position.
                resetBoolValue(symbol_home_flag,
                    [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future) {
                    this->handleResetHomeBoolResponse(future);
                });
            }
        );
    }

    void resetBoolValue(const std::string &value,
            std::function<void(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture)> next_func) {
        RCLCPP_INFO(this->get_logger(), "Resetting BOOL value for symbol: %s", value.c_str());


        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        auto reset_req = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
        reset_req->path.task = "T_ROB1";
        reset_req->path.module = "Grinding";
        reset_req->path.symbol = value;
        reset_req->value = false;

        rws_set_bool_client_->async_send_request(reset_req,
                [this, next_func](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){next_func(future);
        });
    }

    void handleResetHomeBoolResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"resetting bool " + symbol_home_flag);
        if (!successful){return;}

        /*
        Moving from home to grind position 0
        Next wait until bool 'waiting_at_grind0' is set to True
        Initialize the timer for periodic BOOL checks
        */
        RCLCPP_INFO(this->get_logger(), "Waiting at home bool reset. Waiting to reach grind0");

        timer_wait_grind0_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this]() {
                // Create weak_ptr from shared_ptr inside the callback
                auto weak_grind0_timer = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind0_);

                // Try to lock the weak pointer
                if (auto timer = weak_grind0_timer.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_grind0_flag, 
                        weak_grind0_timer,             
                        std::bind(&RWSMotionClient::enableACF, this)
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Could not lock weak pointer");
                }
            }
        );
    }

    
    void enableACF(){
        RCLCPP_INFO(this->get_logger(), "Extending ACF");
        
        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.data = test_request_->force;
        acf_force_publisher_->publish(acf_req);
        
        // request bool reset 
        // ACF was started, now move on to the next position by reseting the waiting_at_home bool
        resetBoolValue(symbol_grind0_flag,
            [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future) {
                this->handleResetGrind0BoolResponse(future);
        });
    }

    void handleResetGrind0BoolResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){
        /*
        
        - The system is currently grinding 
        - Next wait until bool 'finished_grind_pass' is set to True
        -> Initialize the timer for periodic BOOL checks

        */

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"resetting bool " + symbol_grind0_flag);
        if (!successful){return;}

        timer_wait_grind_done_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this]() {
                // Create weak_ptr from shared_ptr inside the callback
                auto weak_timer_grind_done = std::weak_ptr<rclcpp::TimerBase>(timer_wait_grind_done_);

                // Try to lock the weak pointer
                if (auto timer = weak_timer_grind_done.lock()) {
                    this->checkBoolValueFlipBack(
                        this->symbol_grind_done_flag, 
                        weak_timer_grind_done,             
                        std::bind(&RWSMotionClient::grindPassDone, this)
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Could not lock weak pointer to the input timer.");
                }
            }
        );

    }

    void grindPassDone(){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }
        
        // TODO turn off acf and grinder    
        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.data = -5.;
        acf_force_publisher_->publish(acf_req);

        auto grinder_off_req = std::make_shared<data_gathering_msgs::srv::StopGrinder::Request>(); 

        stop_grinder_client_->async_send_request(grinder_off_req, 
                [this](rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){this->handleGrinderOffResponse(future);
        });
    }

    void handleGrinderOffResponse(rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        // Conditions for early return 
        auto result = future.get();
        try {
            if (!result->success) {
                test_response_->success = false;
                test_response_->message += "Failed to turn off grinder: " + std::string(result->message.c_str());
                finalizeResponse(test_service_, test_request_header_, test_response_);
                return; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading turn off grinder response " + std::string(e.what());
            finalizeResponse(test_service_, test_request_header_, test_response_);
            return;
        }

        // Something went wrong during grinding 
        if (!result->grind_successful){
            test_response_->success = false;
            test_response_->message += result->grind_message.c_str();
        }

        resetBoolValue(symbol_grind_done_flag, 
            [this](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future) {
                    this->handleResetGrindDoneResponse(future);
            }
        );
    }

    void handleResetGrindDoneResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }

        auto successful = checkABBResponse(future ,"resetting bool " + symbol_grind_done_flag);
        if (!successful){return;}

        // Wait until the script is done to finish the test service request 
        timer_wait_script_done_ = this->create_wall_timer(
            std::chrono::milliseconds(bool_timer_period_),
            [this](){
                this->checkFinishedBool();
        });
    }
 

    void checkFinishedBool(){

        if (estop_triggered_) {
            RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
            return;
        }
        
        // Stop timer to prevent functions being called in parallel
        if (finished_grind_timer_busy){
            RCLCPP_WARN(this->get_logger(), "Overlapping checkFinishedBool timer callback. Ignoring...");
            return;
        }
        finished_grind_timer_busy = true;

        // RCLCPP_INFO(this->get_logger(), "Checking BOOL value for symbol: %s", symbol_run_status_flag.c_str());

        // Create a request to check the boolean value
        auto bool_req = std::make_shared<abb_robot_msgs::srv::GetRAPIDBool::Request>();

        bool_req->path.task = "T_ROB1";
        bool_req->path.module = "Grinding";
        bool_req->path.symbol = symbol_run_status_flag; 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {

                if (estop_triggered_) {
                    RCLCPP_WARN(this->get_logger(), "E-stop is active. Aborting operation.");
                    return;
                }
                
                // Check whether request was successful 
                auto successful = this->checkABBResponse(future, "reading bool value " + symbol_run_status_flag);
                if (!successful){
                    timer_wait_script_done_->cancel();
                    finished_grind_timer_busy = false; 
                    return;
                }

                auto result = future.get();
                if (!result->value) {
                    RCLCPP_INFO(this->get_logger(), "BOOL value is false. Finished service.");
                    // Finished. Stop timer and reset flag 
                    timer_wait_script_done_->cancel();
                    finished_grind_timer_busy = false; 
                    this->finalizeResponse(this->test_service_, this->test_request_header_, this->test_response_);

                } else {
                    // RCLCPP_WARN(this->get_logger(), "BOOL value is true. Rechecking...");
                    // The flag shows that the test is still running. Restart the timer to check whether it is done later. 
                    finished_grind_timer_busy = false;
                }

            }
        );
    }

    template <typename abb_future>
    bool checkABBResponse(abb_future future, std::string task_message){
        try {
            auto result = future.get();
            if (result->result_code != 1) {
                test_response_->success = false;
                test_response_->message += "Failed to perform task: " + task_message;
                finalizeResponse(test_service_, test_request_header_, test_response_);
                return false; 
            }
        } catch (const std::exception &e) {
            test_response_->success = false;
            test_response_->message += "Error reading response from task: " + task_message + " -- " + std::string(e.what());
            finalizeResponse(test_service_, test_request_header_, test_response_);
            return false;
        }
        return true;
    }

    void finalizeResponse(std::shared_ptr<rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>> service,
                          std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> response) {
        
        // Finish the main service callback by sending back a response. 
        RCLCPP_INFO(this->get_logger(), "Sending response: success=%d, message='%s'", response->success, response->message.c_str());
        service->send_response(*request_header, *response);
        service_available = true; 
    }

    void systemStateCallback(const abb_robot_msgs::msg::SystemState& msg) {
        if (service_available){
            return; // Ignore message if nothing is executing 
        }
        if (!msg.motors_on){
            estop_triggered_ = true; 

            // retract acf
            auto acf_req = stamped_std_msgs::msg::Float32Stamped();
            acf_req.data = -15.;
            acf_force_publisher_->publish(acf_req);

            // request grinder off
            auto grinder_off_req = std::make_shared<data_gathering_msgs::srv::StopGrinder::Request>(); 
            stop_grinder_client_->async_send_request(grinder_off_req, 
                [this](rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){this->handleGrinderOffResponse(future);
            });

            RCLCPP_INFO(this->get_logger(), "E-STOP TRIGGERED. Send requests to retract the ACF and turn off the grinder.");
            // cut off the callback chain process
            // finalize response
            test_response_->success = false;
            test_response_->message += "\nE-stop triggered.\n";
            finalizeResponse(test_service_, test_request_header_, test_response_);
        }

    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RWSMotionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
