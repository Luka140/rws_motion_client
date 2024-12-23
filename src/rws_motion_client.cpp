#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
// #include <abb_robot_msgs/srv/set_io_signal.hpp>
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
#include <data_gathering_msgs/msg/grind_settings.hpp>
#include <ferrobotics_acf/msg/acf_telem_stamped.hpp>

#include <string>


/*
This node interacts with a RAPID program by waiting for booleans in the RAPID script, performing the required action, and then flipping the boolean back.
For this to work correctly, make sure that the robot is NOT IN CONTINUOUS MODE. 

The flow is as follows:

1.  Wait for a test service call
	- This includes: force, RPM, tcp speed, passes

2. Check if currently running. 
    This is first done with a boolean flag in this node (service available).
    If this is true, the 'symbol_run_status_flag' bool in RAPID is checked. If this is false proceed, otherwise reject the request.

3. use '/rws_client/pp_to_main' to set the program pointer to the start of main

4. Set the params for the number of passes ('symbol_nr_passes') and the TCP speed ('symbol_tcp_speed')  based on the values in the request.

5. Start the RAPID program with '/rws_client/start_rapid'

6. Wait until the home position is reached.
    This is done by checking the 'symbol_home_flag' bool on a timer. When the bool turns true, the robot has arrived at home and will wait there until it is set to false again.

7. Start grinder with desired rpm ('/grinder_node/enable_grinder') while still at the home position. 
    Then wait for 'grinder_spinup_duration' number of seconds for the grinder to spin up.

8. Reset the 'symbol_home_flag' (from step 6.) to false again. 
    The robot will now proceed to move to the first grind position. 

9. Wait until the first grinding position is reached 
    This is done by checking the 'symbol_grind0_flag' bool on a timer. When the bool turns true, the robot has arrived at the first grinding position and will wait there until it is set to false again.

10. Extend the ACF by publishing the requested grinding force on '/acf/force'.

11. Reset the 'symbol_grind0_flag' (from step 8.) to false again. 
    The robot will now execute the grinding pass for the requested number of times. 

12. Wait until the robot is done grinding. 
    This is done by checking the 'symbol_grind_done_flag' bool on a timer. When the bool turns true, the robot has finished grinding. It will NOT wait in this position, but immediately move to the home position.

13. Retract the grinder ('/grinder_node/disable_grinder') and turn off the ACF ('/acf/force'). Then reset the 'symbol_grind_done_flag' boolean to false.

14. Wait until the robot has reached the home position again. 
    This is done by checking the 'symbol_run_status' bool on a timer. When this bool is false, the entire RAPID script has been executed. When this occurs, send back the service response. 

*/


class RWSMotionClient : public rclcpp::Node {
public:
    RWSMotionClient() : Node("rws_motion_client"){

        exclusive_group     = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        start_move_service_ = this->create_service<data_gathering_msgs::srv::StartGrindTest>(
            "~/start_grind_move",
            std::bind(&RWSMotionClient::startMoveCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            rmw_qos_profile_services_default,
            exclusive_group 
        );          // TODO ALL THE TIMERS CAN BE SET TO MUTUALLY EXCLUSIVE CALLBACKS

        rws_set_bool_client_        = this->create_client<abb_robot_msgs::srv::SetRAPIDBool>("/rws_client/set_rapid_bool");
        rws_get_bool_client_        = this->create_client<abb_robot_msgs::srv::GetRAPIDBool>("/rws_client/get_rapid_bool");
        rws_pp_to_main_client_      = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/pp_to_main");
        rws_start_rapid_client_     = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/start_rapid");
        // rws_stop_rapid_client_      = this->create_client<abb_robot_msgs::srv::TriggerWithResultCode>("/rws_client/stop_rapid");    // Only used to stop program when issues arise
        rws_set_num_client_         = this->create_client<abb_robot_msgs::srv::SetRAPIDNum>("/rws_client/set_rapid_num");
        rws_sys_states_sub_         = this->create_subscription<abb_robot_msgs::msg::SystemState>("/rws_client/system_states", 10,
                                                                                                  std::bind(&RWSMotionClient::systemStateCallback, this, std::placeholders::_1));
        acf_telem_sub_              = this->create_subscription<ferrobotics_acf::msg::ACFTelemStamped>("/acf/telem", 1,
                                                                                                  std::bind(&RWSMotionClient::acfTelemCallback, this, std::placeholders::_1));                                                                                                  

        start_grinder_client_       = this->create_client<data_gathering_msgs::srv::StartGrinder>("/grinder_node/enable_grinder");
        stop_grinder_client_        = this->create_client<data_gathering_msgs::srv::StopGrinder>("/grinder_node/disable_grinder");
        acf_force_publisher_        = this->create_publisher<stamped_std_msgs::msg::Float32Stamped>("/acf/force", 10);
        settings_publisher_         = this->create_publisher<data_gathering_msgs::msg::GrindSettings>("~/grind_settings", 10);

        // Parameters for symbol names as they are set in the RAPID code.
        symbol_home_flag        = this->declare_parameter<std::string>("symbol_home_flag", "waiting_at_home");              // Flag for home position
        symbol_grind0_flag      = this->declare_parameter<std::string>("symbol_grind0_flag", "waiting_at_grind0");          // Flag for position where the grinder makes first contact
        symbol_grind_done_flag  = this->declare_parameter<std::string>("symbol_grind_done_flag", "finished_grind_pass");    // Flag for position where the grinder makes last contact
        symbol_run_status_flag  = this->declare_parameter<std::string>("symbol_run_status_flag", "run_status");             // Flag for indicating whether the RAPID script is running
        symbol_nr_passes        = this->declare_parameter<std::string>("symbol_nr_passes", "num_pass");                     // Name of the variable which sets the number of grinder passes
        symbol_tcp_speed        = this->declare_parameter<std::string>("symbol_tcp_speed", "tcp_feedrate");                 // Name of the variable which sets the TCP feedrate

        bool_timer_period_      = this->declare_parameter<int>("bool_timer_period", 100);       // Period of the timer checking bools in RAPID [ms]
        max_tcp_speed           = this->declare_parameter<double>("max_tcp_speed", 35.0);       // The maximum allowed TCP speed [mm/s]. Requests with higher speeds will be rejected
        grinder_spinup_duration = this->declare_parameter<int>("grinder_spinup_duration", 4);   // Duration waiting for the grinder to spin up before moving to the first grind position [s]
        max_acf_extension       = this->declare_parameter<double>("max_acf_extension", 34.5);    // The maximum allowed ACF extension [mm]. If exceeded the test is aborted.

        // Initialize flags 
        test_aborted_               = false;
        service_available           = true; 
        flip_back_timer_busy        = false;
        finished_grind_timer_busy   = false;

        // Create default path to the grinding module where all the symbols above are stored.
        default_path_.task   = "T_ROB1";
        default_path_.module = "Grinding";

        // Retract the ACF before starting
        stamped_std_msgs::msg::Float32Stamped retract_message;
        retract_message.data = -5.0;
        retract_message.header.stamp = this->get_clock()->now(); 
        acf_force_publisher_->publish(retract_message);

        RCLCPP_INFO(this->get_logger(), "RWS Motion Client initialized");
    }

private:
    rclcpp::CallbackGroup::SharedPtr exclusive_group;

    // Main service to perform a test and response
    rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>::SharedPtr start_move_service_;
    std::string message_;
    bool success_;

    // The main service
    std::shared_ptr<rclcpp::Service<data_gathering_msgs::srv::StartGrindTest>> test_service_;
    std::shared_ptr<rmw_request_id_t> test_request_header_;
    std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Request> test_request_;
    std::shared_ptr<data_gathering_msgs::srv::StartGrindTest::Response> test_response_;

    // RWS interfaces 
    rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_pp_to_main_client_;
    rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_start_rapid_client_;
    // rclcpp::Client<abb_robot_msgs::srv::TriggerWithResultCode>::SharedPtr rws_stop_rapid_client_;
    rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedPtr rws_get_bool_client_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr rws_set_bool_client_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDNum>::SharedPtr rws_set_num_client_;
    rclcpp::Subscription<abb_robot_msgs::msg::SystemState>::SharedPtr rws_sys_states_sub_;
        
    // Interfaces with other bits of testing code
    rclcpp::Client<data_gathering_msgs::srv::StartGrinder>::SharedPtr start_grinder_client_;
    rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedPtr stop_grinder_client_;
    rclcpp::Publisher<stamped_std_msgs::msg::Float32Stamped>::SharedPtr acf_force_publisher_;
    rclcpp::Publisher<data_gathering_msgs::msg::GrindSettings>::SharedPtr settings_publisher_;
    rclcpp::Subscription<ferrobotics_acf::msg::ACFTelemStamped>::SharedPtr acf_telem_sub_;

    // Bool request with module and task preset
    abb_robot_msgs::srv::GetRAPIDBool::Request bool_symbol_req_default;

    rclcpp::TimerBase::SharedPtr timer_wait_home_;
    rclcpp::TimerBase::SharedPtr timer_wait_grind0_;
    rclcpp::TimerBase::SharedPtr timer_wait_grind_done_;
    rclcpp::TimerBase::SharedPtr timer_wait_script_done_;
    rclcpp::TimerBase::SharedPtr timer_wait_grinder_spinup_;

    // Variable names in the ABB RAPID code
    abb_robot_msgs::msg::RAPIDSymbolPath default_path_; // Default path template
    std::string symbol_home_flag;
    std::string symbol_grind0_flag;
    std::string symbol_grind_done_flag;
    std::string symbol_run_status_flag; 
    std::string symbol_nr_passes;
    std::string symbol_tcp_speed;
    
    bool test_aborted_;
    bool service_available;
    int bool_timer_period_;
    bool flip_back_timer_busy;
    bool finished_grind_timer_busy;
    double max_tcp_speed;
    int grinder_spinup_duration;
    double max_acf_extension;

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
        is_running_bool_req->path = createPath(symbol_run_status_flag);

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Checking run-status on the robot");
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

        // Ensure the ACF is in retracted position 
        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.header.stamp = this->get_clock()->now(); 
        acf_req.data = -5.;
        acf_force_publisher_->publish(acf_req);

        // Reset flags and save the request params. 
        success_ = true;
        message_.clear();
        test_service_           = service;
        test_request_header_    = request_header;
        test_request_           = request;   
        test_response_          = response;

        // Publish test settings for recording 
        auto grind_settings = data_gathering_msgs::msg::GrindSettings();
        grind_settings.rpm = test_request_->rpm;  
        grind_settings.force = test_request_->force;
        grind_settings.passes = test_request_->passes;
        grind_settings.tcp_speed = test_request_->tcp_speed;
        grind_settings.contact_time = test_request_->contact_time;
        settings_publisher_->publish(grind_settings);

        // Include a copy of the request settings in the response
        test_response_->grind_settings = grind_settings;

        RCLCPP_INFO(this->get_logger(), "Requesting PP to main...");

        auto pp_to_main_req = std::make_shared<abb_robot_msgs::srv::TriggerWithResultCode::Request>();
    
        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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
        req_passes->path = createPath(symbol_nr_passes);
        req_passes->value = test_request_->passes;

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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
        req_passes->path = createPath(symbol_tcp_speed);
        req_passes->value = test_request_->tcp_speed;


        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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
        
        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            return;
        }

        startRAPID();
    }

    void startRAPID(){
        RCLCPP_INFO(this->get_logger(), "\n\nStarting RAPID\n");

        auto request = std::make_shared<abb_robot_msgs::srv::TriggerWithResultCode::Request>();

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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


        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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
        Once this is the case, next_func is executed. After performing its task. next_func should use resetBoolValue on this bool.

        */

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            if (auto timer = weak_timer.lock()) {
                timer->cancel();
                return;
            }
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
        bool_req->path = createPath(value); 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this, value, weak_timer, next_func](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {

                if (this->test_aborted_) {
                    RCLCPP_WARN(this->get_logger(), "Aborting test");
                    if (auto timer = weak_timer.lock()) {
                        timer->cancel();
                        flip_back_timer_busy = false;
                        return;
                    }
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
            }
        );
    }

    void enableGrinder(){

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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
            [this](){
                
                if (test_aborted_) {
                    RCLCPP_WARN(this->get_logger(), "Aborting test");
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


        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            return;
        }

        auto reset_req = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
        reset_req->path = createPath(value);
        reset_req->value = false;

        rws_set_bool_client_->async_send_request(reset_req,
                [this, next_func](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){next_func(future);
        });
    }

    void handleResetHomeBoolResponse(rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future){

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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
        
        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            return;
        }

        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.header.stamp = this->get_clock()->now(); 
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

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            return;
        }

        // Check whether service call was successful.
        auto successful = checkABBResponse(future ,"resetting bool " + symbol_grind0_flag);
        if (!successful){return;}

        RCLCPP_INFO(this->get_logger(), "Grinding in progress\nWaiting until all passes are done...");        

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

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            return;
        }
        
        // TODO turn off acf and grinder    
        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.header.stamp = this->get_clock()->now(); 
        acf_req.data = -5.;
        acf_force_publisher_->publish(acf_req);

        auto grinder_off_req = std::make_shared<data_gathering_msgs::srv::StopGrinder::Request>(); 

        stop_grinder_client_->async_send_request(grinder_off_req, 
                [this](rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){this->handleGrinderOffResponse(future);
        });
    }

    void handleGrinderOffResponse(rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
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

        if (test_aborted_) {
            RCLCPP_WARN(this->get_logger(), "Aborting test");
            timer_wait_script_done_->cancel();
            finished_grind_timer_busy = false; 
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
        bool_req->path = createPath(symbol_run_status_flag); 

        // Send the request asynchronously
        rws_get_bool_client_->async_send_request(bool_req,
            [this](rclcpp::Client<abb_robot_msgs::srv::GetRAPIDBool>::SharedFuture future) {

                if (test_aborted_) {
                    RCLCPP_WARN(this->get_logger(), "Aborting test");
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
                test_response_->message += "Failed to perform task: " + task_message + " - result code: " + std::to_string(result->result_code);
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
            abortTest();

            RCLCPP_WARN(this->get_logger(), "E-STOP TRIGGERED. Sent requests to retract the ACF and turn off the grinder.");
            // cut off the callback chain process
            // finalize response
            test_response_->success = false;
            test_response_->message += "\nE-stop triggered.\n";
            finalizeResponse(test_service_, test_request_header_, test_response_);
        }
    }

    void acfTelemCallback(const ferrobotics_acf::msg::ACFTelemStamped& msg) {
        if (service_available){
            // No test is running 
            return;
        }
        if (msg.telemetry.position > max_acf_extension){
            // Reached max extension. Retract ACF 
            abortTest();
            RCLCPP_WARN(this->get_logger(), "Maximum extension reached. Aborting");
            test_response_->success = false;
            test_response_->message += "\nMaximum ACF extension reached. Test aborted.\n";
            finalizeResponse(test_service_, test_request_header_, test_response_);
        }
    }

    void abortTest(){
        test_aborted_ = true; 
        auto acf_req = stamped_std_msgs::msg::Float32Stamped();
        acf_req.header.stamp = this->get_clock()->now(); 
        acf_req.data = -20.;
        acf_force_publisher_->publish(acf_req);

        // request grinder off
        auto grinder_off_req = std::make_shared<data_gathering_msgs::srv::StopGrinder::Request>(); 
        stop_grinder_client_->async_send_request(grinder_off_req, 
            [this](rclcpp::Client<data_gathering_msgs::srv::StopGrinder>::SharedFuture future){this->handleGrinderOffResponse(future);
        });
    }

    abb_robot_msgs::msg::RAPIDSymbolPath createPath(const std::string& symbol) {
        /*
        Utility function to quickly create paths to ABB variables
        */
        abb_robot_msgs::msg::RAPIDSymbolPath custom_path = default_path_; // Copy the default path
        custom_path.symbol = symbol; // Set the desired symbol
        return custom_path;
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RWSMotionClient>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
