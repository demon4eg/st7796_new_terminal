#include "ros_manager.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "ui_manager.h"
#include "display_config.h"

// micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <manipulator_6dof_interfaces/msg/telemetry.h>
#include <manipulator_6dof_interfaces/msg/terminal_cmd.h>
#include <rmw_microros/rmw_microros.h>

// WiFi headers
#include <uros_network_interfaces.h>

static const char *TAG = "ROS_MGR";

typedef struct {
    int type;
    int id;
    float value;
} ros_cmd_t;

static QueueHandle_t ros_cmd_queue;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGW(TAG, "Failed status on line %d: %d. Continuing.",__LINE__,(int)temp_rc);}}

static rcl_publisher_t cmd_pub;
static rcl_publisher_t hb_pub;
static rcl_subscription_t telemetry_sub;
static rcl_subscription_t debug_sub;

static manipulator_6dof_interfaces__msg__TerminalCmd cmd_msg;
static manipulator_6dof_interfaces__msg__Telemetry telemetry_msg;
static std_msgs__msg__Int32 hb_msg;
static std_msgs__msg__String debug_msg;

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    if (timer != NULL) {
        hb_msg.data = esp_timer_get_time() / 1000;
        RCSOFTCHECK(rcl_publish(&hb_pub, &hb_msg, NULL));
    }
}

void debug_callback(const void * msgin)
{
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

    if (msg && msg->data.data) {
        ui_set_terminal_text(msg->data.data);
        ui_set_terminal_text("\n");
    }
}

static void telemetry_callback(const void * msgin)
{
    const manipulator_6dof_interfaces__msg__Telemetry * msg =
        (const manipulator_6dof_interfaces__msg__Telemetry *)msgin;

    if (!msg) return;

    float display_data[13];

    for(int i = 0; i < 6; i++) {
        display_data[i] = msg->xyzrpy[i];
    }

    for(int i = 0; i < 6; i++) {
        display_data[i + 6] = msg->joints[i];
    }

    display_data[12] = msg->gripper;

    ui_update_telemetry(
        display_data, 13,
        msg->state,
        msg->tool_id,
        msg->work_offset_id,
        msg->cartesian_mode,
        msg->speed_override
    );
}

static void ros_ui_request_handler(int type, int id, float value)
{
    if (!ros_cmd_queue) return;

    ros_cmd_t cmd = {
        .type = type,
        .id = id,
        .value = value
    };

    xQueueSend(ros_cmd_queue, &cmd, 0);
    ESP_LOGI(TAG, "CMD queued: type=%d id=%d val=%.2f", type, id, value);
}

void ros_send_full_state(void)
{
    cmd_msg.tool_id = ui_get_tool_id();
    cmd_msg.work_offset_id = ui_get_work_offset();  // убрали _id
    cmd_msg.state = ui_get_robot_state();
    cmd_msg.auto_state = 0;  //暂时 не используем
    cmd_msg.cartesian_mode = ui_get_cartesian_mode();
    cmd_msg.speed_override = ui_get_speed_override();
    cmd_msg.estop = false;  //暂时 не используем
    cmd_msg.action_trigger = 0;

    RCSOFTCHECK(rcl_publish(&cmd_pub, &cmd_msg, NULL));

    ESP_LOGI(TAG, "State sent: tool=%d work=%d state=%d mode=%d speed=%.2f",
        cmd_msg.tool_id,
        cmd_msg.work_offset_id,
        cmd_msg.state,
        cmd_msg.cartesian_mode,
        cmd_msg.speed_override);
}

static void micro_ros_task(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    ui_set_command_callback(ros_ui_request_handler);

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    ui_set_command_callback(ros_ui_request_handler);

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_gui_node", "", &support));

    RCCHECK(rclc_publisher_init_default(&hb_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "terminal/heartbeat"));
    RCCHECK(rclc_publisher_init_default(&cmd_pub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, TerminalCmd), "terminal/command"));

    RCCHECK(rclc_subscription_init_default(&debug_sub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "robot/debug"));
    RCCHECK(rclc_subscription_init_default(&telemetry_sub, &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(manipulator_6dof_interfaces, msg, Telemetry), "robot/telemetry"));

    ros_cmd_queue = xQueueCreate(10, sizeof(ros_cmd_t));

    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback, true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &debug_sub, &debug_msg, &debug_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &telemetry_sub, &telemetry_msg, &telemetry_callback, ON_NEW_DATA));

    std_msgs__msg__String__init(&debug_msg);
    debug_msg.data.data = (char*)malloc(256);
    debug_msg.data.capacity = 256;
    debug_msg.data.size = 0;

    manipulator_6dof_interfaces__msg__TerminalCmd__init(&cmd_msg);
    manipulator_6dof_interfaces__msg__Telemetry__init(&telemetry_msg);
    std_msgs__msg__Int32__init(&hb_msg);

    ESP_LOGI(TAG, "micro-ROS WiFi node started");

    vTaskDelay(pdMS_TO_TICKS(1000));
    ros_send_full_state();

    while(1) {
        ros_cmd_t cmd;
        while (xQueueReceive(ros_cmd_queue, &cmd, 0) == pdTRUE) {
            switch(cmd.type) {
                case 1: cmd_msg.work_offset_id = (uint8_t)cmd.id; break;
                case 2: cmd_msg.tool_id = (uint8_t)cmd.id; break;
                case 3: cmd_msg.state = (uint8_t)cmd.id; break;
                case 4: cmd_msg.auto_state = (uint8_t)cmd.id; break;
                case 5: cmd_msg.cartesian_mode = (cmd.id == 1); break;
                case 6: cmd_msg.speed_override = cmd.value; break;
                case 7: cmd_msg.estop = (cmd.id == 1); break;
                case 8: cmd_msg.action_trigger = (uint8_t)cmd.id; break;
            }

            RCSOFTCHECK(rcl_publish(&cmd_pub, &cmd_msg, NULL));

            ESP_LOGI(TAG, "CMD sent: tool=%d work=%d state=%d auto_state=%d mode=%d estop=%d speed=%.2f trigger=%d",
                    cmd_msg.tool_id,
                    cmd_msg.work_offset_id,
                    cmd_msg.state,
                    cmd_msg.auto_state,
                    cmd_msg.cartesian_mode,
                    cmd_msg.estop,
                    cmd_msg.speed_override,
                    cmd_msg.action_trigger);
            
            if(cmd.type == 8) cmd_msg.action_trigger = 0;
        }

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(debug_msg.data.data);
    vTaskDelete(NULL);
}

void ros_manager_init(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "WiFi power saving disabled");

    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}