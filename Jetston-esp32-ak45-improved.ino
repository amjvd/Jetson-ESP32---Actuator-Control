#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include "driver/twai.h"

//WIRING PINS
#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_26
#define MOTOR_ID 1

// AK45-36 STRICT LIMITS
#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -6.0f
#define V_MAX  6.0f
#define T_MIN -34.0f
#define T_MAX  34.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

//ROBOT SPECIFICS
#define WHEEL_RADIUS 0.05f // Example: 5cm wheel radius, adjust to your robot

//MICRO-ROS GLOBALS
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Global target velocity updated by ROS 2
float target_velocity = 0.0;
unsigned long previousMillis = 0;

// Safety timeout
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Error loop to catch micro-ROS setup failures
void error_loop(){
 while(1){
   // Consider blinking an LED here
   delay(100);
 }
}

// Float to Unsigned Integer Conversion for protocol packing
int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Send standard CAN frame
void send_can_message(uint32_t id, uint8_t* data, uint8_t len) {
  twai_message_t message;
  message.identifier = id;
  message.extd = 0;
  message.rtr = 0;  
  message.data_length_code = len;
  for (int i = 0; i < len; i++) {
      message.data[i] = data[i];
  }
  twai_transmit(&message, pdMS_TO_TICKS(10));
}

//SPECIAL HEX COMMANDS
void enter_mit_mode() {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  send_can_message(MOTOR_ID, data, 8);
}
void set_zero_position() {
  uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  send_can_message(MOTOR_ID, data, 8);
}

//PACK AND SEND MIT COMMAND (as per manual)
void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff) {
  p_des = constrain(p_des, P_MIN, P_MAX);
  v_des = constrain(v_des, V_MIN, V_MAX);
  kp = constrain(kp, KP_MIN, KP_MAX);
  kd = constrain(kd, KD_MIN, KD_MAX);
  t_ff = constrain(t_ff, T_MIN, T_MAX);
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  uint8_t data[8];
  data[0] = p_int >> 8;                          
  data[1] = p_int & 0xFF;                        
  data[2] = v_int >> 4;                          
  data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  data[4] = kp_int & 0xFF;                        
  data[5] = kd_int >> 4;                          
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  data[7] = t_int & 0xFF;                        
  send_can_message(MOTOR_ID, data, 8);
}

//ROS 2 SUBSCRIPTION CALLBACK
//fires every time the Jetson sends a /cmd_vel message
void cmd_vel_callback(const void * msgin) {
 const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
 
 // The teleop publishes linear velocity in m/s. 
 // The AK45 motor expects angular velocity in rad/s.
 // v_rad_sec = v_linear_m_s / wheel_radius
 target_velocity = twist_msg->linear.x / WHEEL_RADIUS; 
 
 // Reset safety timeout
 last_cmd_time = millis();
}

void setup() {
  // Set up the Serial bridge to the Jetson
  set_microros_transports();
  
  // Initialize CAN Driver
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      twai_start();
  }
  
  // Wake up the motor
  delay(1000);
  enter_mit_mode();
  delay(100);
  set_zero_position();
  delay(100);
  
  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_ak45_node", "", &support));
  
  // Create subscriber to the "/cmd_vel" topic
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
    
  // Create executor to handle the incoming messages
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
  // Briefly check for new ROS 2 messages from the Jetson
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  
  unsigned long currentMillis = millis();
  
  // Safety timeout check: Stop motor if Jetson connection is lost or teleop stops
  if (currentMillis - last_cmd_time > CMD_TIMEOUT_MS) {
      target_velocity = 0.0;
  }
  
  // Keep the CAN at 100Hz
  if (currentMillis - previousMillis >= 10) {
      previousMillis = currentMillis;
      // Pure Velocity Control:
      // Position = 0.0 (Ignored), Velocity = target_velocity, Kp = 0.0 (Stiffness off), Kd = 2.0 (Damping active), Torque = 0.0
      pack_cmd(0.0, target_velocity, 0.0, 2.0, 0.0);
  }
}
