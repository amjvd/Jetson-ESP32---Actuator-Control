#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include "driver/twai.h"

// --- WIRING PINS ---
#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_26

// --- MOTOR IDs (Update these as needed!) ---
#define MOTOR_L_FRONT 1
#define MOTOR_L_BACK  2
#define MOTOR_R_FRONT 3
#define MOTOR_R_BACK  4

// --- ROBOT KINEMATICS ---
#define WHEEL_RADIUS 0.05f  // meters
#define TRACK_WIDTH  0.4f   // meters (distance between left and right wheels)

// --- AK45 PACKING LIMITS (For sending commands) ---
#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -30.0f
#define V_MAX  30.0f
#define T_MIN -18.0f
#define T_MAX  18.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

// --- MICRO-ROS GLOBALS ---
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_msg;

// Pre-allocated arrays for JointState
rosidl_runtime_c__String joint_names[4];
double joint_pos[4];
double joint_vel[4];
double joint_eff[4];

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Global targets
float target_linear_v = 0.0;
float target_angular_w = 0.0;

unsigned long previousMillis = 0;
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
 while(1){
   delay(100);
 }
}

// Float to Unsigned Integer Conversion (For Packing Commands)
int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Unsigned Integer to Float Conversion (For Parsing Encoders)
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
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

// Initialize specific motor
void init_motor(uint8_t id) {
  uint8_t enter_mit[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  send_can_message(id, enter_mit, 8);
  delay(10);
  uint8_t set_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  send_can_message(id, set_zero, 8);
  delay(10);
}

// Pack and Send MIT Command
void pack_cmd(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff) {
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
  send_can_message(id, data, 8);
}

// ROS 2 SUBSCRIPTION CALLBACK
void cmd_vel_callback(const void * msgin) {
 const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
 target_linear_v = msg->linear.x;
 target_angular_w = msg->angular.z;
 last_cmd_time = millis();
}

void setup() {
  set_microros_transports();
  
  // CAN INIT
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      twai_start();
  }
  
  delay(1000);
  init_motor(MOTOR_L_FRONT);
  init_motor(MOTOR_L_BACK);
  init_motor(MOTOR_R_FRONT);
  init_motor(MOTOR_R_BACK);
  
  // MICRO-ROS INIT
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_ak45_skidsteer", "", &support));
  
  // Twist Subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    
  // JointState Publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));
    
  // Configure JointState arrays
  joint_msg.name.capacity = 4;
  joint_msg.name.size = 4;
  joint_msg.name.data = joint_names;
  rosidl_runtime_c__String__assign(&joint_msg.name.data[0], "front_left");
  rosidl_runtime_c__String__assign(&joint_msg.name.data[1], "back_left");
  rosidl_runtime_c__String__assign(&joint_msg.name.data[2], "front_right");
  rosidl_runtime_c__String__assign(&joint_msg.name.data[3], "back_right");

  joint_msg.position.capacity = 4;
  joint_msg.position.size = 4;
  joint_msg.position.data = joint_pos;

  joint_msg.velocity.capacity = 4;
  joint_msg.velocity.size = 4;
  joint_msg.velocity.data = joint_vel;

  joint_msg.effort.capacity = 4;
  joint_msg.effort.size = 4;
  joint_msg.effort.data = joint_eff;
    
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void unpack_reply(twai_message_t *rx_msg) {
  // Enforce exactly 8 byte DLC per the AK45 manual
  if (rx_msg->data_length_code != 8) return;

  int id = rx_msg->data[0];
  int p_int = (rx_msg->data[1] << 8) | rx_msg->data[2];
  int v_int = (rx_msg->data[3] << 4) | (rx_msg->data[4] >> 4);
  int i_int = ((rx_msg->data[4] & 0xF) << 8) | rx_msg->data[5];
  int t_int = rx_msg->data[6];
  
  // Use bounds from the provided manual snippet
  float pos = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float vel = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float torque = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  float temp = (float)t_int - 40.0f; // Temperature -40~215
  
  // Route data to the correct joint index
  int idx = -1;
  if (id == MOTOR_L_FRONT) idx = 0;
  else if (id == MOTOR_L_BACK) idx = 1;
  else if (id == MOTOR_R_FRONT) idx = 2;
  else if (id == MOTOR_R_BACK) idx = 3;
  
  if (idx != -1) {
      joint_pos[idx] = pos;
      joint_vel[idx] = vel;
      joint_eff[idx] = torque;
  }
}

void read_can_and_publish() {
  twai_message_t rx_msg;
  bool new_data_received = false;
  
  // Read all available CAN messages from buffer
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
      unpack_reply(&rx_msg);
      new_data_received = true;
  }
  
  // If we got new encoder data this cycle, publish it to Jetson
  if (new_data_received) {
      // (Optional) Populate joint_msg.header.stamp if you have synced time
      rcl_publish(&publisher, &joint_msg, NULL);
  }
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  
  unsigned long currentMillis = millis();
  
  // Safety Stop
  if (currentMillis - last_cmd_time > CMD_TIMEOUT_MS) {
      target_linear_v = 0.0;
      target_angular_w = 0.0;
  }
  
  // 100Hz Motor Control Loop
  if (currentMillis - previousMillis >= 10) {
      previousMillis = currentMillis;
      
      // --- DIFFERENTIAL DRIVE / SKID STEER KINEMATICS ---
      // v_left  = linear_v - (angular_w * TRACK_WIDTH / 2)
      // v_right = linear_v + (angular_w * TRACK_WIDTH / 2)
      float v_left  = target_linear_v - (target_angular_w * TRACK_WIDTH / 2.0f);
      float v_right = target_linear_v + (target_angular_w * TRACK_WIDTH / 2.0f);
      
      // Convert linear m/s to angular rad/s for the motors
      float w_left  = v_left / WHEEL_RADIUS;
      float w_right = v_right / WHEEL_RADIUS;
      
      // Send commands to all 4 motors
      // Note: Right side wheels are typically mirrored on the chassis,
      // so you may need to invert the command (send -w_right) depending on hardware mounting!
      pack_cmd(MOTOR_L_FRONT, 0.0, w_left,  0.0, 2.0, 0.0);
      pack_cmd(MOTOR_L_BACK,  0.0, w_left,  0.0, 2.0, 0.0);
      pack_cmd(MOTOR_R_FRONT, 0.0, w_right, 0.0, 2.0, 0.0); 
      pack_cmd(MOTOR_R_BACK,  0.0, w_right, 0.0, 2.0, 0.0);
      
      // Read CAN replies and publish JointState
      read_can_and_publish();
  }
}
