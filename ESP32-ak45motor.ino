#include "driver/twai.h"

//PINS
#define CAN_TX_PIN GPIO_NUM_27
#define CAN_RX_PIN GPIO_NUM_26

//MOTOR ID
#define MOTOR_ID 1

//AK45-36 LIMITS 
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

// Float to Unsigned Integer Conversion for protocol packing
int float_to_uint(float x, float x_min, float x_max, int bits) {
   float span = x_max - x_min;
   float offset = x_min;
   if(x < x_min) x = x_min;
   else if(x > x_max) x = x_max;
   return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}
// Send CAN
void send_can_message(uint32_t id, uint8_t* data, uint8_t len) {
   twai_message_t message;
   message.identifier = id;
   message.extd = 0; // Standard frame for MIT mode
   message.rtr = 0;  
   message.data_length_code = len;
   for (int i = 0; i < len; i++) {
       message.data[i] = data[i];
   }
   twai_transmit(&message, pdMS_TO_TICKS(10));
}

//SPECIAL HEX COMMANDS
void enter_mit_mode() {
   // Array to enter motor control mode
   uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
   send_can_message(MOTOR_ID, data, 8);
}
void set_zero_position() {
   // Array to set current motor position to point 0
   uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
   send_can_message(MOTOR_ID, data, 8);
}
//PACK AND SEND MIT COMMAND (According to Manual)
void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff) {
   // 1. Constrain to limits
   p_des = constrain(p_des, P_MIN, P_MAX);
   v_des = constrain(v_des, V_MIN, V_MAX);
   kp = constrain(kp, KP_MIN, KP_MAX);
   kd = constrain(kd, KD_MIN, KD_MAX);
   t_ff = constrain(t_ff, T_MIN, T_MAX);
   // 2. Convert to integers with specific bit depths
   int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
   int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
   int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
   int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
   int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
   // 3. Pack bits into the 8-byte payload
   uint8_t data[8];
   data[0] = p_int >> 8;                           // Position high 8 bits
   data[1] = p_int & 0xFF;                         // Position low 8 bits
   data[2] = v_int >> 4;                           // Speed high 8 bits
   data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed low 4 bits, KP high 4 bits
   data[4] = kp_int & 0xFF;                        // Kp value low 8 bits
   data[5] = kd_int >> 4;                          // KD value high 8 bits
   data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KD value low 4 bits, current value high 4 bits
   data[7] = t_int & 0xFF;                         // current value low 8 bits
   send_can_message(MOTOR_ID, data, 8);
}
void setup() {
   Serial.begin(115200);
   delay(2000);
   Serial.println("Initializing CAN Driver...");
   // The manual recommends 1Mbps CAN bit rate
   twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
   if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
       Serial.println("Driver installed.");
   } else {
       Serial.println("Failed to install driver.");
       return;
   }
   if (twai_driver_start() == ESP_OK) {
       Serial.println("Driver started.");
   }
   delay(1000);
   Serial.println("Sending Special Hex Code: Enter MIT Mode");
   enter_mit_mode();
   delay(100);
   Serial.println("Sending Special Hex Code: Set Zero");
   set_zero_position();
   delay(100);
   Serial.println("Motor is Live. Running Soft Hold Loop.");
}

unsigned long previousMillis = 0;
const long interval = 2000; // Time between moves (2000 milliseconds = 2 seconds)
int position_state = 0; // Keeps track of where we are
float target_position = 0.0; 

void loop() {
   unsigned long currentMillis = millis();

   // Every 2 seconds, flip the  position
   if (currentMillis - previousMillis >= interval) {
       previousMillis = currentMillis; // Reset the timer
       
       if (position_state == 0) {
           target_position = 3.14159; // Move to 180 
           position_state = 1;
           Serial.println("Sweeping to 180 degrees (3.14 rad)");
       } else {
           target_position = 0.0; // Move back to 0
           position_state = 0;
           Serial.println("Sweeping to 0 degrees (0.0 rad)");
       }
   }

   // Continuously spam the motor with the current target position at 100Hz
   // Target Position, 0.0 speed limit, 5.0 stiffness, 0.5 damping, 0.0 torque
   pack_cmd(target_position, 0.0, 5.0, 0.5, 0.0);
   
   delay(10); // Maintain the 100Hz 
}
