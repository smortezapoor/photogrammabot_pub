#include <Arduino.h>

#include <stdio.h>
#include <math.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>

#include <ax12.h>


#define PAN_ID  1
#define TILT_ID 2

#define PAN_POS_MIN   0
#define PAN_POS_MAX   1023

#define PAN_VEL_MIN   0
#define PAN_VEL_MAX   1023

#define PAN_EFF_MIN   0
#define PAN_EFF_MAX   1023

#define TILT_POS_MIN  256
#define TILT_POS_MAX  768

#define TILT_VEL_MIN  0
#define TILT_VEL_MAX  1023

#define TILT_EFF_MIN  0
#define TILT_EFF_MAX  1023

#define DEFAULT_VEL  0
#define DEFAULT_EFF  1.5

char pan_name[] = "pan_servo_horn_joint";
char tilt_name[] = "tilt_servo_horn_joint";

int DEBUG = 0;

ros::NodeHandle nh;


/*
 *
 * Conversion Functions
 *
 */

float pos_to_rad(int pos) {
  float step = 0.00511326929; // (300 degs / 1024 steps) to rad
  float rad = (float) (pos - 512) * step;
  return rad;
}

int rad_to_pos(float rad) {
  float step = 1.0/0.00511326929;
  // offset by 150 degs (512 steps)
  int pos = round((rad+2.61799387648)*step);
  return pos;
}

float vel_to_rad(int vel) {
  float step = 0.011623892805; // 0.111 rpm to rad/s

  int dir_mask = 0x400;
  int val_mask = 0x3FF;
  float dir = 0.0;

  if (vel & dir_mask == 0) {
    dir = -1.0;
  } else {
    dir = 1.0;
  }

  float rad = dir * ((float) (vel & val_mask)) * step;
  return rad;
}

int rad_to_vel(float rad) {
  float step = 1.0/0.011623892805;

  int dir_mask = 0x400;
  int val_mask = 0x3FF;

  int vel = abs(round(rad*step));
  vel = vel & val_mask;

//  if (rad >= 0) {
//    vel = vel | dir_mask;
//  }

  return vel;
}

float eff_to_Nm(int eff) {
  float step = 0.00146484375; // 1.5 Nm / 1024

  int dir_mask = 0x400;
  int val_mask = 0x3FF;
  float dir = 0.0;

  if (eff & dir_mask == 0) {
    dir = -1.0;
  } else {
    dir = 1.0;
  }

  float force = dir * ((float) (eff & val_mask)) * step;
  return force;
}

int Nm_to_eff(float force) {
  float step = 1.0/0.00146484375;

  int eff = abs(round(force*step));
  eff = max(min(eff, 1023), 0);
  return eff;
}


/*
 *
 * Position Setting Functions
 *
 */

void setRawPanTiltPos(int pan, int tilt) {
  pan = max(min(pan, PAN_POS_MAX), PAN_POS_MIN);
  tilt = max(min(tilt, TILT_POS_MAX), TILT_POS_MIN);

  int PTData[3][2] = {{PAN_ID, pan},
                      {TILT_ID, tilt},
                     };

  dxlSyncWritePosition(PTData, 2);

  if (DEBUG != 0) {
    char buf[256];
    snprintf(buf, 256, "Set Position - pan: %d  tilt: %d", pan, tilt);
    nh.loginfo(buf);
  }
}

void setPanTiltPos(float pan_rad, float tilt_rad) {
  int pan = rad_to_pos(pan_rad);
  int tilt = rad_to_pos(tilt_rad);

  setRawPanTiltPos(pan, tilt);
}

void setRawPanTilt(int pan_pos, int tilt_pos, int pan_vel, int tilt_vel, int pan_eff, int tilt_eff) {
  if (DEBUG != 0) {
    if (dxlPing(PAN_ID) == 1) {
      nh.loginfo("Pinged pan servo");
    }
    if (dxlPing(TILT_ID) == 1) {
      nh.loginfo("Pinged tilt servo");
    }
  }

  // clip values within max and min
  pan_pos = max(min(pan_pos, PAN_POS_MAX), PAN_POS_MIN);
  tilt_pos = max(min(tilt_pos, TILT_POS_MAX), TILT_POS_MIN);
  pan_vel = max(min(pan_vel, PAN_VEL_MAX), PAN_VEL_MIN);
  tilt_vel = max(min(tilt_vel, TILT_VEL_MAX), TILT_VEL_MIN);
  pan_eff = max(min(pan_eff, PAN_EFF_MAX), PAN_EFF_MIN);
  tilt_eff = max(min(tilt_eff, TILT_EFF_MAX), TILT_EFF_MIN);

  int pos_data[3][2] = {{PAN_ID, pan_pos},
                        {TILT_ID, tilt_pos},
                       };

  int vel_data[3][2] = {{PAN_ID, pan_vel},
                        {TILT_ID, tilt_vel},
                       };

  int eff_data[3][2] = {{PAN_ID, pan_eff},
                        {TILT_ID, tilt_eff},
                       };

  dxlSyncWrite(pos_data, 2, AX_GOAL_POSITION_L,2);
  dxlSyncWrite(vel_data, 2, AX_GOAL_SPEED_L,2);
  dxlSyncWrite(eff_data, 2, AX_TORQUE_LIMIT_L,2);

  if (DEBUG != 0) {
    char buf[256];
    snprintf(buf, 256, "Set Position - pan: %d  tilt: %d", pan_pos, tilt_pos);
    nh.loginfo(buf);

    snprintf(buf, 256, "Set Speed - pan: %d  tilt: %d", pan_vel, tilt_vel);
    nh.loginfo(buf);

    snprintf(buf, 256, "Set Torque - pan: %d  tilt: %d", pan_eff, tilt_eff);
    nh.loginfo(buf);
  }
}

void setPanTilt(float pan_pos_rad, float tilt_pos_rad, float pan_vel_rad, float tilt_vel_rad, float pan_eff_Nm, float tilt_eff_Nm) {
  int pan_pos = rad_to_pos(pan_pos_rad);
  int tilt_pos = rad_to_pos(tilt_pos_rad);
  int pan_vel = rad_to_vel(pan_vel_rad);
  int tilt_vel = rad_to_vel(tilt_vel_rad);
  int pan_eff = Nm_to_eff(pan_eff_Nm);
  int tilt_eff = Nm_to_eff(tilt_eff_Nm);

  setRawPanTilt(pan_pos, tilt_pos, pan_vel, tilt_vel, pan_eff, tilt_eff);
}


/*
 *
 * Call Backs
 *
 */

void cmdCallback(const sensor_msgs::JointState& cmd_msg) {
//  nh.loginfo("PTU command callback");

  int pan_idx = -1;
  int tilt_idx = -1;

  float pan_pos = 0;
  float tilt_pos = 0;
  float pan_vel = DEFAULT_VEL;
  float tilt_vel = DEFAULT_VEL;
  float pan_eff = DEFAULT_EFF;
  float tilt_eff = DEFAULT_EFF;

  // check JointState name fields
  for (int i = 0; i < cmd_msg.name_length; i++) {
    if (strncmp(cmd_msg.name[i], pan_name, strlen(pan_name)) == 0) {
      pan_idx = i;
    } else if (strncmp(cmd_msg.name[i], tilt_name, strlen(tilt_name)) == 0) {
      tilt_idx = i;
    }
  }

  if ( (pan_idx != -1) && (tilt_idx != -1) ) {

    if ( cmd_msg.position_length >= 2 ) {
      pan_pos = cmd_msg.position[pan_idx];
      tilt_pos = cmd_msg.position[tilt_idx];
    } else {
      pan_pos = 0;
      tilt_pos = 0;
    }

    if ( cmd_msg.velocity_length >= 2 ) {
      pan_vel = cmd_msg.velocity[pan_idx];
      tilt_vel = cmd_msg.velocity[tilt_idx];
    } else {
      pan_vel = DEFAULT_VEL;
      tilt_vel = DEFAULT_VEL;
    }

    if ( cmd_msg.effort_length >= 2 ) {
      pan_eff = cmd_msg.effort[pan_idx];
      tilt_eff = cmd_msg.effort[tilt_idx];
    } else {
      pan_eff = DEFAULT_EFF;
      tilt_eff = DEFAULT_EFF;
    }

  }

  if (DEBUG != 0) {
    char buf[128];

    for (int i = 0; i < cmd_msg.name_length; i++) {
      snprintf(buf, 128, "name[%d]: %s", i, cmd_msg.name[i]);
      nh.loginfo(buf);
    }

    char pan_pos_str[16];
    char tilt_pos_str[16];
    char pan_vel_str[16];
    char tilt_vel_str[16];
    char pan_eff_str[16];
    char tilt_eff_str[16];
    dtostrf(pan_pos, 4, 3, pan_pos_str);
    dtostrf(tilt_pos, 4, 3, tilt_pos_str);
    dtostrf(pan_vel, 4, 3, pan_vel_str);
    dtostrf(tilt_vel, 4, 3, tilt_vel_str);
    dtostrf(pan_eff, 4, 3, pan_eff_str);
    dtostrf(tilt_eff, 4, 3, tilt_eff_str);


    snprintf(buf, 128, "[cmdCallback]:\n  p0: %s  p1: %s\n  v0: %s  v1: %s\n  e0: %s  e1: %s\n", pan_pos_str, tilt_pos_str, pan_vel_str, tilt_vel_str, pan_eff_str, tilt_eff_str);
    nh.loginfo(buf);
  }

  setPanTilt(pan_pos, tilt_pos, pan_vel, tilt_vel, pan_eff, tilt_eff);
  
}



ros::Subscriber<sensor_msgs::JointState> sub_cmd("joint_states", &cmdCallback);

sensor_msgs::JointState js_msg;
ros::Publisher pub_state("state", &js_msg);


void publishCurrentState() {
  sensor_msgs::JointState js_msg;

  int pan_pos = dxlGetPosition(PAN_ID);
  int tilt_pos = dxlGetPosition(TILT_ID);
  int pan_speed = dxlGetSpeed(PAN_ID);
  int tilt_speed = dxlGetSpeed(TILT_ID);
  int pan_torque = dxlGetTorque(PAN_ID);
  int tilt_torque = dxlGetTorque(TILT_ID);

  char* name[] = {pan_name, tilt_name};

  float pos[] = {pos_to_rad(pan_pos), pos_to_rad(tilt_pos)};

  float vel[] = {vel_to_rad(pan_speed), vel_to_rad(tilt_speed)};

  float eff[] = {eff_to_Nm(pan_torque), eff_to_Nm(tilt_torque)};

  js_msg.name = name;
  js_msg.position = pos;
  js_msg.velocity = vel;
  js_msg.effort = eff;

  js_msg.name_length = 2;
  js_msg.position_length = 2;
  js_msg.velocity_length = 2;
  js_msg.effort_length = 2;

  pub_state.publish(&js_msg);
}


/*
 *
 * Init / Setup()
 *
 */

void setup() {
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);

  // connect to dynamixel servos
  dxlInit(1000000);

  // configure serial connection to 1 Mbps
  ArduinoHardware* nh_hw = nh.getHardware();
  nh_hw->setBaud(1000000);
  
  nh.initNode();
 
  nh.advertise(pub_state);
  nh.subscribe(sub_cmd);
  
  // wait for node to connect
  while (!nh.connected()) {
    nh.spinOnce();
  }
  
  if (!nh.getParam("~debug", &DEBUG, 1)) {
    DEBUG = 0;
  }
}


/*
 *
 * Main
 *
 */

void loop() {
  // periodically publish state
  // spinOnce
  publishCurrentState();
  nh.spinOnce();
  delay(100);
}
