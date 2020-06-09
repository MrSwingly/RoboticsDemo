#include <stdio.h>
#include <string.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/motor.h>
 
#define MAX_SPEED 10

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timestep);
   
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0);
  }
 
 
 while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }
 
 
  double speeds[4] = {0,0,0,0};
  
  while (wb_robot_step(timestep) != -1) {
  
     if (wb_receiver_get_queue_length(receiver) > 0)
     {
       //const char *buffer = wb_receiver_get_data(receiver);
       //process(buffer);
       wb_receiver_next_packet(receiver);
       for (int i = 0; i < 4; i++) {
       speeds[i] = MAX_SPEED;
     }
     } 
     
    for (int i = 0; i < 4; i++) {
       wb_motor_set_velocity(wheels[i], speeds[i]);
     }  
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}