#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>

int main(int argc, char **argv) {
  wb_robot_init();
  WbDeviceTag emitter, button;
  int timestep = (int)wb_robot_get_basic_time_step();
  emitter = wb_robot_get_device("emitter");
  button = wb_robot_get_device("button");
  wb_touch_sensor_enable(button, timestep);
  bool pushed = false;
  
   while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }

  while (wb_robot_step(timestep) != -1) {
     
     if (!pushed && wb_touch_sensor_get_value(button) > 0){
       printf("I've been pushed! \n");
       pushed = true;
       const char *message = "Go!";
       wb_emitter_send(emitter, message, strlen(message) + 1);
     }
     
  }

  wb_robot_cleanup();

  return 0;
}

