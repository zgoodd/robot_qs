
// #include <DynamixelWorkbench.h>
//#include "../include/xleg_controller/dynamixel_workbench.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  const char* port_name = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int dxl_id = 9;

  // if (argc < 4)
  // {
  //   printf("Please set '-port_name', '-baud_rate', '-dynamixel id' arguments for connected Dynamixels\n");
  //   return 0;
  // }
  // else
  // {
  //   port_name = argv[1];
  //   baud_rate = atoi(argv[2]);
  //   dxl_id = atoi(argv[3]);
  // }

  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  uint16_t model_number = 0;

  result = dxl_wb.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");

    return 0;
  }
  else
    printf("Succeed to init(%d)\n", baud_rate);

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping\n");
  }
  else
  {
    printf("Succeed to ping\n");
    printf("id : %d, model_number : %d\n", dxl_id, model_number);
  }

  int32_t velocity = 10/6/0.229;   // du/s -> rpm -> data
  int32_t acceleration = 0;
  result = dxl_wb.jointMode(dxl_id, (int32_t) velocity, acceleration, &log);    //结束后自动torqueOn

  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to change joint mode\n");
  }
  else
  {
    printf("Succeed to change joint mode\n");
    printf("Dynamixel is moving...\n");

    result=dxl_wb.itemWrite( dxl_id, "Profile_Velocity", (int32_t) (10/6/0.229), &log);
    if (result == false)
    {
        ROS_ERROR("Failed to set Profile_Velocity of Motor %d\n" ,  dxl_id);
        return 0;
    }

    dxl_wb.torqueOff(dxl_id, &log);
    result=dxl_wb.itemWrite( dxl_id, "Min_Position_Limit", (int32_t) (140/0.0879), &log);
    result=dxl_wb.itemWrite( dxl_id, "Max_Position_Limit", (int32_t) (200/0.0879), &log);
    if (result == false)
        ROS_ERROR("Failed to set Position_Limit of Motor %d\n" ,  dxl_id);

    dxl_wb.torqueOn(dxl_id, &log);

    for (int count = 0; count < 2; count++)
    {
      dxl_wb.goalPosition(dxl_id, (int32_t) (150/0.088));

      int32_t get_data = 0;
      result = dxl_wb.itemRead(dxl_id, "Present_Temperature", &get_data, &log);
      int present_temperature = get_data;
      if (result == false)
          ROS_ERROR("Failed to get Present_Temperature of Motor %d.",  dxl_id);
      else
          ROS_INFO("Present_Temperature = %d", present_temperature);

      result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data, &log);
      if (result == false)
          ROS_ERROR("Failed to get Present_Position of Motor %d.",  dxl_id);
      else
          ROS_INFO("Present_Position = %d", get_data);

      result = dxl_wb.itemRead(dxl_id, "Present_Current", &get_data, &log);    //unit 3.36mA
      if (result == false)
          ROS_ERROR("Failed to get Present_Current of Motor %d.",  dxl_id);
      else
          ROS_INFO("Present_Current = %d", get_data);

      sleep(3);
      dxl_wb.goalPosition(dxl_id, (int32_t) (190/0.088));
      sleep(3);
    }
  }

  return 0;
}
