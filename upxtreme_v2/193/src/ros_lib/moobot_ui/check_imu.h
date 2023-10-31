#ifndef _ROS_moobot_ui_check_imu_h
#define _ROS_moobot_ui_check_imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moobot_ui
{

  class check_imu : public ros::Msg
  {
    public:
      typedef float _angular_vel_z_type;
      _angular_vel_z_type angular_vel_z;
      typedef float _linear_acc_x_type;
      _linear_acc_x_type linear_acc_x;

    check_imu():
      angular_vel_z(0),
      linear_acc_x(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angular_vel_z;
      u_angular_vel_z.real = this->angular_vel_z;
      *(outbuffer + offset + 0) = (u_angular_vel_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_vel_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_vel_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_vel_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_vel_z);
      union {
        float real;
        uint32_t base;
      } u_linear_acc_x;
      u_linear_acc_x.real = this->linear_acc_x;
      *(outbuffer + offset + 0) = (u_linear_acc_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_acc_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_acc_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_acc_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_acc_x);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angular_vel_z;
      u_angular_vel_z.base = 0;
      u_angular_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_vel_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_vel_z = u_angular_vel_z.real;
      offset += sizeof(this->angular_vel_z);
      union {
        float real;
        uint32_t base;
      } u_linear_acc_x;
      u_linear_acc_x.base = 0;
      u_linear_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_acc_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_acc_x = u_linear_acc_x.real;
      offset += sizeof(this->linear_acc_x);
     return offset;
    }

    const char * getType(){ return "moobot_ui/check_imu"; };
    const char * getMD5(){ return "9b1b998330ad15a2d1903c80d31c89e4"; };

  };

}
#endif