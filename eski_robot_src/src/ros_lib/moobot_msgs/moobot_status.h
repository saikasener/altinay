#ifndef _ROS_moobot_msgs_moobot_status_h
#define _ROS_moobot_msgs_moobot_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moobot_msgs
{

  class moobot_status : public ros::Msg
  {
    public:
      typedef int32_t _agv_mode_type;
      _agv_mode_type agv_mode;
      typedef bool _agv_stopped_type;
      _agv_stopped_type agv_stopped;
      typedef bool _reset_pressed_type;
      _reset_pressed_type reset_pressed;
      typedef bool _start_pressed_type;
      _start_pressed_type start_pressed;

    moobot_status():
      agv_mode(0),
      agv_stopped(0),
      reset_pressed(0),
      start_pressed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_agv_mode;
      u_agv_mode.real = this->agv_mode;
      *(outbuffer + offset + 0) = (u_agv_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_agv_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_agv_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_agv_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->agv_mode);
      union {
        bool real;
        uint8_t base;
      } u_agv_stopped;
      u_agv_stopped.real = this->agv_stopped;
      *(outbuffer + offset + 0) = (u_agv_stopped.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->agv_stopped);
      union {
        bool real;
        uint8_t base;
      } u_reset_pressed;
      u_reset_pressed.real = this->reset_pressed;
      *(outbuffer + offset + 0) = (u_reset_pressed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reset_pressed);
      union {
        bool real;
        uint8_t base;
      } u_start_pressed;
      u_start_pressed.real = this->start_pressed;
      *(outbuffer + offset + 0) = (u_start_pressed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->start_pressed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_agv_mode;
      u_agv_mode.base = 0;
      u_agv_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_agv_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_agv_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_agv_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->agv_mode = u_agv_mode.real;
      offset += sizeof(this->agv_mode);
      union {
        bool real;
        uint8_t base;
      } u_agv_stopped;
      u_agv_stopped.base = 0;
      u_agv_stopped.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->agv_stopped = u_agv_stopped.real;
      offset += sizeof(this->agv_stopped);
      union {
        bool real;
        uint8_t base;
      } u_reset_pressed;
      u_reset_pressed.base = 0;
      u_reset_pressed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reset_pressed = u_reset_pressed.real;
      offset += sizeof(this->reset_pressed);
      union {
        bool real;
        uint8_t base;
      } u_start_pressed;
      u_start_pressed.base = 0;
      u_start_pressed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->start_pressed = u_start_pressed.real;
      offset += sizeof(this->start_pressed);
     return offset;
    }

    const char * getType(){ return "moobot_msgs/moobot_status"; };
    const char * getMD5(){ return "f814f206f0b579e6ceacfbc4fd82c02b"; };

  };

}
#endif