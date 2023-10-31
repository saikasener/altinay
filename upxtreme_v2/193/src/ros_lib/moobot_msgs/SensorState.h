#ifndef _ROS_moobot_msgs_SensorState_h
#define _ROS_moobot_msgs_SensorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace moobot_msgs
{

  class SensorState : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _led_type;
      _led_type led;
      typedef int64_t _left_encoder_type;
      _left_encoder_type left_encoder;
      typedef int64_t _right_encoder_type;
      _right_encoder_type right_encoder;
      typedef float _battery_type;
      _battery_type battery;
      enum { ERROR_LEFT_MOTOR =  1 };
      enum { ERROR_RIGHT_MOTOR =  2 };
      enum { TORQUE_ON =  1 };
      enum { TORQUE_OFF =  2 };

    SensorState():
      header(),
      led(0),
      left_encoder(0),
      right_encoder(0),
      battery(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->led >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led);
      union {
        int64_t real;
        uint64_t base;
      } u_left_encoder;
      u_left_encoder.real = this->left_encoder;
      *(outbuffer + offset + 0) = (u_left_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_encoder);
      union {
        int64_t real;
        uint64_t base;
      } u_right_encoder;
      u_right_encoder.real = this->right_encoder;
      *(outbuffer + offset + 0) = (u_right_encoder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_encoder.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_encoder.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_encoder.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_encoder.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_encoder.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_encoder.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_encoder.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_encoder);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->led =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->led);
      union {
        int64_t real;
        uint64_t base;
      } u_left_encoder;
      u_left_encoder.base = 0;
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_encoder = u_left_encoder.real;
      offset += sizeof(this->left_encoder);
      union {
        int64_t real;
        uint64_t base;
      } u_right_encoder;
      u_right_encoder.base = 0;
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_encoder.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_encoder = u_right_encoder.real;
      offset += sizeof(this->right_encoder);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
     return offset;
    }

    const char * getType(){ return "moobot_msgs/SensorState"; };
    const char * getMD5(){ return "84b397cf66be520b2eaf2f8ecf9e38be"; };

  };

}
#endif