#ifndef _ROS_SERVICE_agv_mode_srv_h
#define _ROS_SERVICE_agv_mode_srv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace agv_mode_srvs
{

static const char AGV_MODE_SRV[] = "agv_mode_srvs/agv_mode_srv";

  class agv_mode_srvRequest : public ros::Msg
  {
    public:
      typedef int64_t _mode_type;
      _mode_type mode;

    agv_mode_srvRequest():
      mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mode.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mode.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mode.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mode.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mode.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mode.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
     return offset;
    }

    const char * getType(){ return AGV_MODE_SRV; };
    const char * getMD5(){ return "284404659b502753974e60f7457ed2dc"; };

  };

  class agv_mode_srvResponse : public ros::Msg
  {
    public:
      typedef int64_t _newmode_type;
      _newmode_type newmode;

    agv_mode_srvResponse():
      newmode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_newmode;
      u_newmode.real = this->newmode;
      *(outbuffer + offset + 0) = (u_newmode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_newmode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_newmode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_newmode.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_newmode.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_newmode.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_newmode.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_newmode.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->newmode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_newmode;
      u_newmode.base = 0;
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_newmode.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->newmode = u_newmode.real;
      offset += sizeof(this->newmode);
     return offset;
    }

    const char * getType(){ return AGV_MODE_SRV; };
    const char * getMD5(){ return "961eb286071c8202bbb98ce138adb626"; };

  };

  class agv_mode_srv {
    public:
    typedef agv_mode_srvRequest Request;
    typedef agv_mode_srvResponse Response;
  };

}
#endif
