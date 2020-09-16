#ifndef _ROS_SERVICE_GetMap_h
#define _ROS_SERVICE_GetMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rtabmap_ros/MapData.h"

namespace rtabmap_ros
{

static const char GETMAP[] = "rtabmap_ros/GetMap";

  class GetMapRequest : public ros::Msg
  {
    public:
      typedef bool _global_type;
      _global_type global;
      typedef bool _optimized_type;
      _optimized_type optimized;
      typedef bool _graphOnly_type;
      _graphOnly_type graphOnly;

    GetMapRequest():
      global(0),
      optimized(0),
      graphOnly(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_global;
      u_global.real = this->global;
      *(outbuffer + offset + 0) = (u_global.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->global);
      union {
        bool real;
        uint8_t base;
      } u_optimized;
      u_optimized.real = this->optimized;
      *(outbuffer + offset + 0) = (u_optimized.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->optimized);
      union {
        bool real;
        uint8_t base;
      } u_graphOnly;
      u_graphOnly.real = this->graphOnly;
      *(outbuffer + offset + 0) = (u_graphOnly.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->graphOnly);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_global;
      u_global.base = 0;
      u_global.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->global = u_global.real;
      offset += sizeof(this->global);
      union {
        bool real;
        uint8_t base;
      } u_optimized;
      u_optimized.base = 0;
      u_optimized.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->optimized = u_optimized.real;
      offset += sizeof(this->optimized);
      union {
        bool real;
        uint8_t base;
      } u_graphOnly;
      u_graphOnly.base = 0;
      u_graphOnly.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->graphOnly = u_graphOnly.real;
      offset += sizeof(this->graphOnly);
     return offset;
    }

    const char * getType(){ return GETMAP; };
    const char * getMD5(){ return "6213f9f13cced23f4d224b22f59d839c"; };

  };

  class GetMapResponse : public ros::Msg
  {
    public:
      typedef rtabmap_ros::MapData _data_type;
      _data_type data;

    GetMapResponse():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETMAP; };
    const char * getMD5(){ return "be1186efbdcde47a41d62ec7fd4dcf0a"; };

  };

  class GetMap {
    public:
    typedef GetMapRequest Request;
    typedef GetMapResponse Response;
  };

}
#endif
