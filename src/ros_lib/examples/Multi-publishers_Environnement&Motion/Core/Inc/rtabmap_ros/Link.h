#ifndef _ROS_rtabmap_ros_Link_h
#define _ROS_rtabmap_ros_Link_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Transform.h"

namespace rtabmap_ros
{

  class Link : public ros::Msg
  {
    public:
      typedef int32_t _fromId_type;
      _fromId_type fromId;
      typedef int32_t _toId_type;
      _toId_type toId;
      typedef int32_t _type_type;
      _type_type type;
      typedef geometry_msgs::Transform _transform_type;
      _transform_type transform;
      double information[36];

    Link():
      fromId(0),
      toId(0),
      type(0),
      transform(),
      information()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_fromId;
      u_fromId.real = this->fromId;
      *(outbuffer + offset + 0) = (u_fromId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fromId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fromId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fromId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fromId);
      union {
        int32_t real;
        uint32_t base;
      } u_toId;
      u_toId.real = this->toId;
      *(outbuffer + offset + 0) = (u_toId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_toId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_toId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_toId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->toId);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->transform.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      union {
        double real;
        uint64_t base;
      } u_informationi;
      u_informationi.real = this->information[i];
      *(outbuffer + offset + 0) = (u_informationi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_informationi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_informationi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_informationi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_informationi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_informationi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_informationi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_informationi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->information[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_fromId;
      u_fromId.base = 0;
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fromId = u_fromId.real;
      offset += sizeof(this->fromId);
      union {
        int32_t real;
        uint32_t base;
      } u_toId;
      u_toId.base = 0;
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->toId = u_toId.real;
      offset += sizeof(this->toId);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += this->transform.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      union {
        double real;
        uint64_t base;
      } u_informationi;
      u_informationi.base = 0;
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_informationi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->information[i] = u_informationi.real;
      offset += sizeof(this->information[i]);
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/Link"; };
    const char * getMD5(){ return "48bafa7643c4731e90b468e7c4fa06b6"; };

  };

}
#endif
