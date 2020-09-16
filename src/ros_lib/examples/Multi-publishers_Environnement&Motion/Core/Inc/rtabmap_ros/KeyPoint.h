#ifndef _ROS_rtabmap_ros_KeyPoint_h
#define _ROS_rtabmap_ros_KeyPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rtabmap_ros/Point2f.h"

namespace rtabmap_ros
{

  class KeyPoint : public ros::Msg
  {
    public:
      typedef rtabmap_ros::Point2f _pt_type;
      _pt_type pt;
      typedef float _size_type;
      _size_type size;
      typedef float _angle_type;
      _angle_type angle;
      typedef float _response_type;
      _response_type response;
      typedef int32_t _octave_type;
      _octave_type octave;
      typedef int32_t _class_id_type;
      _class_id_type class_id;

    KeyPoint():
      pt(),
      size(0),
      angle(0),
      response(0),
      octave(0),
      class_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pt.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_size;
      u_size.real = this->size;
      *(outbuffer + offset + 0) = (u_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        float real;
        uint32_t base;
      } u_response;
      u_response.real = this->response;
      *(outbuffer + offset + 0) = (u_response.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_response.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_response.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_response.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->response);
      union {
        int32_t real;
        uint32_t base;
      } u_octave;
      u_octave.real = this->octave;
      *(outbuffer + offset + 0) = (u_octave.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_octave.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_octave.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_octave.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->octave);
      union {
        int32_t real;
        uint32_t base;
      } u_class_id;
      u_class_id.real = this->class_id;
      *(outbuffer + offset + 0) = (u_class_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_class_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_class_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_class_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->class_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pt.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_size;
      u_size.base = 0;
      u_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->size = u_size.real;
      offset += sizeof(this->size);
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        float real;
        uint32_t base;
      } u_response;
      u_response.base = 0;
      u_response.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_response.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_response.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_response.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->response = u_response.real;
      offset += sizeof(this->response);
      union {
        int32_t real;
        uint32_t base;
      } u_octave;
      u_octave.base = 0;
      u_octave.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_octave.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_octave.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_octave.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->octave = u_octave.real;
      offset += sizeof(this->octave);
      union {
        int32_t real;
        uint32_t base;
      } u_class_id;
      u_class_id.base = 0;
      u_class_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_class_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_class_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_class_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->class_id = u_class_id.real;
      offset += sizeof(this->class_id);
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/KeyPoint"; };
    const char * getMD5(){ return "11cefb08bec6034bef3e32ec473dc6a7"; };

  };

}
#endif
