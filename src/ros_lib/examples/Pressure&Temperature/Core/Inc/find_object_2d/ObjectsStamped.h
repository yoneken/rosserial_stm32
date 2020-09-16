#ifndef _ROS_find_object_2d_ObjectsStamped_h
#define _ROS_find_object_2d_ObjectsStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32MultiArray.h"

namespace find_object_2d
{

  class ObjectsStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::Float32MultiArray _objects_type;
      _objects_type objects;

    ObjectsStamped():
      header(),
      objects()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->objects.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->objects.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "find_object_2d/ObjectsStamped"; };
    const char * getMD5(){ return "5ec2736b62b92d101276c97e8db387b1"; };

  };

}
#endif
