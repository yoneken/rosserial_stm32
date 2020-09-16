#ifndef _ROS_rtabmap_ros_Goal_h
#define _ROS_rtabmap_ros_Goal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rtabmap_ros
{

  class Goal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _node_id_type;
      _node_id_type node_id;
      typedef const char* _node_label_type;
      _node_label_type node_label;
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;

    Goal():
      header(),
      node_id(0),
      node_label(""),
      frame_id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_node_id;
      u_node_id.real = this->node_id;
      *(outbuffer + offset + 0) = (u_node_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_node_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_node_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_node_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->node_id);
      uint32_t length_node_label = strlen(this->node_label);
      varToArr(outbuffer + offset, length_node_label);
      offset += 4;
      memcpy(outbuffer + offset, this->node_label, length_node_label);
      offset += length_node_label;
      uint32_t length_frame_id = strlen(this->frame_id);
      varToArr(outbuffer + offset, length_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, length_frame_id);
      offset += length_frame_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_node_id;
      u_node_id.base = 0;
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_node_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->node_id = u_node_id.real;
      offset += sizeof(this->node_id);
      uint32_t length_node_label;
      arrToVar(length_node_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_label-1]=0;
      this->node_label = (char *)(inbuffer + offset-1);
      offset += length_node_label;
      uint32_t length_frame_id;
      arrToVar(length_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_id-1]=0;
      this->frame_id = (char *)(inbuffer + offset-1);
      offset += length_frame_id;
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/Goal"; };
    const char * getMD5(){ return "70f8a16a321d4ec76904a9989a8798b7"; };

  };

}
#endif
