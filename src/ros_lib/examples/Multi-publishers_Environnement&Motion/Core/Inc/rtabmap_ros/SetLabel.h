#ifndef _ROS_SERVICE_SetLabel_h
#define _ROS_SERVICE_SetLabel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rtabmap_ros
{

static const char SETLABEL[] = "rtabmap_ros/SetLabel";

  class SetLabelRequest : public ros::Msg
  {
    public:
      typedef int32_t _node_id_type;
      _node_id_type node_id;
      typedef const char* _node_label_type;
      _node_label_type node_label;

    SetLabelRequest():
      node_id(0),
      node_label("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
     return offset;
    }

    const char * getType(){ return SETLABEL; };
    const char * getMD5(){ return "baadfb04a43ec26085eb7bebc9a80862"; };

  };

  class SetLabelResponse : public ros::Msg
  {
    public:

    SetLabelResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETLABEL; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetLabel {
    public:
    typedef SetLabelRequest Request;
    typedef SetLabelResponse Response;
  };

}
#endif
