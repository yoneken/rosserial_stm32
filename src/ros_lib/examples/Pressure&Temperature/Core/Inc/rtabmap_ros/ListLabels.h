#ifndef _ROS_SERVICE_ListLabels_h
#define _ROS_SERVICE_ListLabels_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rtabmap_ros
{

static const char LISTLABELS[] = "rtabmap_ros/ListLabels";

  class ListLabelsRequest : public ros::Msg
  {
    public:

    ListLabelsRequest()
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

    const char * getType(){ return LISTLABELS; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ListLabelsResponse : public ros::Msg
  {
    public:
      uint32_t ids_length;
      typedef int32_t _ids_type;
      _ids_type st_ids;
      _ids_type * ids;
      uint32_t labels_length;
      typedef char* _labels_type;
      _labels_type st_labels;
      _labels_type * labels;

    ListLabelsResponse():
      ids_length(0), ids(NULL),
      labels_length(0), labels(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids_length);
      for( uint32_t i = 0; i < ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_idsi;
      u_idsi.real = this->ids[i];
      *(outbuffer + offset + 0) = (u_idsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_idsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_idsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_idsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids[i]);
      }
      *(outbuffer + offset + 0) = (this->labels_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labels_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labels_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labels_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labels_length);
      for( uint32_t i = 0; i < labels_length; i++){
      uint32_t length_labelsi = strlen(this->labels[i]);
      varToArr(outbuffer + offset, length_labelsi);
      offset += 4;
      memcpy(outbuffer + offset, this->labels[i], length_labelsi);
      offset += length_labelsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ids_length);
      if(ids_lengthT > ids_length)
        this->ids = (int32_t*)realloc(this->ids, ids_lengthT * sizeof(int32_t));
      ids_length = ids_lengthT;
      for( uint32_t i = 0; i < ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_ids;
      u_st_ids.base = 0;
      u_st_ids.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ids.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ids.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ids.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ids = u_st_ids.real;
      offset += sizeof(this->st_ids);
        memcpy( &(this->ids[i]), &(this->st_ids), sizeof(int32_t));
      }
      uint32_t labels_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labels_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labels_length);
      if(labels_lengthT > labels_length)
        this->labels = (char**)realloc(this->labels, labels_lengthT * sizeof(char*));
      labels_length = labels_lengthT;
      for( uint32_t i = 0; i < labels_length; i++){
      uint32_t length_st_labels;
      arrToVar(length_st_labels, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_labels; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_labels-1]=0;
      this->st_labels = (char *)(inbuffer + offset-1);
      offset += length_st_labels;
        memcpy( &(this->labels[i]), &(this->st_labels), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return LISTLABELS; };
    const char * getMD5(){ return "eeaf81b9c58c5f0e5150e4704df1159c"; };

  };

  class ListLabels {
    public:
    typedef ListLabelsRequest Request;
    typedef ListLabelsResponse Response;
  };

}
#endif
