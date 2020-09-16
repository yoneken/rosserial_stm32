#ifndef _ROS_rtabmap_ros_UserData_h
#define _ROS_rtabmap_ros_UserData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace rtabmap_ros
{

  class UserData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _rows_type;
      _rows_type rows;
      typedef uint32_t _cols_type;
      _cols_type cols;
      typedef uint32_t _type_type;
      _type_type type;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;

    UserData():
      header(),
      rows(0),
      cols(0),
      type(0),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->rows >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rows >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rows >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rows >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rows);
      *(outbuffer + offset + 0) = (this->cols >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cols >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cols >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cols >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cols);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->type >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->type >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->type >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->rows =  ((uint32_t) (*(inbuffer + offset)));
      this->rows |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rows |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->rows |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->rows);
      this->cols =  ((uint32_t) (*(inbuffer + offset)));
      this->cols |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->cols |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->cols |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->cols);
      this->type =  ((uint32_t) (*(inbuffer + offset)));
      this->type |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->type |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->type |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->type);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint8_t*)realloc(this->data, data_lengthT * sizeof(uint8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/UserData"; };
    const char * getMD5(){ return "e2b3ca3c96ccd4baa19ca1aeef9ec767"; };

  };

}
#endif
