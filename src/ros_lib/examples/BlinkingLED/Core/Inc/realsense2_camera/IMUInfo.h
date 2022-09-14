#ifndef _ROS_realsense2_camera_IMUInfo_h
#define _ROS_realsense2_camera_IMUInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace realsense2_camera
{

  class IMUInfo : public ros::Msg
  {
    public:
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;
      double data[12];
      double noise_variances[3];
      double bias_variances[3];

    IMUInfo():
      frame_id(""),
      data(),
      noise_variances(),
      bias_variances()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_frame_id = strlen(this->frame_id);
      varToArr(outbuffer + offset, length_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, length_frame_id);
      offset += length_frame_id;
      for( uint32_t i = 0; i < 12; i++){
      union {
        double real;
        uint64_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_datai.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_datai.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_datai.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_datai.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_noise_variancesi;
      u_noise_variancesi.real = this->noise_variances[i];
      *(outbuffer + offset + 0) = (u_noise_variancesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_noise_variancesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_noise_variancesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_noise_variancesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_noise_variancesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_noise_variancesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_noise_variancesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_noise_variancesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->noise_variances[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_bias_variancesi;
      u_bias_variancesi.real = this->bias_variances[i];
      *(outbuffer + offset + 0) = (u_bias_variancesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bias_variancesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bias_variancesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bias_variancesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_bias_variancesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_bias_variancesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_bias_variancesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_bias_variancesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->bias_variances[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_frame_id;
      arrToVar(length_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_frame_id-1]=0;
      this->frame_id = (char *)(inbuffer + offset-1);
      offset += length_frame_id;
      for( uint32_t i = 0; i < 12; i++){
      union {
        double real;
        uint64_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_datai.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_noise_variancesi;
      u_noise_variancesi.base = 0;
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_noise_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->noise_variances[i] = u_noise_variancesi.real;
      offset += sizeof(this->noise_variances[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        double real;
        uint64_t base;
      } u_bias_variancesi;
      u_bias_variancesi.base = 0;
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_bias_variancesi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->bias_variances[i] = u_bias_variancesi.real;
      offset += sizeof(this->bias_variances[i]);
      }
     return offset;
    }

    const char * getType(){ return "realsense2_camera/IMUInfo"; };
    const char * getMD5(){ return "a02adb3a99530b11ba18a16f40f9512a"; };

  };

}
#endif
