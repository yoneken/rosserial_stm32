#ifndef _ROS_rtabmap_ros_RGBDImage_h
#define _ROS_rtabmap_ros_RGBDImage_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"

namespace rtabmap_ros
{

  class RGBDImage : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef sensor_msgs::CameraInfo _rgbCameraInfo_type;
      _rgbCameraInfo_type rgbCameraInfo;
      typedef sensor_msgs::CameraInfo _depthCameraInfo_type;
      _depthCameraInfo_type depthCameraInfo;
      typedef sensor_msgs::Image _rgb_type;
      _rgb_type rgb;
      typedef sensor_msgs::Image _depth_type;
      _depth_type depth;
      typedef sensor_msgs::CompressedImage _rgbCompressed_type;
      _rgbCompressed_type rgbCompressed;
      typedef sensor_msgs::CompressedImage _depthCompressed_type;
      _depthCompressed_type depthCompressed;

    RGBDImage():
      header(),
      rgbCameraInfo(),
      depthCameraInfo(),
      rgb(),
      depth(),
      rgbCompressed(),
      depthCompressed()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->rgbCameraInfo.serialize(outbuffer + offset);
      offset += this->depthCameraInfo.serialize(outbuffer + offset);
      offset += this->rgb.serialize(outbuffer + offset);
      offset += this->depth.serialize(outbuffer + offset);
      offset += this->rgbCompressed.serialize(outbuffer + offset);
      offset += this->depthCompressed.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->rgbCameraInfo.deserialize(inbuffer + offset);
      offset += this->depthCameraInfo.deserialize(inbuffer + offset);
      offset += this->rgb.deserialize(inbuffer + offset);
      offset += this->depth.deserialize(inbuffer + offset);
      offset += this->rgbCompressed.deserialize(inbuffer + offset);
      offset += this->depthCompressed.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/RGBDImage"; };
    const char * getMD5(){ return "a7fa8f34f44b976b3b2dec13c02f9f0e"; };

  };

}
#endif
