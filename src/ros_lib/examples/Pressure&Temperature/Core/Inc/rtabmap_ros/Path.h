#ifndef _ROS_rtabmap_ros_Path_h
#define _ROS_rtabmap_ros_Path_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace rtabmap_ros
{

  class Path : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t nodeIds_length;
      typedef int32_t _nodeIds_type;
      _nodeIds_type st_nodeIds;
      _nodeIds_type * nodeIds;
      uint32_t poses_length;
      typedef geometry_msgs::Pose _poses_type;
      _poses_type st_poses;
      _poses_type * poses;

    Path():
      header(),
      nodeIds_length(0), nodeIds(NULL),
      poses_length(0), poses(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->nodeIds_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodeIds_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodeIds_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodeIds_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodeIds_length);
      for( uint32_t i = 0; i < nodeIds_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_nodeIdsi;
      u_nodeIdsi.real = this->nodeIds[i];
      *(outbuffer + offset + 0) = (u_nodeIdsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nodeIdsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nodeIdsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nodeIdsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodeIds[i]);
      }
      *(outbuffer + offset + 0) = (this->poses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->poses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->poses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->poses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->poses_length);
      for( uint32_t i = 0; i < poses_length; i++){
      offset += this->poses[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t nodeIds_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodeIds_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodeIds_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodeIds_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodeIds_length);
      if(nodeIds_lengthT > nodeIds_length)
        this->nodeIds = (int32_t*)realloc(this->nodeIds, nodeIds_lengthT * sizeof(int32_t));
      nodeIds_length = nodeIds_lengthT;
      for( uint32_t i = 0; i < nodeIds_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_nodeIds;
      u_st_nodeIds.base = 0;
      u_st_nodeIds.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_nodeIds.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_nodeIds.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_nodeIds.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_nodeIds = u_st_nodeIds.real;
      offset += sizeof(this->st_nodeIds);
        memcpy( &(this->nodeIds[i]), &(this->st_nodeIds), sizeof(int32_t));
      }
      uint32_t poses_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->poses_length);
      if(poses_lengthT > poses_length)
        this->poses = (geometry_msgs::Pose*)realloc(this->poses, poses_lengthT * sizeof(geometry_msgs::Pose));
      poses_length = poses_lengthT;
      for( uint32_t i = 0; i < poses_length; i++){
      offset += this->st_poses.deserialize(inbuffer + offset);
        memcpy( &(this->poses[i]), &(this->st_poses), sizeof(geometry_msgs::Pose));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/Path"; };
    const char * getMD5(){ return "ce3513b544acee43df74e3869c3272c0"; };

  };

}
#endif
