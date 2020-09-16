#ifndef _ROS_rtabmap_ros_MapGraph_h
#define _ROS_rtabmap_ros_MapGraph_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include "rtabmap_ros/Link.h"

namespace rtabmap_ros
{

  class MapGraph : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Transform _mapToOdom_type;
      _mapToOdom_type mapToOdom;
      uint32_t posesId_length;
      typedef int32_t _posesId_type;
      _posesId_type st_posesId;
      _posesId_type * posesId;
      uint32_t poses_length;
      typedef geometry_msgs::Pose _poses_type;
      _poses_type st_poses;
      _poses_type * poses;
      uint32_t links_length;
      typedef rtabmap_ros::Link _links_type;
      _links_type st_links;
      _links_type * links;

    MapGraph():
      header(),
      mapToOdom(),
      posesId_length(0), posesId(NULL),
      poses_length(0), poses(NULL),
      links_length(0), links(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->mapToOdom.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->posesId_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->posesId_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->posesId_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->posesId_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posesId_length);
      for( uint32_t i = 0; i < posesId_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_posesIdi;
      u_posesIdi.real = this->posesId[i];
      *(outbuffer + offset + 0) = (u_posesIdi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posesIdi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posesIdi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posesIdi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posesId[i]);
      }
      *(outbuffer + offset + 0) = (this->poses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->poses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->poses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->poses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->poses_length);
      for( uint32_t i = 0; i < poses_length; i++){
      offset += this->poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->links_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->links_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->links_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->links_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->links_length);
      for( uint32_t i = 0; i < links_length; i++){
      offset += this->links[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->mapToOdom.deserialize(inbuffer + offset);
      uint32_t posesId_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      posesId_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      posesId_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      posesId_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->posesId_length);
      if(posesId_lengthT > posesId_length)
        this->posesId = (int32_t*)realloc(this->posesId, posesId_lengthT * sizeof(int32_t));
      posesId_length = posesId_lengthT;
      for( uint32_t i = 0; i < posesId_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_posesId;
      u_st_posesId.base = 0;
      u_st_posesId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_posesId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_posesId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_posesId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_posesId = u_st_posesId.real;
      offset += sizeof(this->st_posesId);
        memcpy( &(this->posesId[i]), &(this->st_posesId), sizeof(int32_t));
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
      uint32_t links_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      links_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      links_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      links_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->links_length);
      if(links_lengthT > links_length)
        this->links = (rtabmap_ros::Link*)realloc(this->links, links_lengthT * sizeof(rtabmap_ros::Link));
      links_length = links_lengthT;
      for( uint32_t i = 0; i < links_length; i++){
      offset += this->st_links.deserialize(inbuffer + offset);
        memcpy( &(this->links[i]), &(this->st_links), sizeof(rtabmap_ros::Link));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/MapGraph"; };
    const char * getMD5(){ return "903c2e44ed7e275e08ef323188cb38ee"; };

  };

}
#endif
