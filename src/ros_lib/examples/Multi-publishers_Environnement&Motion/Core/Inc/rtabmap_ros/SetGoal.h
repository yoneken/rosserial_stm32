#ifndef _ROS_SERVICE_SetGoal_h
#define _ROS_SERVICE_SetGoal_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace rtabmap_ros
{

static const char SETGOAL[] = "rtabmap_ros/SetGoal";

  class SetGoalRequest : public ros::Msg
  {
    public:
      typedef int32_t _node_id_type;
      _node_id_type node_id;
      typedef const char* _node_label_type;
      _node_label_type node_label;
      typedef const char* _frame_id_type;
      _frame_id_type frame_id;

    SetGoalRequest():
      node_id(0),
      node_label(""),
      frame_id("")
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

    const char * getType(){ return SETGOAL; };
    const char * getMD5(){ return "375ab24253ceefb71e0472c1b972fff4"; };

  };

  class SetGoalResponse : public ros::Msg
  {
    public:
      uint32_t path_ids_length;
      typedef int32_t _path_ids_type;
      _path_ids_type st_path_ids;
      _path_ids_type * path_ids;
      uint32_t path_poses_length;
      typedef geometry_msgs::Pose _path_poses_type;
      _path_poses_type st_path_poses;
      _path_poses_type * path_poses;
      typedef float _planning_time_type;
      _planning_time_type planning_time;

    SetGoalResponse():
      path_ids_length(0), path_ids(NULL),
      path_poses_length(0), path_poses(NULL),
      planning_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->path_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_ids_length);
      for( uint32_t i = 0; i < path_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_path_idsi;
      u_path_idsi.real = this->path_ids[i];
      *(outbuffer + offset + 0) = (u_path_idsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_path_idsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_path_idsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_path_idsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_ids[i]);
      }
      *(outbuffer + offset + 0) = (this->path_poses_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->path_poses_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->path_poses_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->path_poses_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->path_poses_length);
      for( uint32_t i = 0; i < path_poses_length; i++){
      offset += this->path_poses[i].serialize(outbuffer + offset);
      }
      union {
        float real;
        uint32_t base;
      } u_planning_time;
      u_planning_time.real = this->planning_time;
      *(outbuffer + offset + 0) = (u_planning_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_planning_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_planning_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_planning_time.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->planning_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t path_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_ids_length);
      if(path_ids_lengthT > path_ids_length)
        this->path_ids = (int32_t*)realloc(this->path_ids, path_ids_lengthT * sizeof(int32_t));
      path_ids_length = path_ids_lengthT;
      for( uint32_t i = 0; i < path_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_path_ids;
      u_st_path_ids.base = 0;
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_path_ids.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_path_ids = u_st_path_ids.real;
      offset += sizeof(this->st_path_ids);
        memcpy( &(this->path_ids[i]), &(this->st_path_ids), sizeof(int32_t));
      }
      uint32_t path_poses_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      path_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      path_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      path_poses_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->path_poses_length);
      if(path_poses_lengthT > path_poses_length)
        this->path_poses = (geometry_msgs::Pose*)realloc(this->path_poses, path_poses_lengthT * sizeof(geometry_msgs::Pose));
      path_poses_length = path_poses_lengthT;
      for( uint32_t i = 0; i < path_poses_length; i++){
      offset += this->st_path_poses.deserialize(inbuffer + offset);
        memcpy( &(this->path_poses[i]), &(this->st_path_poses), sizeof(geometry_msgs::Pose));
      }
      union {
        float real;
        uint32_t base;
      } u_planning_time;
      u_planning_time.base = 0;
      u_planning_time.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_planning_time.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_planning_time.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_planning_time.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->planning_time = u_planning_time.real;
      offset += sizeof(this->planning_time);
     return offset;
    }

    const char * getType(){ return SETGOAL; };
    const char * getMD5(){ return "adca6a85ab21f03d516676b74309de28"; };

  };

  class SetGoal {
    public:
    typedef SetGoalRequest Request;
    typedef SetGoalResponse Response;
  };

}
#endif
