#ifndef _ROS_SERVICE_GetPlan_h
#define _ROS_SERVICE_GetPlan_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rtabmap_ros/Path.h"
#include "geometry_msgs/PoseStamped.h"

namespace rtabmap_ros
{

static const char GETPLAN[] = "rtabmap_ros/GetPlan";

  class GetPlanRequest : public ros::Msg
  {
    public:
      typedef int32_t _goal_node_type;
      _goal_node_type goal_node;
      typedef geometry_msgs::PoseStamped _goal_type;
      _goal_type goal;
      typedef float _tolerance_type;
      _tolerance_type tolerance;

    GetPlanRequest():
      goal_node(0),
      goal(),
      tolerance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_goal_node;
      u_goal_node.real = this->goal_node;
      *(outbuffer + offset + 0) = (u_goal_node.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_goal_node.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_goal_node.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_goal_node.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_node);
      offset += this->goal.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_tolerance;
      u_tolerance.real = this->tolerance;
      *(outbuffer + offset + 0) = (u_tolerance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tolerance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_goal_node;
      u_goal_node.base = 0;
      u_goal_node.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_goal_node.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_goal_node.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_goal_node.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->goal_node = u_goal_node.real;
      offset += sizeof(this->goal_node);
      offset += this->goal.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_tolerance;
      u_tolerance.base = 0;
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tolerance = u_tolerance.real;
      offset += sizeof(this->tolerance);
     return offset;
    }

    const char * getType(){ return GETPLAN; };
    const char * getMD5(){ return "93cea387b2aa9245414c000574ff1591"; };

  };

  class GetPlanResponse : public ros::Msg
  {
    public:
      typedef rtabmap_ros::Path _plan_type;
      _plan_type plan;

    GetPlanResponse():
      plan()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->plan.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->plan.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPLAN; };
    const char * getMD5(){ return "0412b9858bfcee4b2ee4fbf2f8eb5028"; };

  };

  class GetPlan {
    public:
    typedef GetPlanRequest Request;
    typedef GetPlanResponse Response;
  };

}
#endif
