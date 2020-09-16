#ifndef _ROS_rtabmap_ros_MapData_h
#define _ROS_rtabmap_ros_MapData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/NodeData.h"

namespace rtabmap_ros
{

  class MapData : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef rtabmap_ros::MapGraph _graph_type;
      _graph_type graph;
      uint32_t nodes_length;
      typedef rtabmap_ros::NodeData _nodes_type;
      _nodes_type st_nodes;
      _nodes_type * nodes;

    MapData():
      header(),
      graph(),
      nodes_length(0), nodes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->graph.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->nodes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nodes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nodes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nodes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nodes_length);
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->nodes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->graph.deserialize(inbuffer + offset);
      uint32_t nodes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nodes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nodes_length);
      if(nodes_lengthT > nodes_length)
        this->nodes = (rtabmap_ros::NodeData*)realloc(this->nodes, nodes_lengthT * sizeof(rtabmap_ros::NodeData));
      nodes_length = nodes_lengthT;
      for( uint32_t i = 0; i < nodes_length; i++){
      offset += this->st_nodes.deserialize(inbuffer + offset);
        memcpy( &(this->nodes[i]), &(this->st_nodes), sizeof(rtabmap_ros::NodeData));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/MapData"; };
    const char * getMD5(){ return "363079139b951c370e18ce85bf2f22e2"; };

  };

}
#endif
