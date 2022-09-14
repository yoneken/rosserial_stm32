#ifndef _ROS_rtabmap_ros_NodeData_h
#define _ROS_rtabmap_ros_NodeData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "rtabmap_ros/GPS.h"
#include "geometry_msgs/Transform.h"
#include "rtabmap_ros/Point3f.h"
#include "rtabmap_ros/KeyPoint.h"
#include "sensor_msgs/PointCloud2.h"

namespace rtabmap_ros
{

  class NodeData : public ros::Msg
  {
    public:
      typedef int32_t _id_type;
      _id_type id;
      typedef int32_t _mapId_type;
      _mapId_type mapId;
      typedef int32_t _weight_type;
      _weight_type weight;
      typedef double _stamp_type;
      _stamp_type stamp;
      typedef const char* _label_type;
      _label_type label;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Pose _groundTruthPose_type;
      _groundTruthPose_type groundTruthPose;
      typedef rtabmap_ros::GPS _gps_type;
      _gps_type gps;
      uint32_t image_length;
      typedef uint8_t _image_type;
      _image_type st_image;
      _image_type * image;
      uint32_t depth_length;
      typedef uint8_t _depth_type;
      _depth_type st_depth;
      _depth_type * depth;
      uint32_t fx_length;
      typedef float _fx_type;
      _fx_type st_fx;
      _fx_type * fx;
      uint32_t fy_length;
      typedef float _fy_type;
      _fy_type st_fy;
      _fy_type * fy;
      uint32_t cx_length;
      typedef float _cx_type;
      _cx_type st_cx;
      _cx_type * cx;
      uint32_t cy_length;
      typedef float _cy_type;
      _cy_type st_cy;
      _cy_type * cy;
      uint32_t width_length;
      typedef float _width_type;
      _width_type st_width;
      _width_type * width;
      uint32_t height_length;
      typedef float _height_type;
      _height_type st_height;
      _height_type * height;
      typedef float _baseline_type;
      _baseline_type baseline;
      uint32_t localTransform_length;
      typedef geometry_msgs::Transform _localTransform_type;
      _localTransform_type st_localTransform;
      _localTransform_type * localTransform;
      uint32_t laserScan_length;
      typedef uint8_t _laserScan_type;
      _laserScan_type st_laserScan;
      _laserScan_type * laserScan;
      typedef int32_t _laserScanMaxPts_type;
      _laserScanMaxPts_type laserScanMaxPts;
      typedef float _laserScanMaxRange_type;
      _laserScanMaxRange_type laserScanMaxRange;
      typedef int32_t _laserScanFormat_type;
      _laserScanFormat_type laserScanFormat;
      typedef geometry_msgs::Transform _laserScanLocalTransform_type;
      _laserScanLocalTransform_type laserScanLocalTransform;
      uint32_t userData_length;
      typedef uint8_t _userData_type;
      _userData_type st_userData;
      _userData_type * userData;
      uint32_t grid_ground_length;
      typedef uint8_t _grid_ground_type;
      _grid_ground_type st_grid_ground;
      _grid_ground_type * grid_ground;
      uint32_t grid_obstacles_length;
      typedef uint8_t _grid_obstacles_type;
      _grid_obstacles_type st_grid_obstacles;
      _grid_obstacles_type * grid_obstacles;
      uint32_t grid_empty_cells_length;
      typedef uint8_t _grid_empty_cells_type;
      _grid_empty_cells_type st_grid_empty_cells;
      _grid_empty_cells_type * grid_empty_cells;
      typedef float _grid_cell_size_type;
      _grid_cell_size_type grid_cell_size;
      typedef rtabmap_ros::Point3f _grid_view_point_type;
      _grid_view_point_type grid_view_point;
      uint32_t wordIds_length;
      typedef int32_t _wordIds_type;
      _wordIds_type st_wordIds;
      _wordIds_type * wordIds;
      uint32_t wordKpts_length;
      typedef rtabmap_ros::KeyPoint _wordKpts_type;
      _wordKpts_type st_wordKpts;
      _wordKpts_type * wordKpts;
      typedef sensor_msgs::PointCloud2 _wordPts_type;
      _wordPts_type wordPts;
      uint32_t descriptors_length;
      typedef uint8_t _descriptors_type;
      _descriptors_type st_descriptors;
      _descriptors_type * descriptors;

    NodeData():
      id(0),
      mapId(0),
      weight(0),
      stamp(0),
      label(""),
      pose(),
      groundTruthPose(),
      gps(),
      image_length(0), image(NULL),
      depth_length(0), depth(NULL),
      fx_length(0), fx(NULL),
      fy_length(0), fy(NULL),
      cx_length(0), cx(NULL),
      cy_length(0), cy(NULL),
      width_length(0), width(NULL),
      height_length(0), height(NULL),
      baseline(0),
      localTransform_length(0), localTransform(NULL),
      laserScan_length(0), laserScan(NULL),
      laserScanMaxPts(0),
      laserScanMaxRange(0),
      laserScanFormat(0),
      laserScanLocalTransform(),
      userData_length(0), userData(NULL),
      grid_ground_length(0), grid_ground(NULL),
      grid_obstacles_length(0), grid_obstacles(NULL),
      grid_empty_cells_length(0), grid_empty_cells(NULL),
      grid_cell_size(0),
      grid_view_point(),
      wordIds_length(0), wordIds(NULL),
      wordKpts_length(0), wordKpts(NULL),
      wordPts(),
      descriptors_length(0), descriptors(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_mapId;
      u_mapId.real = this->mapId;
      *(outbuffer + offset + 0) = (u_mapId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mapId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mapId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mapId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mapId);
      union {
        int32_t real;
        uint32_t base;
      } u_weight;
      u_weight.real = this->weight;
      *(outbuffer + offset + 0) = (u_weight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_weight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_weight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weight);
      union {
        double real;
        uint64_t base;
      } u_stamp;
      u_stamp.real = this->stamp;
      *(outbuffer + offset + 0) = (u_stamp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stamp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stamp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stamp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_stamp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_stamp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_stamp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_stamp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->stamp);
      uint32_t length_label = strlen(this->label);
      varToArr(outbuffer + offset, length_label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, length_label);
      offset += length_label;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->groundTruthPose.serialize(outbuffer + offset);
      offset += this->gps.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->image_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->image_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->image_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->image_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->image_length);
      for( uint32_t i = 0; i < image_length; i++){
      *(outbuffer + offset + 0) = (this->image[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->image[i]);
      }
      *(outbuffer + offset + 0) = (this->depth_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->depth_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->depth_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->depth_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->depth_length);
      for( uint32_t i = 0; i < depth_length; i++){
      *(outbuffer + offset + 0) = (this->depth[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->depth[i]);
      }
      *(outbuffer + offset + 0) = (this->fx_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fx_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fx_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fx_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fx_length);
      for( uint32_t i = 0; i < fx_length; i++){
      union {
        float real;
        uint32_t base;
      } u_fxi;
      u_fxi.real = this->fx[i];
      *(outbuffer + offset + 0) = (u_fxi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fxi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fxi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fxi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fx[i]);
      }
      *(outbuffer + offset + 0) = (this->fy_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->fy_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->fy_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->fy_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fy_length);
      for( uint32_t i = 0; i < fy_length; i++){
      union {
        float real;
        uint32_t base;
      } u_fyi;
      u_fyi.real = this->fy[i];
      *(outbuffer + offset + 0) = (u_fyi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fyi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fyi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fyi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fy[i]);
      }
      *(outbuffer + offset + 0) = (this->cx_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cx_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cx_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cx_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cx_length);
      for( uint32_t i = 0; i < cx_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cxi;
      u_cxi.real = this->cx[i];
      *(outbuffer + offset + 0) = (u_cxi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cxi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cxi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cxi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cx[i]);
      }
      *(outbuffer + offset + 0) = (this->cy_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cy_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cy_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cy_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cy_length);
      for( uint32_t i = 0; i < cy_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cyi;
      u_cyi.real = this->cy[i];
      *(outbuffer + offset + 0) = (u_cyi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cyi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cyi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cyi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cy[i]);
      }
      *(outbuffer + offset + 0) = (this->width_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width_length);
      for( uint32_t i = 0; i < width_length; i++){
      union {
        float real;
        uint32_t base;
      } u_widthi;
      u_widthi.real = this->width[i];
      *(outbuffer + offset + 0) = (u_widthi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_widthi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_widthi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_widthi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width[i]);
      }
      *(outbuffer + offset + 0) = (this->height_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height_length);
      for( uint32_t i = 0; i < height_length; i++){
      union {
        float real;
        uint32_t base;
      } u_heighti;
      u_heighti.real = this->height[i];
      *(outbuffer + offset + 0) = (u_heighti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heighti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heighti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heighti.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_baseline;
      u_baseline.real = this->baseline;
      *(outbuffer + offset + 0) = (u_baseline.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_baseline.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_baseline.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_baseline.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->baseline);
      *(outbuffer + offset + 0) = (this->localTransform_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->localTransform_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->localTransform_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->localTransform_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->localTransform_length);
      for( uint32_t i = 0; i < localTransform_length; i++){
      offset += this->localTransform[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->laserScan_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->laserScan_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->laserScan_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->laserScan_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->laserScan_length);
      for( uint32_t i = 0; i < laserScan_length; i++){
      *(outbuffer + offset + 0) = (this->laserScan[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->laserScan[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_laserScanMaxPts;
      u_laserScanMaxPts.real = this->laserScanMaxPts;
      *(outbuffer + offset + 0) = (u_laserScanMaxPts.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_laserScanMaxPts.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_laserScanMaxPts.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_laserScanMaxPts.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->laserScanMaxPts);
      union {
        float real;
        uint32_t base;
      } u_laserScanMaxRange;
      u_laserScanMaxRange.real = this->laserScanMaxRange;
      *(outbuffer + offset + 0) = (u_laserScanMaxRange.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_laserScanMaxRange.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_laserScanMaxRange.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_laserScanMaxRange.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->laserScanMaxRange);
      union {
        int32_t real;
        uint32_t base;
      } u_laserScanFormat;
      u_laserScanFormat.real = this->laserScanFormat;
      *(outbuffer + offset + 0) = (u_laserScanFormat.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_laserScanFormat.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_laserScanFormat.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_laserScanFormat.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->laserScanFormat);
      offset += this->laserScanLocalTransform.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->userData_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->userData_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->userData_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->userData_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->userData_length);
      for( uint32_t i = 0; i < userData_length; i++){
      *(outbuffer + offset + 0) = (this->userData[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->userData[i]);
      }
      *(outbuffer + offset + 0) = (this->grid_ground_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->grid_ground_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->grid_ground_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->grid_ground_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->grid_ground_length);
      for( uint32_t i = 0; i < grid_ground_length; i++){
      *(outbuffer + offset + 0) = (this->grid_ground[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->grid_ground[i]);
      }
      *(outbuffer + offset + 0) = (this->grid_obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->grid_obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->grid_obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->grid_obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->grid_obstacles_length);
      for( uint32_t i = 0; i < grid_obstacles_length; i++){
      *(outbuffer + offset + 0) = (this->grid_obstacles[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->grid_obstacles[i]);
      }
      *(outbuffer + offset + 0) = (this->grid_empty_cells_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->grid_empty_cells_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->grid_empty_cells_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->grid_empty_cells_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->grid_empty_cells_length);
      for( uint32_t i = 0; i < grid_empty_cells_length; i++){
      *(outbuffer + offset + 0) = (this->grid_empty_cells[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->grid_empty_cells[i]);
      }
      union {
        float real;
        uint32_t base;
      } u_grid_cell_size;
      u_grid_cell_size.real = this->grid_cell_size;
      *(outbuffer + offset + 0) = (u_grid_cell_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_grid_cell_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_grid_cell_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_grid_cell_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->grid_cell_size);
      offset += this->grid_view_point.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->wordIds_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wordIds_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wordIds_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wordIds_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wordIds_length);
      for( uint32_t i = 0; i < wordIds_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_wordIdsi;
      u_wordIdsi.real = this->wordIds[i];
      *(outbuffer + offset + 0) = (u_wordIdsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wordIdsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wordIdsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wordIdsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wordIds[i]);
      }
      *(outbuffer + offset + 0) = (this->wordKpts_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->wordKpts_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->wordKpts_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->wordKpts_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wordKpts_length);
      for( uint32_t i = 0; i < wordKpts_length; i++){
      offset += this->wordKpts[i].serialize(outbuffer + offset);
      }
      offset += this->wordPts.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->descriptors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->descriptors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->descriptors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->descriptors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->descriptors_length);
      for( uint32_t i = 0; i < descriptors_length; i++){
      *(outbuffer + offset + 0) = (this->descriptors[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->descriptors[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_mapId;
      u_mapId.base = 0;
      u_mapId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mapId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mapId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mapId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mapId = u_mapId.real;
      offset += sizeof(this->mapId);
      union {
        int32_t real;
        uint32_t base;
      } u_weight;
      u_weight.base = 0;
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_weight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->weight = u_weight.real;
      offset += sizeof(this->weight);
      union {
        double real;
        uint64_t base;
      } u_stamp;
      u_stamp.base = 0;
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_stamp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->stamp = u_stamp.real;
      offset += sizeof(this->stamp);
      uint32_t length_label;
      arrToVar(length_label, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_label; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_label-1]=0;
      this->label = (char *)(inbuffer + offset-1);
      offset += length_label;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->groundTruthPose.deserialize(inbuffer + offset);
      offset += this->gps.deserialize(inbuffer + offset);
      uint32_t image_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      image_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      image_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      image_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->image_length);
      if(image_lengthT > image_length)
        this->image = (uint8_t*)realloc(this->image, image_lengthT * sizeof(uint8_t));
      image_length = image_lengthT;
      for( uint32_t i = 0; i < image_length; i++){
      this->st_image =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_image);
        memcpy( &(this->image[i]), &(this->st_image), sizeof(uint8_t));
      }
      uint32_t depth_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      depth_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      depth_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      depth_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->depth_length);
      if(depth_lengthT > depth_length)
        this->depth = (uint8_t*)realloc(this->depth, depth_lengthT * sizeof(uint8_t));
      depth_length = depth_lengthT;
      for( uint32_t i = 0; i < depth_length; i++){
      this->st_depth =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_depth);
        memcpy( &(this->depth[i]), &(this->st_depth), sizeof(uint8_t));
      }
      uint32_t fx_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fx_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fx_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fx_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fx_length);
      if(fx_lengthT > fx_length)
        this->fx = (float*)realloc(this->fx, fx_lengthT * sizeof(float));
      fx_length = fx_lengthT;
      for( uint32_t i = 0; i < fx_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_fx;
      u_st_fx.base = 0;
      u_st_fx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_fx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_fx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_fx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_fx = u_st_fx.real;
      offset += sizeof(this->st_fx);
        memcpy( &(this->fx[i]), &(this->st_fx), sizeof(float));
      }
      uint32_t fy_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      fy_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      fy_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      fy_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->fy_length);
      if(fy_lengthT > fy_length)
        this->fy = (float*)realloc(this->fy, fy_lengthT * sizeof(float));
      fy_length = fy_lengthT;
      for( uint32_t i = 0; i < fy_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_fy;
      u_st_fy.base = 0;
      u_st_fy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_fy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_fy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_fy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_fy = u_st_fy.real;
      offset += sizeof(this->st_fy);
        memcpy( &(this->fy[i]), &(this->st_fy), sizeof(float));
      }
      uint32_t cx_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cx_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cx_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cx_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cx_length);
      if(cx_lengthT > cx_length)
        this->cx = (float*)realloc(this->cx, cx_lengthT * sizeof(float));
      cx_length = cx_lengthT;
      for( uint32_t i = 0; i < cx_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cx;
      u_st_cx.base = 0;
      u_st_cx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cx = u_st_cx.real;
      offset += sizeof(this->st_cx);
        memcpy( &(this->cx[i]), &(this->st_cx), sizeof(float));
      }
      uint32_t cy_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cy_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cy_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cy_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cy_length);
      if(cy_lengthT > cy_length)
        this->cy = (float*)realloc(this->cy, cy_lengthT * sizeof(float));
      cy_length = cy_lengthT;
      for( uint32_t i = 0; i < cy_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cy;
      u_st_cy.base = 0;
      u_st_cy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cy = u_st_cy.real;
      offset += sizeof(this->st_cy);
        memcpy( &(this->cy[i]), &(this->st_cy), sizeof(float));
      }
      uint32_t width_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      width_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      width_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      width_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->width_length);
      if(width_lengthT > width_length)
        this->width = (float*)realloc(this->width, width_lengthT * sizeof(float));
      width_length = width_lengthT;
      for( uint32_t i = 0; i < width_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_width;
      u_st_width.base = 0;
      u_st_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_width = u_st_width.real;
      offset += sizeof(this->st_width);
        memcpy( &(this->width[i]), &(this->st_width), sizeof(float));
      }
      uint32_t height_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      height_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      height_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      height_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->height_length);
      if(height_lengthT > height_length)
        this->height = (float*)realloc(this->height, height_lengthT * sizeof(float));
      height_length = height_lengthT;
      for( uint32_t i = 0; i < height_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_height;
      u_st_height.base = 0;
      u_st_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_height = u_st_height.real;
      offset += sizeof(this->st_height);
        memcpy( &(this->height[i]), &(this->st_height), sizeof(float));
      }
      union {
        float real;
        uint32_t base;
      } u_baseline;
      u_baseline.base = 0;
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_baseline.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->baseline = u_baseline.real;
      offset += sizeof(this->baseline);
      uint32_t localTransform_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      localTransform_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      localTransform_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      localTransform_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->localTransform_length);
      if(localTransform_lengthT > localTransform_length)
        this->localTransform = (geometry_msgs::Transform*)realloc(this->localTransform, localTransform_lengthT * sizeof(geometry_msgs::Transform));
      localTransform_length = localTransform_lengthT;
      for( uint32_t i = 0; i < localTransform_length; i++){
      offset += this->st_localTransform.deserialize(inbuffer + offset);
        memcpy( &(this->localTransform[i]), &(this->st_localTransform), sizeof(geometry_msgs::Transform));
      }
      uint32_t laserScan_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      laserScan_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      laserScan_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      laserScan_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->laserScan_length);
      if(laserScan_lengthT > laserScan_length)
        this->laserScan = (uint8_t*)realloc(this->laserScan, laserScan_lengthT * sizeof(uint8_t));
      laserScan_length = laserScan_lengthT;
      for( uint32_t i = 0; i < laserScan_length; i++){
      this->st_laserScan =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_laserScan);
        memcpy( &(this->laserScan[i]), &(this->st_laserScan), sizeof(uint8_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_laserScanMaxPts;
      u_laserScanMaxPts.base = 0;
      u_laserScanMaxPts.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_laserScanMaxPts.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_laserScanMaxPts.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_laserScanMaxPts.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->laserScanMaxPts = u_laserScanMaxPts.real;
      offset += sizeof(this->laserScanMaxPts);
      union {
        float real;
        uint32_t base;
      } u_laserScanMaxRange;
      u_laserScanMaxRange.base = 0;
      u_laserScanMaxRange.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_laserScanMaxRange.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_laserScanMaxRange.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_laserScanMaxRange.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->laserScanMaxRange = u_laserScanMaxRange.real;
      offset += sizeof(this->laserScanMaxRange);
      union {
        int32_t real;
        uint32_t base;
      } u_laserScanFormat;
      u_laserScanFormat.base = 0;
      u_laserScanFormat.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_laserScanFormat.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_laserScanFormat.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_laserScanFormat.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->laserScanFormat = u_laserScanFormat.real;
      offset += sizeof(this->laserScanFormat);
      offset += this->laserScanLocalTransform.deserialize(inbuffer + offset);
      uint32_t userData_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      userData_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      userData_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      userData_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->userData_length);
      if(userData_lengthT > userData_length)
        this->userData = (uint8_t*)realloc(this->userData, userData_lengthT * sizeof(uint8_t));
      userData_length = userData_lengthT;
      for( uint32_t i = 0; i < userData_length; i++){
      this->st_userData =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_userData);
        memcpy( &(this->userData[i]), &(this->st_userData), sizeof(uint8_t));
      }
      uint32_t grid_ground_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      grid_ground_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      grid_ground_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      grid_ground_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->grid_ground_length);
      if(grid_ground_lengthT > grid_ground_length)
        this->grid_ground = (uint8_t*)realloc(this->grid_ground, grid_ground_lengthT * sizeof(uint8_t));
      grid_ground_length = grid_ground_lengthT;
      for( uint32_t i = 0; i < grid_ground_length; i++){
      this->st_grid_ground =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_grid_ground);
        memcpy( &(this->grid_ground[i]), &(this->st_grid_ground), sizeof(uint8_t));
      }
      uint32_t grid_obstacles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      grid_obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      grid_obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      grid_obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->grid_obstacles_length);
      if(grid_obstacles_lengthT > grid_obstacles_length)
        this->grid_obstacles = (uint8_t*)realloc(this->grid_obstacles, grid_obstacles_lengthT * sizeof(uint8_t));
      grid_obstacles_length = grid_obstacles_lengthT;
      for( uint32_t i = 0; i < grid_obstacles_length; i++){
      this->st_grid_obstacles =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_grid_obstacles);
        memcpy( &(this->grid_obstacles[i]), &(this->st_grid_obstacles), sizeof(uint8_t));
      }
      uint32_t grid_empty_cells_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      grid_empty_cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      grid_empty_cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      grid_empty_cells_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->grid_empty_cells_length);
      if(grid_empty_cells_lengthT > grid_empty_cells_length)
        this->grid_empty_cells = (uint8_t*)realloc(this->grid_empty_cells, grid_empty_cells_lengthT * sizeof(uint8_t));
      grid_empty_cells_length = grid_empty_cells_lengthT;
      for( uint32_t i = 0; i < grid_empty_cells_length; i++){
      this->st_grid_empty_cells =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_grid_empty_cells);
        memcpy( &(this->grid_empty_cells[i]), &(this->st_grid_empty_cells), sizeof(uint8_t));
      }
      union {
        float real;
        uint32_t base;
      } u_grid_cell_size;
      u_grid_cell_size.base = 0;
      u_grid_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_grid_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_grid_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_grid_cell_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->grid_cell_size = u_grid_cell_size.real;
      offset += sizeof(this->grid_cell_size);
      offset += this->grid_view_point.deserialize(inbuffer + offset);
      uint32_t wordIds_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      wordIds_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      wordIds_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      wordIds_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->wordIds_length);
      if(wordIds_lengthT > wordIds_length)
        this->wordIds = (int32_t*)realloc(this->wordIds, wordIds_lengthT * sizeof(int32_t));
      wordIds_length = wordIds_lengthT;
      for( uint32_t i = 0; i < wordIds_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_wordIds;
      u_st_wordIds.base = 0;
      u_st_wordIds.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_wordIds.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_wordIds.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_wordIds.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_wordIds = u_st_wordIds.real;
      offset += sizeof(this->st_wordIds);
        memcpy( &(this->wordIds[i]), &(this->st_wordIds), sizeof(int32_t));
      }
      uint32_t wordKpts_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      wordKpts_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      wordKpts_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      wordKpts_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->wordKpts_length);
      if(wordKpts_lengthT > wordKpts_length)
        this->wordKpts = (rtabmap_ros::KeyPoint*)realloc(this->wordKpts, wordKpts_lengthT * sizeof(rtabmap_ros::KeyPoint));
      wordKpts_length = wordKpts_lengthT;
      for( uint32_t i = 0; i < wordKpts_length; i++){
      offset += this->st_wordKpts.deserialize(inbuffer + offset);
        memcpy( &(this->wordKpts[i]), &(this->st_wordKpts), sizeof(rtabmap_ros::KeyPoint));
      }
      offset += this->wordPts.deserialize(inbuffer + offset);
      uint32_t descriptors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      descriptors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      descriptors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      descriptors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->descriptors_length);
      if(descriptors_lengthT > descriptors_length)
        this->descriptors = (uint8_t*)realloc(this->descriptors, descriptors_lengthT * sizeof(uint8_t));
      descriptors_length = descriptors_lengthT;
      for( uint32_t i = 0; i < descriptors_length; i++){
      this->st_descriptors =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_descriptors);
        memcpy( &(this->descriptors[i]), &(this->st_descriptors), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/NodeData"; };
    const char * getMD5(){ return "9ea28782bcb5ac28722f26aaaa7acb18"; };

  };

}
#endif
