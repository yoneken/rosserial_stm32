#ifndef _ROS_rtabmap_ros_Info_h
#define _ROS_rtabmap_ros_Info_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"

namespace rtabmap_ros
{

  class Info : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _refId_type;
      _refId_type refId;
      typedef int32_t _loopClosureId_type;
      _loopClosureId_type loopClosureId;
      typedef int32_t _proximityDetectionId_type;
      _proximityDetectionId_type proximityDetectionId;
      typedef geometry_msgs::Transform _loopClosureTransform_type;
      _loopClosureTransform_type loopClosureTransform;
      uint32_t posteriorKeys_length;
      typedef int32_t _posteriorKeys_type;
      _posteriorKeys_type st_posteriorKeys;
      _posteriorKeys_type * posteriorKeys;
      uint32_t posteriorValues_length;
      typedef float _posteriorValues_type;
      _posteriorValues_type st_posteriorValues;
      _posteriorValues_type * posteriorValues;
      uint32_t likelihoodKeys_length;
      typedef int32_t _likelihoodKeys_type;
      _likelihoodKeys_type st_likelihoodKeys;
      _likelihoodKeys_type * likelihoodKeys;
      uint32_t likelihoodValues_length;
      typedef float _likelihoodValues_type;
      _likelihoodValues_type st_likelihoodValues;
      _likelihoodValues_type * likelihoodValues;
      uint32_t rawLikelihoodKeys_length;
      typedef int32_t _rawLikelihoodKeys_type;
      _rawLikelihoodKeys_type st_rawLikelihoodKeys;
      _rawLikelihoodKeys_type * rawLikelihoodKeys;
      uint32_t rawLikelihoodValues_length;
      typedef float _rawLikelihoodValues_type;
      _rawLikelihoodValues_type st_rawLikelihoodValues;
      _rawLikelihoodValues_type * rawLikelihoodValues;
      uint32_t weightsKeys_length;
      typedef int32_t _weightsKeys_type;
      _weightsKeys_type st_weightsKeys;
      _weightsKeys_type * weightsKeys;
      uint32_t weightsValues_length;
      typedef int32_t _weightsValues_type;
      _weightsValues_type st_weightsValues;
      _weightsValues_type * weightsValues;
      uint32_t labelsKeys_length;
      typedef int32_t _labelsKeys_type;
      _labelsKeys_type st_labelsKeys;
      _labelsKeys_type * labelsKeys;
      uint32_t labelsValues_length;
      typedef char* _labelsValues_type;
      _labelsValues_type st_labelsValues;
      _labelsValues_type * labelsValues;
      uint32_t statsKeys_length;
      typedef char* _statsKeys_type;
      _statsKeys_type st_statsKeys;
      _statsKeys_type * statsKeys;
      uint32_t statsValues_length;
      typedef float _statsValues_type;
      _statsValues_type st_statsValues;
      _statsValues_type * statsValues;
      uint32_t localPath_length;
      typedef int32_t _localPath_type;
      _localPath_type st_localPath;
      _localPath_type * localPath;
      typedef int32_t _currentGoalId_type;
      _currentGoalId_type currentGoalId;

    Info():
      header(),
      refId(0),
      loopClosureId(0),
      proximityDetectionId(0),
      loopClosureTransform(),
      posteriorKeys_length(0), posteriorKeys(NULL),
      posteriorValues_length(0), posteriorValues(NULL),
      likelihoodKeys_length(0), likelihoodKeys(NULL),
      likelihoodValues_length(0), likelihoodValues(NULL),
      rawLikelihoodKeys_length(0), rawLikelihoodKeys(NULL),
      rawLikelihoodValues_length(0), rawLikelihoodValues(NULL),
      weightsKeys_length(0), weightsKeys(NULL),
      weightsValues_length(0), weightsValues(NULL),
      labelsKeys_length(0), labelsKeys(NULL),
      labelsValues_length(0), labelsValues(NULL),
      statsKeys_length(0), statsKeys(NULL),
      statsValues_length(0), statsValues(NULL),
      localPath_length(0), localPath(NULL),
      currentGoalId(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_refId;
      u_refId.real = this->refId;
      *(outbuffer + offset + 0) = (u_refId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_refId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_refId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_refId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->refId);
      union {
        int32_t real;
        uint32_t base;
      } u_loopClosureId;
      u_loopClosureId.real = this->loopClosureId;
      *(outbuffer + offset + 0) = (u_loopClosureId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_loopClosureId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_loopClosureId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_loopClosureId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->loopClosureId);
      union {
        int32_t real;
        uint32_t base;
      } u_proximityDetectionId;
      u_proximityDetectionId.real = this->proximityDetectionId;
      *(outbuffer + offset + 0) = (u_proximityDetectionId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_proximityDetectionId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_proximityDetectionId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_proximityDetectionId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->proximityDetectionId);
      offset += this->loopClosureTransform.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->posteriorKeys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->posteriorKeys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->posteriorKeys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->posteriorKeys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posteriorKeys_length);
      for( uint32_t i = 0; i < posteriorKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_posteriorKeysi;
      u_posteriorKeysi.real = this->posteriorKeys[i];
      *(outbuffer + offset + 0) = (u_posteriorKeysi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posteriorKeysi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posteriorKeysi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posteriorKeysi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posteriorKeys[i]);
      }
      *(outbuffer + offset + 0) = (this->posteriorValues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->posteriorValues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->posteriorValues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->posteriorValues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posteriorValues_length);
      for( uint32_t i = 0; i < posteriorValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_posteriorValuesi;
      u_posteriorValuesi.real = this->posteriorValues[i];
      *(outbuffer + offset + 0) = (u_posteriorValuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_posteriorValuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_posteriorValuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_posteriorValuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->posteriorValues[i]);
      }
      *(outbuffer + offset + 0) = (this->likelihoodKeys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->likelihoodKeys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->likelihoodKeys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->likelihoodKeys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->likelihoodKeys_length);
      for( uint32_t i = 0; i < likelihoodKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_likelihoodKeysi;
      u_likelihoodKeysi.real = this->likelihoodKeys[i];
      *(outbuffer + offset + 0) = (u_likelihoodKeysi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_likelihoodKeysi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_likelihoodKeysi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_likelihoodKeysi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->likelihoodKeys[i]);
      }
      *(outbuffer + offset + 0) = (this->likelihoodValues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->likelihoodValues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->likelihoodValues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->likelihoodValues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->likelihoodValues_length);
      for( uint32_t i = 0; i < likelihoodValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_likelihoodValuesi;
      u_likelihoodValuesi.real = this->likelihoodValues[i];
      *(outbuffer + offset + 0) = (u_likelihoodValuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_likelihoodValuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_likelihoodValuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_likelihoodValuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->likelihoodValues[i]);
      }
      *(outbuffer + offset + 0) = (this->rawLikelihoodKeys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rawLikelihoodKeys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rawLikelihoodKeys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rawLikelihoodKeys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rawLikelihoodKeys_length);
      for( uint32_t i = 0; i < rawLikelihoodKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_rawLikelihoodKeysi;
      u_rawLikelihoodKeysi.real = this->rawLikelihoodKeys[i];
      *(outbuffer + offset + 0) = (u_rawLikelihoodKeysi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rawLikelihoodKeysi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rawLikelihoodKeysi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rawLikelihoodKeysi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rawLikelihoodKeys[i]);
      }
      *(outbuffer + offset + 0) = (this->rawLikelihoodValues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rawLikelihoodValues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->rawLikelihoodValues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->rawLikelihoodValues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rawLikelihoodValues_length);
      for( uint32_t i = 0; i < rawLikelihoodValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_rawLikelihoodValuesi;
      u_rawLikelihoodValuesi.real = this->rawLikelihoodValues[i];
      *(outbuffer + offset + 0) = (u_rawLikelihoodValuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rawLikelihoodValuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rawLikelihoodValuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rawLikelihoodValuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rawLikelihoodValues[i]);
      }
      *(outbuffer + offset + 0) = (this->weightsKeys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->weightsKeys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->weightsKeys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->weightsKeys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weightsKeys_length);
      for( uint32_t i = 0; i < weightsKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_weightsKeysi;
      u_weightsKeysi.real = this->weightsKeys[i];
      *(outbuffer + offset + 0) = (u_weightsKeysi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weightsKeysi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_weightsKeysi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_weightsKeysi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weightsKeys[i]);
      }
      *(outbuffer + offset + 0) = (this->weightsValues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->weightsValues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->weightsValues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->weightsValues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weightsValues_length);
      for( uint32_t i = 0; i < weightsValues_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_weightsValuesi;
      u_weightsValuesi.real = this->weightsValues[i];
      *(outbuffer + offset + 0) = (u_weightsValuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weightsValuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_weightsValuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_weightsValuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weightsValues[i]);
      }
      *(outbuffer + offset + 0) = (this->labelsKeys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labelsKeys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labelsKeys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labelsKeys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labelsKeys_length);
      for( uint32_t i = 0; i < labelsKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_labelsKeysi;
      u_labelsKeysi.real = this->labelsKeys[i];
      *(outbuffer + offset + 0) = (u_labelsKeysi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_labelsKeysi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_labelsKeysi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_labelsKeysi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labelsKeys[i]);
      }
      *(outbuffer + offset + 0) = (this->labelsValues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->labelsValues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->labelsValues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->labelsValues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->labelsValues_length);
      for( uint32_t i = 0; i < labelsValues_length; i++){
      uint32_t length_labelsValuesi = strlen(this->labelsValues[i]);
      varToArr(outbuffer + offset, length_labelsValuesi);
      offset += 4;
      memcpy(outbuffer + offset, this->labelsValues[i], length_labelsValuesi);
      offset += length_labelsValuesi;
      }
      *(outbuffer + offset + 0) = (this->statsKeys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->statsKeys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->statsKeys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->statsKeys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->statsKeys_length);
      for( uint32_t i = 0; i < statsKeys_length; i++){
      uint32_t length_statsKeysi = strlen(this->statsKeys[i]);
      varToArr(outbuffer + offset, length_statsKeysi);
      offset += 4;
      memcpy(outbuffer + offset, this->statsKeys[i], length_statsKeysi);
      offset += length_statsKeysi;
      }
      *(outbuffer + offset + 0) = (this->statsValues_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->statsValues_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->statsValues_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->statsValues_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->statsValues_length);
      for( uint32_t i = 0; i < statsValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_statsValuesi;
      u_statsValuesi.real = this->statsValues[i];
      *(outbuffer + offset + 0) = (u_statsValuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_statsValuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_statsValuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_statsValuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->statsValues[i]);
      }
      *(outbuffer + offset + 0) = (this->localPath_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->localPath_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->localPath_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->localPath_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->localPath_length);
      for( uint32_t i = 0; i < localPath_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_localPathi;
      u_localPathi.real = this->localPath[i];
      *(outbuffer + offset + 0) = (u_localPathi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_localPathi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_localPathi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_localPathi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->localPath[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_currentGoalId;
      u_currentGoalId.real = this->currentGoalId;
      *(outbuffer + offset + 0) = (u_currentGoalId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentGoalId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentGoalId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentGoalId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentGoalId);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_refId;
      u_refId.base = 0;
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_refId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->refId = u_refId.real;
      offset += sizeof(this->refId);
      union {
        int32_t real;
        uint32_t base;
      } u_loopClosureId;
      u_loopClosureId.base = 0;
      u_loopClosureId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_loopClosureId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_loopClosureId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_loopClosureId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->loopClosureId = u_loopClosureId.real;
      offset += sizeof(this->loopClosureId);
      union {
        int32_t real;
        uint32_t base;
      } u_proximityDetectionId;
      u_proximityDetectionId.base = 0;
      u_proximityDetectionId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_proximityDetectionId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_proximityDetectionId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_proximityDetectionId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->proximityDetectionId = u_proximityDetectionId.real;
      offset += sizeof(this->proximityDetectionId);
      offset += this->loopClosureTransform.deserialize(inbuffer + offset);
      uint32_t posteriorKeys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      posteriorKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      posteriorKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      posteriorKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->posteriorKeys_length);
      if(posteriorKeys_lengthT > posteriorKeys_length)
        this->posteriorKeys = (int32_t*)realloc(this->posteriorKeys, posteriorKeys_lengthT * sizeof(int32_t));
      posteriorKeys_length = posteriorKeys_lengthT;
      for( uint32_t i = 0; i < posteriorKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_posteriorKeys;
      u_st_posteriorKeys.base = 0;
      u_st_posteriorKeys.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_posteriorKeys.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_posteriorKeys.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_posteriorKeys.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_posteriorKeys = u_st_posteriorKeys.real;
      offset += sizeof(this->st_posteriorKeys);
        memcpy( &(this->posteriorKeys[i]), &(this->st_posteriorKeys), sizeof(int32_t));
      }
      uint32_t posteriorValues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      posteriorValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      posteriorValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      posteriorValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->posteriorValues_length);
      if(posteriorValues_lengthT > posteriorValues_length)
        this->posteriorValues = (float*)realloc(this->posteriorValues, posteriorValues_lengthT * sizeof(float));
      posteriorValues_length = posteriorValues_lengthT;
      for( uint32_t i = 0; i < posteriorValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_posteriorValues;
      u_st_posteriorValues.base = 0;
      u_st_posteriorValues.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_posteriorValues.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_posteriorValues.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_posteriorValues.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_posteriorValues = u_st_posteriorValues.real;
      offset += sizeof(this->st_posteriorValues);
        memcpy( &(this->posteriorValues[i]), &(this->st_posteriorValues), sizeof(float));
      }
      uint32_t likelihoodKeys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      likelihoodKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      likelihoodKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      likelihoodKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->likelihoodKeys_length);
      if(likelihoodKeys_lengthT > likelihoodKeys_length)
        this->likelihoodKeys = (int32_t*)realloc(this->likelihoodKeys, likelihoodKeys_lengthT * sizeof(int32_t));
      likelihoodKeys_length = likelihoodKeys_lengthT;
      for( uint32_t i = 0; i < likelihoodKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_likelihoodKeys;
      u_st_likelihoodKeys.base = 0;
      u_st_likelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_likelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_likelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_likelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_likelihoodKeys = u_st_likelihoodKeys.real;
      offset += sizeof(this->st_likelihoodKeys);
        memcpy( &(this->likelihoodKeys[i]), &(this->st_likelihoodKeys), sizeof(int32_t));
      }
      uint32_t likelihoodValues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      likelihoodValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      likelihoodValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      likelihoodValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->likelihoodValues_length);
      if(likelihoodValues_lengthT > likelihoodValues_length)
        this->likelihoodValues = (float*)realloc(this->likelihoodValues, likelihoodValues_lengthT * sizeof(float));
      likelihoodValues_length = likelihoodValues_lengthT;
      for( uint32_t i = 0; i < likelihoodValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_likelihoodValues;
      u_st_likelihoodValues.base = 0;
      u_st_likelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_likelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_likelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_likelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_likelihoodValues = u_st_likelihoodValues.real;
      offset += sizeof(this->st_likelihoodValues);
        memcpy( &(this->likelihoodValues[i]), &(this->st_likelihoodValues), sizeof(float));
      }
      uint32_t rawLikelihoodKeys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rawLikelihoodKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rawLikelihoodKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rawLikelihoodKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rawLikelihoodKeys_length);
      if(rawLikelihoodKeys_lengthT > rawLikelihoodKeys_length)
        this->rawLikelihoodKeys = (int32_t*)realloc(this->rawLikelihoodKeys, rawLikelihoodKeys_lengthT * sizeof(int32_t));
      rawLikelihoodKeys_length = rawLikelihoodKeys_lengthT;
      for( uint32_t i = 0; i < rawLikelihoodKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_rawLikelihoodKeys;
      u_st_rawLikelihoodKeys.base = 0;
      u_st_rawLikelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_rawLikelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_rawLikelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_rawLikelihoodKeys.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_rawLikelihoodKeys = u_st_rawLikelihoodKeys.real;
      offset += sizeof(this->st_rawLikelihoodKeys);
        memcpy( &(this->rawLikelihoodKeys[i]), &(this->st_rawLikelihoodKeys), sizeof(int32_t));
      }
      uint32_t rawLikelihoodValues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      rawLikelihoodValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      rawLikelihoodValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      rawLikelihoodValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->rawLikelihoodValues_length);
      if(rawLikelihoodValues_lengthT > rawLikelihoodValues_length)
        this->rawLikelihoodValues = (float*)realloc(this->rawLikelihoodValues, rawLikelihoodValues_lengthT * sizeof(float));
      rawLikelihoodValues_length = rawLikelihoodValues_lengthT;
      for( uint32_t i = 0; i < rawLikelihoodValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_rawLikelihoodValues;
      u_st_rawLikelihoodValues.base = 0;
      u_st_rawLikelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_rawLikelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_rawLikelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_rawLikelihoodValues.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_rawLikelihoodValues = u_st_rawLikelihoodValues.real;
      offset += sizeof(this->st_rawLikelihoodValues);
        memcpy( &(this->rawLikelihoodValues[i]), &(this->st_rawLikelihoodValues), sizeof(float));
      }
      uint32_t weightsKeys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      weightsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      weightsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      weightsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->weightsKeys_length);
      if(weightsKeys_lengthT > weightsKeys_length)
        this->weightsKeys = (int32_t*)realloc(this->weightsKeys, weightsKeys_lengthT * sizeof(int32_t));
      weightsKeys_length = weightsKeys_lengthT;
      for( uint32_t i = 0; i < weightsKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_weightsKeys;
      u_st_weightsKeys.base = 0;
      u_st_weightsKeys.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_weightsKeys.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_weightsKeys.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_weightsKeys.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_weightsKeys = u_st_weightsKeys.real;
      offset += sizeof(this->st_weightsKeys);
        memcpy( &(this->weightsKeys[i]), &(this->st_weightsKeys), sizeof(int32_t));
      }
      uint32_t weightsValues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      weightsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      weightsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      weightsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->weightsValues_length);
      if(weightsValues_lengthT > weightsValues_length)
        this->weightsValues = (int32_t*)realloc(this->weightsValues, weightsValues_lengthT * sizeof(int32_t));
      weightsValues_length = weightsValues_lengthT;
      for( uint32_t i = 0; i < weightsValues_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_weightsValues;
      u_st_weightsValues.base = 0;
      u_st_weightsValues.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_weightsValues.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_weightsValues.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_weightsValues.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_weightsValues = u_st_weightsValues.real;
      offset += sizeof(this->st_weightsValues);
        memcpy( &(this->weightsValues[i]), &(this->st_weightsValues), sizeof(int32_t));
      }
      uint32_t labelsKeys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labelsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labelsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labelsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labelsKeys_length);
      if(labelsKeys_lengthT > labelsKeys_length)
        this->labelsKeys = (int32_t*)realloc(this->labelsKeys, labelsKeys_lengthT * sizeof(int32_t));
      labelsKeys_length = labelsKeys_lengthT;
      for( uint32_t i = 0; i < labelsKeys_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_labelsKeys;
      u_st_labelsKeys.base = 0;
      u_st_labelsKeys.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_labelsKeys.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_labelsKeys.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_labelsKeys.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_labelsKeys = u_st_labelsKeys.real;
      offset += sizeof(this->st_labelsKeys);
        memcpy( &(this->labelsKeys[i]), &(this->st_labelsKeys), sizeof(int32_t));
      }
      uint32_t labelsValues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      labelsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      labelsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      labelsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->labelsValues_length);
      if(labelsValues_lengthT > labelsValues_length)
        this->labelsValues = (char**)realloc(this->labelsValues, labelsValues_lengthT * sizeof(char*));
      labelsValues_length = labelsValues_lengthT;
      for( uint32_t i = 0; i < labelsValues_length; i++){
      uint32_t length_st_labelsValues;
      arrToVar(length_st_labelsValues, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_labelsValues; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_labelsValues-1]=0;
      this->st_labelsValues = (char *)(inbuffer + offset-1);
      offset += length_st_labelsValues;
        memcpy( &(this->labelsValues[i]), &(this->st_labelsValues), sizeof(char*));
      }
      uint32_t statsKeys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      statsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      statsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      statsKeys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->statsKeys_length);
      if(statsKeys_lengthT > statsKeys_length)
        this->statsKeys = (char**)realloc(this->statsKeys, statsKeys_lengthT * sizeof(char*));
      statsKeys_length = statsKeys_lengthT;
      for( uint32_t i = 0; i < statsKeys_length; i++){
      uint32_t length_st_statsKeys;
      arrToVar(length_st_statsKeys, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_statsKeys; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_statsKeys-1]=0;
      this->st_statsKeys = (char *)(inbuffer + offset-1);
      offset += length_st_statsKeys;
        memcpy( &(this->statsKeys[i]), &(this->st_statsKeys), sizeof(char*));
      }
      uint32_t statsValues_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      statsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      statsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      statsValues_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->statsValues_length);
      if(statsValues_lengthT > statsValues_length)
        this->statsValues = (float*)realloc(this->statsValues, statsValues_lengthT * sizeof(float));
      statsValues_length = statsValues_lengthT;
      for( uint32_t i = 0; i < statsValues_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_statsValues;
      u_st_statsValues.base = 0;
      u_st_statsValues.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_statsValues.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_statsValues.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_statsValues.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_statsValues = u_st_statsValues.real;
      offset += sizeof(this->st_statsValues);
        memcpy( &(this->statsValues[i]), &(this->st_statsValues), sizeof(float));
      }
      uint32_t localPath_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      localPath_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      localPath_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      localPath_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->localPath_length);
      if(localPath_lengthT > localPath_length)
        this->localPath = (int32_t*)realloc(this->localPath, localPath_lengthT * sizeof(int32_t));
      localPath_length = localPath_lengthT;
      for( uint32_t i = 0; i < localPath_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_localPath;
      u_st_localPath.base = 0;
      u_st_localPath.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_localPath.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_localPath.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_localPath.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_localPath = u_st_localPath.real;
      offset += sizeof(this->st_localPath);
        memcpy( &(this->localPath[i]), &(this->st_localPath), sizeof(int32_t));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_currentGoalId;
      u_currentGoalId.base = 0;
      u_currentGoalId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentGoalId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentGoalId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentGoalId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentGoalId = u_currentGoalId.real;
      offset += sizeof(this->currentGoalId);
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/Info"; };
    const char * getMD5(){ return "f9cc2e215f6cae05161c74d102ffc592"; };

  };

}
#endif
