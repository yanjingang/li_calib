/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef VELODYNE_CORRECTION_HPP
#define VELODYNE_CORRECTION_HPP

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <angles/angles.h>
#include <iostream>
#include <vector>

#include <utils/pcl_utils.h>


namespace licalib {

class VelodyneCorrection {
public:
  typedef std::shared_ptr<VelodyneCorrection> Ptr;

  enum ModelType {
    VLP_16,
    HDL_32E,    // not support yet
    HESAI_40P,
    RS_16
  };

  VelodyneCorrection(ModelType modelType = VLP_16) : m_modelType(modelType) {
    setParameters(m_modelType);
  }

  void unpack_scan(const velodyne_msgs::VelodyneScan::ConstPtr &lidarMsg,
                   TPointCloud &outPointCloud) const {
    outPointCloud.clear();
    outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
    if (m_modelType == ModelType::VLP_16) {
      outPointCloud.height = 16;
      outPointCloud.width = 24*(int)lidarMsg->packets.size();
      outPointCloud.is_dense = false;
      outPointCloud.resize(outPointCloud.height * outPointCloud.width);
    }

    int block_counter = 0;

    double scan_timestamp = lidarMsg->header.stamp.toSec();

    for (size_t i = 0; i < lidarMsg->packets.size(); ++i) {
      float azimuth;
      float azimuth_diff;
      float last_azimuth_diff=0;
      float azimuth_corrected_f;
      int azimuth_corrected;
      float x, y, z;

      const raw_packet_t *raw = (const raw_packet_t *) &lidarMsg->packets[i].data[0];

      for (int block = 0; block < BLOCKS_PER_PACKET; block++, block_counter++) {
        // Calculate difference between current and next block's azimuth angle.
        azimuth = (float)(raw->blocks[block].rotation);

        if (block < (BLOCKS_PER_PACKET-1)){
          azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
          last_azimuth_diff = azimuth_diff;
        }
        else {
          azimuth_diff = last_azimuth_diff;
        }

        for (int firing=0, k=0; firing < FIRINGS_PER_BLOCK; firing++) {
          for (int dsr=0; dsr < N_SCAN; dsr++, k += RAW_SCAN_SIZE) {

            /** Position Calculation */
            union two_bytes tmp;
            tmp.bytes[0] = raw->blocks[block].data[k];
            tmp.bytes[1] = raw->blocks[block].data[k+1];

            /** correct for the laser rotation as a function of timing during the firings **/
            azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*DSR_TOFFSET) + (firing*FIRING_TOFFSET)) / BLOCK_TDURATION);
            azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

            /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
            if ((azimuth_corrected >= m_config.min_angle
                 && azimuth_corrected <= m_config.max_angle
                 && m_config.min_angle < m_config.max_angle)
                || (m_config.min_angle > m_config.max_angle
                    && (azimuth_corrected <= m_config.max_angle
                        || azimuth_corrected >= m_config.min_angle))) {
              // convert polar coordinates to Euclidean XYZ
              float distance = tmp.uint * DISTANCE_RESOLUTION;

              float cos_vert_angle = cos_vert_angle_[dsr];
              float sin_vert_angle = sin_vert_angle_[dsr];

              float cos_rot_angle = cos_rot_table_[azimuth_corrected];
              float sin_rot_angle = sin_rot_table_[azimuth_corrected];

              x = distance * cos_vert_angle * sin_rot_angle;
              y = distance * cos_vert_angle * cos_rot_angle;
              z = distance * sin_vert_angle;

              /** Use standard ROS coordinate system (right-hand rule) */
              float x_coord = y;
              float y_coord = -x;
              float z_coord = z;

              float intensity = raw->blocks[block].data[k+2];  // 反射率
              double point_timestamp;
              if (m_modelType == HESAI_40P) {
                point_timestamp = scan_timestamp + getExactTime(scan_mapping_40[dsr], 2*block_counter+firing);
              } else {
                point_timestamp = scan_timestamp + getExactTime(scan_mapping_16[dsr], 2*block_counter+firing);
              }

              TPoint point;
              point.timestamp = point_timestamp;
              if (pointInRange(distance)) {
                point.x = x_coord;
                point.y = y_coord;
                point.z = z_coord;
                point.intensity = intensity;
              } else {
                point.x = NAN;
                point.y = NAN;
                point.z = NAN;
                point.intensity = 0;
              }
              if(m_modelType == ModelType::VLP_16)
                outPointCloud.at(2*block_counter+firing, scan_mapping_16[dsr]) = point;
            }
          }
        }
      }
    }
  }


  void unpack_scan(const sensor_msgs::PointCloud2::ConstPtr &lidarMsg,
                   TPointCloud &outPointCloud) const {
    if (m_modelType == HESAI_40P) {
      VPointCloud temp_pc;
      pcl::fromROSMsg(*lidarMsg, temp_pc);
      outPointCloud.clear();
      outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
      outPointCloud.height = N_SCAN;  // 这针对的是雷达线数
      outPointCloud.width = temp_pc.size()/N_SCAN;
      outPointCloud.is_dense = lidarMsg->is_dense;
      outPointCloud.resize(outPointCloud.height * outPointCloud.width);

      double timebase = lidarMsg->header.stamp.toSec();
      for (int h = 0; h < outPointCloud.height; h++) {   // 两次遍历
        for (int w = 0; w < outPointCloud.width; w++) {
          TPoint point;
          pcl::PointXYZI& rs_point=temp_pc[w+h*temp_pc.size()/N_SCAN];
          point.x = rs_point.y * -1;
          point.y = rs_point.x;
          point.z = rs_point.z;
          point.intensity = rs_point.intensity;
          point.timestamp = timebase + getHesai40PExactTime(h,w);
          outPointCloud.at(w,h) = point;
        }
      }
    } else if (m_modelType == RS_16) {
      VPointCloud temp_pc;
      pcl::fromROSMsg(*lidarMsg, temp_pc);
      outPointCloud.clear();
      outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
      outPointCloud.height = N_SCAN;  // 这针对的是雷达线数
      outPointCloud.width = temp_pc.size()/N_SCAN;
      outPointCloud.is_dense = false;
      outPointCloud.resize(outPointCloud.height * outPointCloud.width);

      double timebase = lidarMsg->header.stamp.toSec();
      for (int h = 0; h < outPointCloud.height; h++) {   // 两次遍历
        for (int w = 0; w < outPointCloud.width; w++) {
          TPoint point;
          pcl::PointXYZI& rs_point=temp_pc[w+h*temp_pc.size()/N_SCAN];
          point.x = rs_point.x;
          point.y = rs_point.y;
          point.z = rs_point.z;
          point.intensity = rs_point.intensity;
          point.timestamp = timebase + getRS16ExactTime(h,w);
          outPointCloud.at(w,h) = point;
        }
      }

    } else {
      VPointCloud temp_pc;
      pcl::fromROSMsg(*lidarMsg, temp_pc);

      outPointCloud.clear();
      outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
      outPointCloud.height = temp_pc.height;
      outPointCloud.width = temp_pc.width;
      outPointCloud.is_dense = false;
      outPointCloud.resize(outPointCloud.height * outPointCloud.width);

      double timebase = lidarMsg->header.stamp.toSec();
      for (int h = 0; h < temp_pc.height; h++) {
        for (int w = 0; w < temp_pc.width; w++) {
          TPoint point;
          point.x = temp_pc.at(w,h).x;
          point.y = temp_pc.at(w,h).y;
          point.z = temp_pc.at(w,h).z;
          point.intensity = temp_pc.at(w,h).intensity;
          point.timestamp = timebase + getExactTime(h,w);
          outPointCloud.at(w,h) = point;
        }
      }
    }
  }


  inline double getExactTime(int dsr, int firing) const {
    return mVLP16TimeBlock[firing][dsr];
  }

  inline double getRS16ExactTime(int dsr, int firing) const{
    return mRS16TimeBlock[firing][dsr];
  }

  inline double getHesai40PExactTime(int dsr, int firing) const{
    return mHesai40PTimeBlock[firing][dsr];
  }

private:
  void setParameters(ModelType modelType) {
    m_modelType = modelType;
    m_config.max_range = 200;   // 最大测距
    m_config.min_range = 0.4;   // 盲区距离（为了降低操作人员对点云的干扰，可以适当设置大一些）
    m_config.min_angle = 0;     // 最小角度
    m_config.max_angle = 36000; // 最大水平视场角
    // Set up cached values for sin and cos of all the possible headings
    for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
      float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
      cos_rot_table_[rot_index] = cosf(rotation);
      sin_rot_table_[rot_index] = sinf(rotation);
    }

    if( modelType == VLP_16 ){
      FIRINGS_PER_BLOCK =   2;
      N_SCAN            =  16;     // 雷达线数
      HORIZON_SCAN      =  1824;   // 水平分辨率 = 水平视场角 / 水平角分辨率
      BLOCK_TDURATION   = 110.592f;   // [µs]
      DSR_TOFFSET       =   2.304f;   // [µs]
      FIRING_TOFFSET    =  55.296f;   // [µs]
      PACKET_TIME = (BLOCKS_PER_PACKET*2*FIRING_TOFFSET);

      float vert_correction[N_SCAN] = {
              -0.2617993877991494,
              0.017453292519943295,
              -0.22689280275926285,
              0.05235987755982989,
              -0.19198621771937624,
              0.08726646259971647,
              -0.15707963267948966,
              0.12217304763960307,
              -0.12217304763960307,
              0.15707963267948966,
              -0.08726646259971647,
              0.19198621771937624,
              -0.05235987755982989,
              0.22689280275926285,
              -0.017453292519943295,
              0.2617993877991494
      };
      for(int i = 0; i < N_SCAN; i++) {
        cos_vert_angle_[i] = std::cos(vert_correction[i]);
        sin_vert_angle_[i] = std::sin(vert_correction[i]);
      }
      scan_mapping_16[0]=15;
      scan_mapping_16[1]=7;
      scan_mapping_16[2]=14;
      scan_mapping_16[3]=6;
      scan_mapping_16[4]=13;
      scan_mapping_16[5]=5;
      scan_mapping_16[6]=12;
      scan_mapping_16[7]=4;
      scan_mapping_16[8]=11;
      scan_mapping_16[9]=3;
      scan_mapping_16[10]=10;
      scan_mapping_16[11]=2;
      scan_mapping_16[12]=9;
      scan_mapping_16[13]=1;
      scan_mapping_16[14]=8;
      scan_mapping_16[15]=0;

      for(unsigned int w = 0; w < HORIZON_SCAN; w++) {
        for(unsigned int h = 0; h < N_SCAN; h++) {
          mHesai40PTimeBlock[w][h] = h * 2.304 * 1e-6 + w * 55.296 * 1e-6; /// VLP_16 16*1824
        }
      }

    } else if( modelType == HESAI_40P ){ // 这里需要查找雷达用户手册
      FIRINGS_PER_BLOCK =   1;      // P39精确时间计算（用户手册）
      N_SCAN            =  40;      // 雷达线数 Pardar40P有40个通道
      HORIZON_SCAN      =  1800;    // 水平分辨率 = 水平视场角 / 水平角分辨率 = 360 / 0.2(10hz时) = 1800
      BLOCK_TDURATION   = 111.0f;   // [µs]
      DSR_TOFFSET       =   2.8f;   // [µs]
      FIRING_TOFFSET    =  55.5f;   // [µs]
      PACKET_TIME = (BLOCKS_PER_PACKET*2*FIRING_TOFFSET);

      // 垂直角度 ( 在官方的角度修正文件中的方位角Azimuth列 https://wwwcms.hesaitech.com/uploads/Pandar40_P_Angle_Correction_File_6cda8d0c59.csv )
      float vert_correction[N_SCAN] = { // 弧度制 参考P33
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            3.125
            -5.208
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
            -1.042
      };
      for(int i = 0; i < N_SCAN; i++) {
        cos_vert_angle_[i] = std::cos(vert_correction[i]);
        sin_vert_angle_[i] = std::sin(vert_correction[i]);
      }
      scan_mapping_40[0]=0;
      scan_mapping_40[1]=1;
      scan_mapping_40[2]=2;
      scan_mapping_40[3]=3;
      scan_mapping_40[4]=4;
      scan_mapping_40[5]=5;
      scan_mapping_40[6]=6;
      scan_mapping_40[7]=7;
      scan_mapping_40[8]=8;
      scan_mapping_40[9]=9;
      scan_mapping_40[10]=10;
      scan_mapping_40[11]=11;
      scan_mapping_40[12]=12;
      scan_mapping_40[13]=13;
      scan_mapping_40[14]=14;
      scan_mapping_40[15]=15;
      scan_mapping_40[16]=16;
      scan_mapping_40[17]=17;
      scan_mapping_40[18]=18;
      scan_mapping_40[19]=19;
      scan_mapping_40[20]=20;
      scan_mapping_40[21]=21;
      scan_mapping_40[22]=22;
      scan_mapping_40[23]=23;
      scan_mapping_40[24]=24;
      scan_mapping_40[25]=25;
      scan_mapping_40[26]=26;
      scan_mapping_40[27]=27;
      scan_mapping_40[28]=28;
      scan_mapping_40[29]=29;
      scan_mapping_40[30]=30;
      scan_mapping_40[31]=31;
      scan_mapping_40[32]=32;
      scan_mapping_40[33]=33;
      scan_mapping_40[34]=34;
      scan_mapping_40[35]=35;
      scan_mapping_40[36]=36;
      scan_mapping_40[37]=37;
      scan_mapping_40[38]=38;
      scan_mapping_40[39]=39;

      for(unsigned int w = 0; w < HORIZON_SCAN; w++) {
        for(unsigned int h = 0; h < N_SCAN; h++) {
          mRS16TimeBlock[w][h] = h * 2.8 * 1e-6 + w * 55.5 * 1e-6;
        }
      }
    } else if( modelType == RS_16 ){ // 这里需要查找雷达用户手册
      FIRINGS_PER_BLOCK =   2;      // P39精确时间计算（用户手册）
      N_SCAN            =  16;     // 雷达线数
      HORIZON_SCAN      =  1800;   // 水平分辨率 = 水平视场角 / 水平角分辨率
      BLOCK_TDURATION   = 111.0f;   // [µs]
      DSR_TOFFSET       =   2.8f;   // [µs]
      FIRING_TOFFSET    =  55.5f;   // [µs]
      PACKET_TIME = (BLOCKS_PER_PACKET*2*FIRING_TOFFSET);

      float vert_correction[N_SCAN] = { // 弧度制 参考P33
              -0.2617993877991494,
              -0.22689280275926285,
              -0.19198621771937624,
              -0.15707963267948966,
              -0.12217304763960307,
              -0.08726646259971647,
              -0.05235987755982989,
              -0.017453292519943295,
              0.017453292519943295,
              0.05235987755982989,
              0.08726646259971647,
              0.12217304763960307,
              0.15707963267948966,
              0.19198621771937624,
              0.22689280275926285,
              0.2617993877991494
      };
      for(int i = 0; i < N_SCAN; i++) {
        cos_vert_angle_[i] = std::cos(vert_correction[i]);
        sin_vert_angle_[i] = std::sin(vert_correction[i]);
      }
      scan_mapping_16[0]=0;
      scan_mapping_16[1]=1;
      scan_mapping_16[2]=2;
      scan_mapping_16[3]=3;
      scan_mapping_16[4]=4;
      scan_mapping_16[5]=5;
      scan_mapping_16[6]=6;
      scan_mapping_16[7]=7;
      scan_mapping_16[8]=8;
      scan_mapping_16[9]=9;
      scan_mapping_16[10]=10;
      scan_mapping_16[11]=11;
      scan_mapping_16[12]=12;
      scan_mapping_16[13]=13;
      scan_mapping_16[14]=14;
      scan_mapping_16[15]=15;

      for(unsigned int w = 0; w < HORIZON_SCAN; w++) {
        for(unsigned int h = 0; h < N_SCAN; h++) {
          mRS16TimeBlock[w][h] = h * 2.8 * 1e-6 + w * 55.5 * 1e-6;
        }
      }
    }
  }

  inline bool pointInRange(float range) const {
    return (range >= m_config.min_range
            && range <= m_config.max_range);
  }

private:
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE); //一个block中32个点数据（距离+强度）
  constexpr static const float ROTATION_RESOLUTION = 0.01f;             //角度分辨率
  static const uint16_t ROTATION_MAX_UNITS = 36000u;                    //角度最大值36000，除以100，即360
  constexpr static const float DISTANCE_RESOLUTION = 0.002f;            //距离分辨率

  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;

  static const int BLOCKS_PER_PACKET = 12;    //一个packet含有12个block
  static const int PACKET_STATUS_SIZE = 2;

  int    FIRINGS_PER_BLOCK;
  int    N_SCAN;        // 雷达线数
  int    HORIZON_SCAN;  // 水平分辨率 = 水平视场角 / 水平角分辨率
  float  BLOCK_TDURATION;
  float  DSR_TOFFSET;
  float  FIRING_TOFFSET;
  float  PACKET_TIME ;

  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];
  float cos_vert_angle_[32];
  float sin_vert_angle_[32];
  int scan_mapping_16[16];
  int scan_mapping_32[32];
  int scan_mapping_40[40];

  typedef struct raw_block {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK （起始标志符）
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees（角度值）
    uint8_t  data[BLOCK_DATA_SIZE]; //32个点的数据
  } raw_block_t;                    //block数据格式

  union two_bytes {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  union four_bytes {
    uint32_t uint32;
    float_t  float32;
  };

  typedef struct raw_packet {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint32_t revolution;
    uint8_t status[PACKET_STATUS_SIZE];
  } raw_packet_t;

  /** configuration parameters */
  typedef struct {
    double max_range;                ///< maximum range to publish
    double min_range;
    int min_angle;                   ///< minimum angle to publish
    int max_angle;                   ///< maximum angle to publish
  } Config;
  Config m_config;


  ModelType m_modelType;

  double mVLP16TimeBlock[1824][16];
  double mRS16TimeBlock[1800][16];
  double mHesai40PTimeBlock[1800][40];
};

}


#endif
