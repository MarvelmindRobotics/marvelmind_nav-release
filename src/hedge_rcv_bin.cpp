/*
Copyright (c) 2020, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "marvelmind_nav/hedge_pos.h"
#include "marvelmind_nav/hedge_pos_a.h"
#include "marvelmind_nav/hedge_pos_ang.h"
#include "marvelmind_nav/beacon_pos_a.h"
#include "marvelmind_nav/hedge_imu_raw.h"
#include "marvelmind_nav/hedge_imu_fusion.h"
#include "marvelmind_nav/beacon_distance.h"
#include "marvelmind_nav/hedge_telemetry.h"
#include "marvelmind_nav/hedge_quality.h"
#include "marvelmind_nav/marvelmind_waypoint.h"
extern "C" 
{
#include "marvelmind_nav/marvelmind_hedge.h"
}

#include <sstream>

#define ROS_NODE_NAME "hedge_rcv_bin"
#define HEDGE_POSITION_TOPIC_NAME "hedge_pos"
#define HEDGE_POSITION_ADDRESSED_TOPIC_NAME "hedge_pos_a"
#define HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME "hedge_pos_ang"
#define BEACONS_POSITION_ADDRESSED_TOPIC_NAME "beacons_pos_a"
#define HEDGE_IMU_RAW_TOPIC_NAME "hedge_imu_raw"
#define HEDGE_IMU_FUSION_TOPIC_NAME "hedge_imu_fusion"
#define BEACON_RAW_DISTANCE_TOPIC_NAME "beacon_raw_distance"
#define HEDGE_TELEMETRY_TOPIC_NAME "hedge_telemetry"
#define HEDGE_QUALITY_TOPIC_NAME "hedge_quality"
#define MARVELMIND_WAYPOINT_TOPIC_NAME "marvelmind_waypoint"

struct MarvelmindHedge * hedge= NULL;

static uint32_t hedge_timestamp_prev= 0;
marvelmind_nav::hedge_pos hedge_pos_noaddress_msg;// hedge coordinates message (old version without address) for publishing to ROS topic
marvelmind_nav::hedge_pos_a hedge_pos_msg;// hedge coordinates message for publishing to ROS topic
marvelmind_nav::hedge_pos_ang hedge_pos_ang_msg;// hedge coordinates and angle message for publishing to ROS topic
marvelmind_nav::beacon_pos_a beacon_pos_msg;// stationary beacon coordinates message for publishing to ROS topic
marvelmind_nav::hedge_imu_raw hedge_imu_raw_msg;// raw IMU data message for publishing to ROS topic
marvelmind_nav::hedge_imu_fusion hedge_imu_fusion_msg;// IMU fusion data message for publishing to ROS topic
marvelmind_nav::beacon_distance beacon_raw_distance_msg;// Raw distance message for publishing to ROS topic
marvelmind_nav::hedge_telemetry hedge_telemetry_msg;// Telemetry message for publishing to ROS topic
marvelmind_nav::hedge_quality hedge_quality_msg;// Quality message for publishing to ROS topic
marvelmind_nav::marvelmind_waypoint marvelmind_waypoint_msg;// Waypoint message for publishing to ROS topic

static sem_t *sem;
struct timespec ts;

////////////////////////////////////////////////////////////////////////

void semCallback()
{
	sem_post(sem);
}

static int hedgeReceivePrepare(int argc, char **argv)
{
	 // get port name from command line arguments (if specified)
    const char * ttyFileName;
    uint32_t baudRate;
    if (argc>=2) ttyFileName=argv[1];
      else ttyFileName=DEFAULT_TTY_FILENAME;
    if (argc>=3) baudRate= atoi(argv[2]);
      else baudRate=DEFAULT_TTY_BAUDRATE;
    
    // Init
    hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        ROS_INFO ("Error: Unable to create MarvelmindHedge");
        return -1;
    }
    hedge->ttyFileName=ttyFileName;
    hedge->baudRate= baudRate;
    hedge->verbose=true; // show errors and warnings
    hedge->anyInputPacketCallback= semCallback;
    startMarvelmindHedge (hedge);
}

static bool hedgeReceiveCheck(void)
{
  if (hedge->haveNewValues_)
    {
        struct PositionValue position;
        getPositionFromMarvelmindHedge (hedge, &position);
        
        hedge_pos_msg.address= position.address;
        hedge_pos_ang_msg.address= position.address;
        
        hedge_pos_msg.flags= position.flags;
        hedge_pos_noaddress_msg.flags= position.flags;
        hedge_pos_ang_msg.flags= position.flags;
        if (hedge_pos_msg.flags&(1<<1))// flag of timestamp format 
          {
			hedge_pos_msg.timestamp_ms= position.timestamp;// msec
			hedge_pos_noaddress_msg.timestamp_ms= position.timestamp;
		  }	
	     else 
	      {
            hedge_pos_msg.timestamp_ms= position.timestamp*15.625;// alpha-cycles ==> msec
            hedge_pos_noaddress_msg.timestamp_ms= position.timestamp*15.625;
          } 
        hedge_pos_ang_msg.timestamp_ms= position.timestamp;
          
        hedge_pos_msg.x_m= position.x/1000.0; 
        hedge_pos_msg.y_m= position.y/1000.0; 
        hedge_pos_msg.z_m= position.z/1000.0; 
        
        hedge_pos_noaddress_msg.x_m= position.x/1000.0; 
        hedge_pos_noaddress_msg.y_m= position.y/1000.0; 
        hedge_pos_noaddress_msg.z_m= position.z/1000.0;
        
        hedge_pos_ang_msg.x_m= position.x/1000.0; 
        hedge_pos_ang_msg.y_m= position.y/1000.0; 
        hedge_pos_ang_msg.z_m= position.z/1000.0;
        
        hedge_pos_ang_msg.angle= position.angle;
        
        hedge->haveNewValues_=false;
        
        return true;
    }
   return false;
}

static bool beaconReceiveCheck(void)
{
  uint8_t i;
  struct StationaryBeaconsPositions positions;
  struct StationaryBeaconPosition *bp= NULL;
  bool foundUpd= false;
  uint8_t n;
	
  getStationaryBeaconsPositionsFromMarvelmindHedge (hedge, &positions);
  n= positions.numBeacons;
  if (n == 0) 
	return false;
  
  for(i=0;i<n;i++)
  {
	  bp= &positions.beacons[i];
	  if (bp->updatedForMsg)
	  {
		  clearStationaryBeaconUpdatedFlag(hedge, bp->address);
		  foundUpd= true;
		  break;
	  } 
  }
  if (!foundUpd)
	return false;
  if (bp == NULL) 
	return false;
  	      
  beacon_pos_msg.address= bp->address;
  beacon_pos_msg.x_m= bp->x/1000.0; 
  beacon_pos_msg.y_m= bp->y/1000.0; 
  beacon_pos_msg.z_m= bp->z/1000.0; 
  
  return true;
}

static bool hedgeIMURawReceiveCheck(void)
{
  if (!hedge->rawIMU.updated)
     return false;
     
  hedge_imu_raw_msg.acc_x= hedge->rawIMU.acc_x;
  hedge_imu_raw_msg.acc_y= hedge->rawIMU.acc_y;
  hedge_imu_raw_msg.acc_z= hedge->rawIMU.acc_z;
  
  hedge_imu_raw_msg.gyro_x= hedge->rawIMU.gyro_x;
  hedge_imu_raw_msg.gyro_y= hedge->rawIMU.gyro_y;
  hedge_imu_raw_msg.gyro_z= hedge->rawIMU.gyro_z;
  
  hedge_imu_raw_msg.compass_x= hedge->rawIMU.compass_x;
  hedge_imu_raw_msg.compass_y= hedge->rawIMU.compass_y;
  hedge_imu_raw_msg.compass_z= hedge->rawIMU.compass_z;
  
  hedge_imu_raw_msg.timestamp_ms= hedge->rawIMU.timestamp;
  
  hedge->rawIMU.updated= false;
  
  return true;
}

static bool hedgeIMUFusionReceiveCheck(void)
{
  if (!hedge->fusionIMU.updated)
     return false;
     
  hedge_imu_fusion_msg.x_m= hedge->fusionIMU.x/1000.0;
  hedge_imu_fusion_msg.y_m= hedge->fusionIMU.y/1000.0;
  hedge_imu_fusion_msg.z_m= hedge->fusionIMU.z/1000.0;
  
  hedge_imu_fusion_msg.qw= hedge->fusionIMU.qw/10000.0;
  hedge_imu_fusion_msg.qx= hedge->fusionIMU.qx/10000.0;
  hedge_imu_fusion_msg.qy= hedge->fusionIMU.qy/10000.0;
  hedge_imu_fusion_msg.qz= hedge->fusionIMU.qz/10000.0;
  
  hedge_imu_fusion_msg.vx= hedge->fusionIMU.vx/1000.0;
  hedge_imu_fusion_msg.vy= hedge->fusionIMU.vy/1000.0;
  hedge_imu_fusion_msg.vz= hedge->fusionIMU.vz/1000.0;
  
  hedge_imu_fusion_msg.ax= hedge->fusionIMU.ax/1000.0;
  hedge_imu_fusion_msg.ay= hedge->fusionIMU.ay/1000.0;
  hedge_imu_fusion_msg.az= hedge->fusionIMU.az/1000.0;
  
  hedge_imu_fusion_msg.timestamp_ms= hedge->fusionIMU.timestamp;
  
  hedge->fusionIMU.updated= false;
  
  return true;
}

static void getRawDistance(uint8_t index)
{   
  beacon_raw_distance_msg.address_hedge= hedge->rawDistances.address_hedge;
  beacon_raw_distance_msg.address_beacon= hedge->rawDistances.distances[index].address_beacon;
  beacon_raw_distance_msg.distance_m= hedge->rawDistances.distances[index].distance/1000.0;
}

static bool hedgeTelemetryUpdateCheck(void)
{
	if (!hedge->telemetry.updated) 
		return false;
		
	hedge_telemetry_msg.battery_voltage= hedge->telemetry.vbat_mv/1000.0;
	hedge_telemetry_msg.rssi_dbm= hedge->telemetry.rssi_dbm;
		
	hedge->telemetry.updated= false;
	return true;
}

static bool hedgeQualityUpdateCheck(void)
{
	if (!hedge->quality.updated) 
		return false;
		
	hedge_quality_msg.address= hedge->quality.address;
	hedge_quality_msg.quality_percents= hedge->quality.quality_per;
		
	hedge->quality.updated= false;
	return true;
}

static bool marvelmindWaypointUpdateCheck(void)
{uint8_t i,n;
 uint8_t nUpdated;
	
	if (!hedge->waypoints.updated) 
		return false;
		
	nUpdated= 0;
    n= hedge->waypoints.numItems;
    for(i=0;i<n;i++)
    {
		if (!hedge->waypoints.items[i].updated) 
		  continue;
		  
		nUpdated++;
		if (nUpdated == 1)
		{
		  marvelmind_waypoint_msg.total_items= n;
		  marvelmind_waypoint_msg.item_index= i;
		  
		  marvelmind_waypoint_msg.movement_type= hedge->waypoints.items[i].movementType;
		  marvelmind_waypoint_msg.param1= hedge->waypoints.items[i].param1;
		  marvelmind_waypoint_msg.param2= hedge->waypoints.items[i].param2;
		  marvelmind_waypoint_msg.param3= hedge->waypoints.items[i].param3;
		  
	      hedge->waypoints.items[i].updated= false;
	    }
	}		
		
	if (nUpdated==1) 
	{
		hedge->waypoints.updated= false;
	}
	return (nUpdated>0);
}

/**
 * Node for Marvelmind hedgehog binary streaming data processing
 */
int main(int argc, char **argv)
{uint8_t beaconReadIterations;
  // initialize ROS node
  ros::init(argc, argv, ROS_NODE_NAME);
  
  sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);
  
  // prepare hedgehog data receiver module
  hedgeReceivePrepare(argc, argv);

  // ROS node reference 
  ros::NodeHandle n;

  // Register topics for puplishing messages
  ros::Publisher hedge_pos_ang_publisher = n.advertise<marvelmind_nav::hedge_pos_ang>(HEDGE_POSITION_WITH_ANGLE_TOPIC_NAME, 1000);
  ros::Publisher hedge_pos_publisher = n.advertise<marvelmind_nav::hedge_pos_a>(HEDGE_POSITION_ADDRESSED_TOPIC_NAME, 1000);
  ros::Publisher hedge_pos_noaddress_publisher = n.advertise<marvelmind_nav::hedge_pos>(HEDGE_POSITION_TOPIC_NAME, 1000);
  
  ros::Publisher beacons_pos_publisher = n.advertise<marvelmind_nav::beacon_pos_a>(BEACONS_POSITION_ADDRESSED_TOPIC_NAME, 1000);
  
  ros::Publisher hedge_imu_raw_publisher = n.advertise<marvelmind_nav::hedge_imu_raw>(HEDGE_IMU_RAW_TOPIC_NAME, 1000);
  ros::Publisher hedge_imu_fusion_publisher = n.advertise<marvelmind_nav::hedge_imu_fusion>(HEDGE_IMU_FUSION_TOPIC_NAME, 1000);
  
  ros::Publisher beacon_distance_publisher = n.advertise<marvelmind_nav::beacon_distance>(BEACON_RAW_DISTANCE_TOPIC_NAME, 1000);
  
  ros::Publisher hedge_telemetry_publisher = n.advertise<marvelmind_nav::hedge_telemetry>(HEDGE_TELEMETRY_TOPIC_NAME, 1000);
  ros::Publisher hedge_quality_publisher = n.advertise<marvelmind_nav::hedge_quality>(HEDGE_QUALITY_TOPIC_NAME, 1000);
  
  ros::Publisher marvelmind_waypoint_publisher = n.advertise<marvelmind_nav::marvelmind_waypoint>(MARVELMIND_WAYPOINT_TOPIC_NAME, 1000);

  // 200 Hz 
  ros::Rate loop_rate(200);

  // default values for position message
  hedge_pos_ang_msg.address= 0;
  hedge_pos_ang_msg.timestamp_ms = 0;
  hedge_pos_ang_msg.x_m = 0.0;
  hedge_pos_ang_msg.y_m = 0.0;
  hedge_pos_ang_msg.z_m = 0.0;
  hedge_pos_ang_msg.flags = (1<<0);// 'data not available' flag
  hedge_pos_ang_msg.angle= 0.0;
  
  hedge_pos_msg.address= 0;
  hedge_pos_msg.timestamp_ms = 0;
  hedge_pos_msg.x_m = 0.0;
  hedge_pos_msg.y_m = 0.0;
  hedge_pos_msg.z_m = 0.0;
  hedge_pos_msg.flags = (1<<0);// 'data not available' flag
  
  hedge_pos_noaddress_msg.timestamp_ms = 0;
  hedge_pos_noaddress_msg.x_m = 0.0;
  hedge_pos_noaddress_msg.y_m = 0.0;
  hedge_pos_noaddress_msg.z_m = 0.0;
  hedge_pos_noaddress_msg.flags = (1<<0);// 'data not available' flag
  
  beacon_pos_msg.address= 0;
  beacon_pos_msg.x_m = 0.0;
  beacon_pos_msg.y_m = 0.0;
  beacon_pos_msg.z_m = 0.0;

  
  while (ros::ok())
  {
    if (hedge->terminationRequired)
      {
		  break;
      }	
       
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
     {
        ROS_INFO("clock_gettime");
        return -1;
	 }
    ts.tv_sec += 2;
    sem_timedwait(sem,&ts);  
	  
    if (hedgeReceiveCheck())
     {// hedgehog data received
		ROS_INFO("Address: %d, timestamp: %d, %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d", 	
				(int) hedge_pos_ang_msg.address,
				(int) hedge_pos_ang_msg.timestamp_ms, 
				(int) (hedge_pos_ang_msg.timestamp_ms - hedge_timestamp_prev),
				(float) hedge_pos_ang_msg.x_m, (float) hedge_pos_ang_msg.y_m, (float) hedge_pos_ang_msg.z_m, 
				(float) hedge_pos_ang_msg.angle,
				(int) hedge_pos_msg.flags);
		hedge_pos_ang_publisher.publish(hedge_pos_ang_msg);
        hedge_pos_publisher.publish(hedge_pos_msg);
        hedge_pos_noaddress_publisher.publish(hedge_pos_noaddress_msg);
        
        hedge_timestamp_prev= hedge_pos_ang_msg.timestamp_ms;
     }   
     
    beaconReadIterations= 0; 
    while(beaconReceiveCheck())
     {// stationary beacons data received
		ROS_INFO("Stationary beacon: Address: %d, X=%.3f  Y= %.3f  Z=%.3f", 	
				(int) beacon_pos_msg.address,
				(float) beacon_pos_msg.x_m, (float) beacon_pos_msg.y_m, (float) beacon_pos_msg.z_m);
        beacons_pos_publisher.publish(beacon_pos_msg);
        
        if ((beaconReadIterations++)>4)
          break;
     }
     
    if (hedgeIMURawReceiveCheck())
    {
		ROS_INFO("Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d", 	
				(int) hedge_imu_raw_msg.timestamp_ms,
				(int) hedge_imu_raw_msg.acc_x, (int) hedge_imu_raw_msg.acc_y, (int) hedge_imu_raw_msg.acc_z,
				(int) hedge_imu_raw_msg.gyro_x, (int) hedge_imu_raw_msg.gyro_y, (int) hedge_imu_raw_msg.gyro_z,
				(int) hedge_imu_raw_msg.compass_x, (int) hedge_imu_raw_msg.compass_y, (int) hedge_imu_raw_msg.compass_z);
		hedge_imu_raw_publisher.publish(hedge_imu_raw_msg);
    } 
    
    if (hedgeIMUFusionReceiveCheck())
    {
		ROS_INFO("IMU fusion: Timestamp: %08d, X=%.3f  Y= %.3f  Z=%.3f  q=%.3f,%.3f,%.3f,%.3f v=%.3f,%.3f,%.3f  a=%.3f,%.3f,%.3f", 	
				(int) hedge_imu_fusion_msg.timestamp_ms,
				(float) hedge_imu_fusion_msg.x_m, (float) hedge_imu_fusion_msg.y_m, (float) hedge_imu_fusion_msg.z_m,
				(float) hedge_imu_fusion_msg.qw, (float) hedge_imu_fusion_msg.qx, (float) hedge_imu_fusion_msg.qy, (float) hedge_imu_fusion_msg.qz,
				(float) hedge_imu_fusion_msg.vx, (float) hedge_imu_fusion_msg.vy, (float) hedge_imu_fusion_msg.vz,
				(float) hedge_imu_fusion_msg.ax, (float) hedge_imu_fusion_msg.ay, (float) hedge_imu_fusion_msg.az);
		hedge_imu_fusion_publisher.publish(hedge_imu_fusion_msg);
    } 
    
    if (hedge->rawDistances.updated)
     {uint8_t i;
     
		for(i=0;i<4;i++)
		{
		  getRawDistance(i);
		  if (beacon_raw_distance_msg.address_beacon != 0)
		  {
			  ROS_INFO("Raw distance: %02d ==> %02d,  Distance= %.3f ", 	
				(int) beacon_raw_distance_msg.address_hedge,
				(int) beacon_raw_distance_msg.address_beacon,
				(float) beacon_raw_distance_msg.distance_m);
			  beacon_distance_publisher.publish(beacon_raw_distance_msg);
		  }	
		} 
		hedge->rawDistances.updated= false;
	 }
	 
	if (hedgeTelemetryUpdateCheck())
	 {
		 ROS_INFO("Vbat= %.3f V, RSSI= %02d ", 	
				(float) hedge_telemetry_msg.battery_voltage,
				(int) hedge_telemetry_msg.rssi_dbm);
		 hedge_telemetry_publisher.publish(hedge_telemetry_msg);
	 } 
	 
	if (hedgeQualityUpdateCheck())
	 {
		 ROS_INFO("Quality: Address= %d,  Quality= %02d %% ", 	
				(int) hedge_quality_msg.address,
				(int) hedge_quality_msg.quality_percents);
		 hedge_quality_publisher.publish(hedge_quality_msg);
	 }
	 
	if (marvelmindWaypointUpdateCheck())
	 {
		 int n= marvelmind_waypoint_msg.item_index+1;
		 ROS_INFO("Waypoint %03d/%03d: Type= %03d,  Param1= %05d, Param2= %05d, Param3= %05d ", 	
				(int) n,
				(int) marvelmind_waypoint_msg.total_items, marvelmind_waypoint_msg.movement_type,
				marvelmind_waypoint_msg.param1, marvelmind_waypoint_msg.param2, marvelmind_waypoint_msg.param3);
		 marvelmind_waypoint_publisher.publish(marvelmind_waypoint_msg);
	 }

    ros::spinOnce();

    loop_rate.sleep();
  }

  // Exit
  if (hedge != NULL) 
    {
      stopMarvelmindHedge (hedge);
      destroyMarvelmindHedge (hedge);
    }
    
   sem_close(sem);

  return 0;
}
