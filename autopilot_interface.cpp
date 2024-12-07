/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * mavlink_set_position_target_local_ned_t 構造体を変更します。
 * ローカル NED フレーム内のターゲット XYZ 位置を m 単位で指定します。
 * 
 */
void
set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.x   = x;
	sp.y   = y;
	sp.z   = z;

	printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);

}




/*
 * Set target local ned velocity
 *
 * mavlink_set_position_target_local_ned_t 構造体を変更します。
 * ローカル NED フレームの速度をメートル毎秒で指定します。
 * 
 */
void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.vx  = vx;
	sp.vy  = vy;
	sp.vz  = vz;

	printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}

/*
 * Set target local ned acceleration
 *
 * mavlink_set_position_target_local_ned_t 構造体を変更します。
 * ローカル NED フレームの加速度をメートル毎秒2乗で指定します。
 * 
 */
void
set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{

	// NOT IMPLEMENTED
	fprintf(stderr,"set_acceleration doesn't work yet \n");
	throw 1;


	sp.type_mask =
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     ;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	sp.afx  = ax;
	sp.afy  = ay;
	sp.afz  = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * mavlink_set_position_target_local_ned_t 構造体をターゲットヨーで変更します。
 * ラジアン単位で設定します。
 * 
 */
void
set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;

	sp.yaw  = yaw;

	printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);

}

/*
 * Set target local ned yaw rate
 *
 * mavlink_set_position_target_local_ned_t 構造体を変更します。
 * ラジアン/秒で設定します。
 * 
 */
void
set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask &=
		MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;

	sp.yaw_rate  = yaw_rate;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Generic_Port *port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;
}


// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;
	printf("READ MESSAGE\n");

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				case MAVLINK_MSG_ID_GPS_RAW_INT:
				{
					//printf("MAVLINK_MSG_ID_GPS_RAW_INT\n");
					mavlink_msg_gps_raw_int_decode(&message, &(current_messages.gps_raw_int));
					current_messages.time_stamps.gps_raw_int = get_time_usec();
					this_timestamps.gps_raw_int = current_messages.time_stamps.gps_raw_int;
					break;
				}

				case MAVLINK_MSG_ID_COMMAND_ACK:
				{
					//printf("MAVLINK_MSG_ID_COMMAND_ACK\n");
					mavlink_msg_command_ack_decode(&message, &(current_messages.command_ack));
					current_messages.time_stamps.command_ack = get_time_usec();
					this_timestamps.command_ack = current_messages.time_stamps.command_ack;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
				this_timestamps.battery_status             &&
				this_timestamps.radio_status               &&
				this_timestamps.local_position_ned         &&
				this_timestamps.global_position_int        &&
				this_timestamps.position_target_local_ned  &&
				this_timestamps.position_target_global_int &&
				this_timestamps.highres_imu                &&
				this_timestamps.attitude                   &&
				this_timestamps.gps_raw_int                &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}




//追加

//追加
// ------------------------------------------------------------------------------
//   Write Message 
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
send_input_global_position_int_message()
{

	uint8_t target_system = system_id;
	uint8_t target_component = autopilot_id;
	// mavlink_gps_input_t gps_input;
	mavlink_global_position_int_t global_position_int;

	global_position_int.lat = 351523041;
	global_position_int.lon = 1369686962;
	global_position_int.alt = 0;
	global_position_int.relative_alt = 0;
	global_position_int.vx = 0;
	global_position_int.vy = 0;
	global_position_int.vz = 0;
	global_position_int.hdg = UINT16_MAX;


	mavlink_message_t message;

	mavlink_msg_global_position_int_encode(target_system, target_component, &message, &global_position_int);
	// mavlink_msg_gps_raw_int_encode(target_system, target_component, &message, &gps_input);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GPS_INPUT_message \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

int
Autopilot_Interface::
send_input_gps_raw_int_message( uint64_t time_usec )
{

	uint8_t target_system = system_id;
	uint8_t target_component = autopilot_id;
	// mavlink_gps_input_t gps_input;
	mavlink_gps_raw_int_t gps_input;

	gps_input.time_usec = time_usec; 
	gps_input.lat = 351523041;
	gps_input.lon = 1369686962;
	gps_input.alt = 0; 
	gps_input.eph = UINT8_MAX; //not used
	gps_input.epv = UINT8_MAX; //not used
	gps_input.vel = UINT8_MAX; //not used
	gps_input.cog = UINT16_MAX; //not used
	gps_input.fix_type = 0;  //gpsの動作状況の設定 0: 3D fix, 1: 2D fix, 2: 1D fix, 3: no fix
	gps_input.satellites_visible = UINT8_MAX; //not used
	//extension field
	gps_input.alt_ellipsoid = UINT16_MAX; //not used
	gps_input.h_acc = UINT16_MAX; //not used
	gps_input.v_acc = UINT16_MAX; //not used
	gps_input.hdg_acc = UINT16_MAX; //not used
	gps_input.yaw = UINT16_MAX; //not used

	mavlink_message_t message;

	mavlink_msg_gps_raw_int_encode(target_system, target_component, &message, &gps_input);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GPS_INPUT_message \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

int
Autopilot_Interface::
send_input_gps_message( uint64_t time_usec )
{

	uint8_t target_system = system_id;
	uint8_t target_component = autopilot_id;
	mavlink_gps_input_t gps_input;

	gps_input.time_usec = time_usec; 
	gps_input.gps_id = 2; 
	gps_input.ignore_flags = GPS_INPUT_IGNORE_FLAG_ALT| GPS_INPUT_IGNORE_FLAG_HDOP | GPS_INPUT_IGNORE_FLAG_VDOP | GPS_INPUT_IGNORE_FLAG_VEL_HORIZ | GPS_INPUT_IGNORE_FLAG_VEL_VERT | GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY | GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY | GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
	gps_input.time_week_ms = 0; 
	gps_input.time_week = 0; 
	gps_input.fix_type = 3;   //gpsの動作状況の設定 0-1: no fix, 2: 2D fix, 3: 3D fix

	gps_input.lat = 351523041;
	gps_input.lon = 1369686962;

	gps_input.alt = 0; //not used
	gps_input.hdop = 0; ///not used
	gps_input.vdop = 0; //not used
	gps_input.vn = 0; //not used
	gps_input.ve = 0; //not used
	gps_input.vd = 0; //not used
	gps_input.speed_accuracy = 0; //not used
	gps_input.horiz_accuracy = 0; //not used
	gps_input.vert_accuracy = 0; //not used

	gps_input.satellites_visible = 1; 
	gps_input.yaw = 0; //not used



	int32_t is_healthy = 0;

	mavlink_message_t message;

	mavlink_msg_gps_input_pack(target_system,  target_component, &message, gps_input.time_usec,
							   gps_input.gps_id, gps_input.ignore_flags, gps_input.time_week_ms, 
							   gps_input.time_week,  gps_input.fix_type, gps_input.lat, gps_input.lon, gps_input.alt, gps_input.hdop, gps_input.vdop,
							    gps_input.vn, gps_input.ve, gps_input.vd, gps_input.speed_accuracy, gps_input.horiz_accuracy, gps_input.vert_accuracy, gps_input.satellites_visible, gps_input.yaw);
	
	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GPS_INPUT_message \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

int
Autopilot_Interface::
send_input_hil_gps_message( uint64_t time_usec )
{

	uint8_t target_system = system_id;
	uint8_t target_component = autopilot_id;
	mavlink_hil_gps_t gps_input;
	

	gps_input.time_usec = time_usec; 

	gps_input.lat = 351523041;
	gps_input.lon = 1369686962;

	gps_input.alt = 0; //not used
	
	gps_input.eph = UINT16_MAX; ///not used
	gps_input.epv = UINT16_MAX; //not used
	gps_input.vel = UINT16_MAX; //not used
	
	gps_input.vn = 0;
	gps_input.ve = 0;
	gps_input.vd = 0;
	
	gps_input.cog = UINT16_MAX; //not used

	gps_input.fix_type = 3;   //gpsの動作状況の設定 0-1: no fix, 2: 2D fix, 3: 3D fix
	gps_input.satellites_visible = 1; 
	gps_input.id = 0;
	gps_input.yaw = 0; //not used



	int32_t is_healthy = 0;

	mavlink_message_t message;


	mavlink_msg_hil_gps_encode(target_system, target_component, &message, &gps_input);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send GPS_INPUT_message \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}



// ------------------------------------------------------------------------------
//   Write Message MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES ( 520 )
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
autopilot_calibrate()
{
	printf("CALIBRATION\n");

	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES; 
	com.confirmation     = false;
	com.param1           = 1; //1: request autopilot version

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Message MAV_CMD_SET_MESSAGE_INTERVAL ( 511 )
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
set_message_interval( float msg_id, float interval_us )
{
	printf("SET_MESSAGE_INTERVAL\n");

	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_SET_MESSAGE_INTERVAL; 
	com.confirmation     = false;
	com.param1           = msg_id; //msg_id
	com.param2           = interval_us; //interval_us
	com.param7 		     = NAN; //0: response target

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MAV_CMD_SET_MESSAGE_INTERVAL \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Message MAV_CMD_NAV_TAKEOFF_LOCAL ( 24 )
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
takeoff_local( float asec_rate, float yaw, float x, float y, float z )
{
	printf("TAKEOFF_LOCAL: %f[m] \n", z);

	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_TAKEOFF_LOCAL; 
	com.confirmation     = false;
	com.param3           = asec_rate; //asec rate
	com.param4           = yaw; //yaw
	com.param5 		     = x; //x
	com.param6 		     = y; //y
	com.param7 		     = z; //z	(z軸は鉛直下向き)

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MAV_CMD_NAV_TAKEOFF_LOCAL \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Message MAV_CMD_NAV_LAND_LOCAL ( 23 )
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
land_local(float asec_rate, float yaw, float x, float y, float z)
{
	printf("LAND_LOCAL\n");

	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_LAND_LOCAL; 
	com.confirmation     = false;
	com.param3           = asec_rate; //asec rate
	com.param4           = yaw; //yaw
	com.param5 		     = x; //x
	com.param6 		     = y; //y
	com.param7 		     = z; //z	(z軸は鉛直下向き)

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MAV_CMD_NAV_LAND_LOCAL \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}




// ------------------------------------------------------------------------------
//   Write Message MAV_CMD_NAV_TAKEOFF ( 22 )
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
takeoff( float pitch, float yaw, float latitude, float longitude, float altitude )
{
	printf("TAKEOFF: %f[m] \n", altitude);

	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_TAKEOFF; 
	com.confirmation     = false;
	com.param1           = pitch; //pitch
	com.param4           = yaw; //yaw
	com.param5 		     = latitude; //latitude
	com.param6 		     = longitude; //longitude
	com.param7 		     = altitude; //altitude
	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MAV_CMD_NAV_TAKEOFF \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Message MAV_CMD_NAV_LAND ( 21 )
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
land( int land_mode, float yaw, float latitude, float longitude, float altitude )
{
	printf("LAND\n");

	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_LAND; 
	com.confirmation     = false;
	com.param1           = NAN; //abort_alt	
	com.param2           = land_mode; //land_mode
	com.param4           = yaw; //yaw
	com.param5 		     = latitude; //latitude
	com.param6 		     = longitude; //longitude
	com.param7 		     = altitude; //altitude

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send MAV_CMD_NAV_LAND \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp;
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		sp = current_setpoint.data;
	}

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
enable_offboard_control()
{
	// Should only send this command once
	if ( control_status == false )
	{
		printf("ENABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to go off-board
		int success = toggle_offboard_control( true );

		// Check the command was written
		if ( success )
			control_status = true;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if not offboard_status

}


// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
disable_offboard_control()
{

	// Should only send this command once
	if ( control_status == true )
	{
		printf("DISABLE OFFBOARD MODE\n");

		// ----------------------------------------------------------------------
		//   TOGGLE OFF-BOARD MODE
		// ----------------------------------------------------------------------

		// Sends the command to stop off-board
		int success = toggle_offboard_control( false );

		// Check the command was written
		if ( success )
			control_status = false;
		else
		{
			fprintf(stderr,"Error: off-board mode not set, could not write message\n");
			//throw EXIT_FAILURE;
		}

		printf("\n");

	} // end: if offboard_status

}

// ------------------------------------------------------------------------------
//   Arm
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
arm_disarm( bool flag )
{
	if(flag)
	{
		printf("ARM ROTORS\n");
	}
	else
	{
		printf("DISARM ROTORS\n");
	}

	// Prepare command for off-board mode
	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_COMPONENT_ARM_DISARM;
	com.confirmation     = false;
	com.param1           = (float) flag;
	com.param2           = NAN; // force

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
toggle_offboard_control( bool flag )
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = {
    NAN, // param1
    NAN, // param2
    NAN, // param3
    NAN, // param4
    NAN, // param5
    NAN, // param6
    NAN, // param7
    0,   // command
    0,   // target_system
    0,   // target_component
    0    // confirmation
};
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
	com.confirmation     = false;
	com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);

	// Done!
	return len;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------

	if ( !port->is_running() ) // PORT_OPEN
	{
		fprintf(stderr,"ERROR: port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// これは心臓の鼓動から来るもので、理論的には、私たちが直接接続している自動操縦装置からしか来ないはずだ。 
	// 複数の車両がある場合、このようなidの発見は期待できません。
	// その場合は、手動で ID を設定する。

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------
	
	//フライトコントローラにGPSをつないでいるかどうか
	bool FC_GPS = false;
	
	if(FC_GPS)
	{
		// Wait for initial position ned
		while ( not ( current_messages.time_stamps.local_position_ned &&
					current_messages.time_stamps.attitude            )  )
		{
			if ( time_to_exit )
				return;
			usleep(500000);
		}

		// copy initial position ned
		Mavlink_Messages local_data = current_messages;
		initial_position.x        = local_data.local_position_ned.x;
		initial_position.y        = local_data.local_position_ned.y;
		initial_position.z        = local_data.local_position_ned.z;
		initial_position.vx       = local_data.local_position_ned.vx;
		initial_position.vy       = local_data.local_position_ned.vy;
		initial_position.vz       = local_data.local_position_ned.vz;
		initial_position.yaw      = local_data.attitude.yaw;
		initial_position.yaw_rate = local_data.attitude.yawspeed;

		printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
		printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
		printf("\n");
	}

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{

	disable_offboard_control();

	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		current_setpoint.data = sp;
	}

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



