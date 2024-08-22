/*
 Name:		GroundControl.ino
 Created:	2/6/2023 10:04:16 AM
 Author:	Imami Joel Betofe
*/

// the setup function runs once when you press reset or power the board
#include <mavlink.h>
#include <serial_port.h>

int main()
{
    // Connect to the serial port of the drone
    serial_port_t* serial_port = serial_port_open("/dev/ttyUSB0", 57600, MAVlink);

    // Wait for the heartbeat message from the drone to confirm the connection
    mavlink_message_t message;
    while (serial_port_read_message(serial_port, &message) != MAVLINK_MSG_ID_HEARTBEAT)
    {
        printf("Waiting for heartbeat from drone...\n");
        usleep(1000000); // sleep for 1 second
    }

    // Send a command to set the drone's mode to guided
    uint8_t target_system = message.sysid;
    uint8_t target_component = message.compid;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = MAV_MODE_GUIDED;
    mavlink_msg_set_mode_pack(1, 200, &message, target_system, base_mode, custom_mode);
    serial_port_write_message(serial_port, &message);

    // Send a command to arm the drone's motors
    mavlink_msg_command_long_pack(1, 200, &message, target_system, target_component,
        MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
    serial_port_write_message(serial_port, &message);

    // Send a command to take off to a specified altitude
    float altitude = 5.0;
    mavlink_msg_command_long_pack(1, 200, &message, target_system, target_component,
        MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude, 0);
    serial_port_write_message(serial_port, &message);

    // Wait for the drone to reach the desired altitude
    mavlink_message_t altitude_message;
    while (1)
    {
        if (serial_port_read_message(serial_port, &altitude_message) == MAVLINK_MSG_ID_VFR_HUD)
        {
            float current_altitude = mavlink_msg_vfr_hud_get_alt(&altitude_message);
            if (current_altitude >= altitude)
                break;
            printf("Altitude: %f\n", current_altitude);
        }
        usleep(500000); // sleep for 0.5 seconds
    }

    // Send a command to fly to a specific location
    float latitude = 37.788022;
    float longitude = -122.399797;
    mavlink_msg_mission_item_pack(1, 200, &message, target_system, target_component,
        0, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD

