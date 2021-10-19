#include "motor.h"
#include "epos.h"

#include "socketcan/socketcan.h"
#include "socketcan/printd.h"
#include "canopen/canopen.h"


#include <math.h>
#include <stdlib.h>
#include <inttypes.h>



int motor_pdo_fd = -1; //!< Process CAN-connection.
int motor_cfg_fd = -1; //!< Configuration CAN-connection.


static int motor_config_node(uint16_t node) {
	int err = 0;
	int num_PDOs;

	// Set Configuration parameters
	err |= epos_Maximal_Profile_Velocity(node, 100);
	if( err != 0 ) {
		printd(LOG_FATAL, "Motor: error configuring node %d, no power?\n", node);
		return err;
	}
	err |= epos_Quickstop_Deceleration(node, 10000);
	err |= epos_Profile_Acceleration(node, 10000);
	err |= epos_Profile_Deceleration(node, 10000);
	err |= epos_Motion_Profile_Type(node, trapezodial_profile);
	err |= epos_Miscellaneous_Configuration(node, Meassure_main_position_sensors_motor_speed_exacting_by_detecting_encoder_pulse_time);
	if(err != 0) {
		printd(LOG_FATAL, "Motor: error configuring node %d.\n", node);
		return err;
	}


	// PDO cob id's
	err |= epos_Receive_PDO_n_Parameter(node, 1, PDO_RX1_ID + node);
	err |= epos_Receive_PDO_n_Parameter(node, 2, PDO_RX2_ID + node);
	err |= epos_Receive_PDO_n_Parameter(node, 3, PDO_RX3_ID + node);
	err |= epos_Receive_PDO_n_Parameter(node, 4, PDO_RX4_ID + node);
	err |= epos_Transmit_PDO_n_Parameter(node, 1, PDO_TX1_ID + node);
	err |= epos_Transmit_PDO_n_Parameter(node, 2, PDO_TX2_ID + node);
	err |= epos_Transmit_PDO_n_Parameter(node, 3, PDO_TX3_ID + node);
	err |= epos_Transmit_PDO_n_Parameter(node, 4, PDO_TX4_ID + node);


	/*** Communication, from pc to epos ***/

	// PDO RX1
	num_PDOs = 2;
	Epos_pdo_mapping RxPDO1[] = {
		{0x6040, 0x00, 16},   // Controlword
		{0x2030, 0x00, 16}   // Current Command
	};
	err |= epos_Receive_PDO_n_Mapping(node, 1, num_PDOs, RxPDO1);

	// PDO RX2
	num_PDOs = 2;
	Epos_pdo_mapping RxPDO2[] = {
		{0x6040, 0x00, 16},  // Controlword
		{0x6060, 0x00, 8}   // Mode of Operation
	};
	err |= epos_Receive_PDO_n_Mapping(node, 2, num_PDOs, RxPDO2);

	// PDO RX3
	num_PDOs = 2;
	Epos_pdo_mapping RxPDO3[] = {
		{0x6040, 0x00, 16},  // Controlword
		{0x607A, 0x00, 32}   // Target Position
	};
	err |= epos_Receive_PDO_n_Mapping(node, 3, num_PDOs, RxPDO3);

	// PDO RX4
	num_PDOs = 2;
	Epos_pdo_mapping RxPDO4[] = {
		{0x6040, 0x00, 16},  // Controlword
		{0x60FF, 0x00, 32}   // Target Velocity
	};
	err |= epos_Receive_PDO_n_Mapping(node, 4, num_PDOs, RxPDO4);

	// Disable the rest
	// err |= epos_Receive_PDO_n_Mapping(node, 3, 0, NULL);
	// err |= epos_Receive_PDO_n_Mapping(node, 4, 0, NULL);


	/*** Communication, from epos to pc ***/

	// PDO TX1
	num_PDOs = 2;
	Epos_pdo_mapping TxPDO1[] = {
		{0x6041, 0x00, 16},  // Statusword
		{0x6078, 0x00, 16}   // Current Actual Value
	};
	err |= epos_Transmit_PDO_n_Mapping(node, 1, num_PDOs, TxPDO1);

	// PDO TX2
	num_PDOs = 3;
	Epos_pdo_mapping TxPDO2[] = {
		{0x6041, 0x00, 16},  // Statusword
		{0x6060, 0x00, 8},   // Mode of Operation
		{0x6061, 0x00, 8}    // Mode of Operation Display
	};
	err |= epos_Transmit_PDO_n_Mapping(node, 2, num_PDOs, TxPDO2);

	// PDO TX3
	num_PDOs = 2;
	Epos_pdo_mapping TxPDO3[] = {
		{0x6041, 0x00, 16},  // Statusword
		{0x6064, 0x00, 32}   // Position Actual Value
	};
	err |= epos_Transmit_PDO_n_Mapping(node, 3, num_PDOs, TxPDO3);

	// PDO TX4
	num_PDOs = 2;
	Epos_pdo_mapping TxPDO4[] = {
		{0x6041, 0x00, 16},  // Statusword
		{0x606C, 0x00, 32}   // Velocity Actual Value
	};
	err |= epos_Transmit_PDO_n_Mapping(node, 4, num_PDOs, TxPDO4);

	// Disable the rest
	// err |= epos_Transmit_PDO_n_Mapping(node, 3, 0, NULL);
	// err |= epos_Transmit_PDO_n_Mapping(node, 4, 0, NULL);


	return err;
}


int motor_init(void) {
	int err = 0;

	// Open two connections to the CAN-network
	uint16_t pdo_masks[4] = {COB_MASK, COB_MASK, COB_MASK, COB_MASK};
	uint16_t pdo_filters[4] = {
		PDO_TX1_ID + MOTOR_EPOS_NODEID,
		PDO_TX2_ID + MOTOR_EPOS_NODEID,
		PDO_TX3_ID + MOTOR_EPOS_NODEID,
		PDO_TX4_ID + MOTOR_EPOS_NODEID
	};
	motor_pdo_fd = socketcan_open(pdo_filters, pdo_masks, 4);

	uint16_t cfg_masks[3] = {COB_MASK, COB_MASK, COB_MASK};
	uint16_t cfg_filters[3] = {
		0x00,
		NMT_TX + MOTOR_EPOS_NODEID,
		SDO_TX + MOTOR_EPOS_NODEID,
	};
	motor_cfg_fd = socketcan_open(cfg_filters, cfg_masks, 5);

	// Check that we connected OK
	if (motor_pdo_fd == -1 || motor_cfg_fd == -1) {
		return MOTOR_ERROR;
	}

	// Configure each node
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Enter_PreOperational);
	if (err != 0) {
		return MOTOR_ERROR;
	}

	err |= motor_config_node(MOTOR_EPOS_NODEID);
	if (err != 0) {
		return MOTOR_ERROR;
	}

	// Set the default mode
	motor_setmode(Motor_mode_Current);
	if (err != 0) {
		return MOTOR_ERROR;
	}

	return 0;
}


void motor_close(void) {
	socketcan_close(motor_pdo_fd);
	socketcan_close(motor_cfg_fd);
}


int motor_enable(void) {
	int err = 0;
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Enter_PreOperational);
	err |= epos_Controlword(MOTOR_EPOS_NODEID, Shutdown); // switch_on_disabled -> switch_on_enabled
	err |= epos_Controlword(MOTOR_EPOS_NODEID, Switch_On_And_Enable_Operation);

	// Open PDO-communication
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Start_Node);

	return err;
}


int motor_disable(void) {
	int err = 0;

	// Stop PDO-communication
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Enter_PreOperational);
	err |= epos_Controlword(MOTOR_EPOS_NODEID, Disable_Voltage);
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Stop_Node);

	return err;
}


int motor_halt(void) {
	int err = 0;

	// Stop PDO-communication
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Enter_PreOperational);
	err |= epos_Controlword(MOTOR_EPOS_NODEID, Quickstop);
	err |= NMT_change_state(motor_cfg_fd, CANOPEN_BROADCAST_ID, NMT_Stop_Node);

	return err;
}


int motor_setmode(enum Motor_mode mode) {
	int err = 0;
	err |= epos_Modes_of_Operation(MOTOR_EPOS_NODEID, (enum Epos_mode)mode);
	return err;
}



/********* Utils: *********/
int motor_mmsec_to_rpm(int mm_per_sec) {
	const double wheel_circumference = (2.0*MOTOR_WHEEL_RADIUS)*M_PI;
	const double mm_per_rot = wheel_circumference/MOTOR_GEAR_RATIO;
	int mm_per_min = 60*mm_per_sec;
	int rpm = mm_per_min/mm_per_rot;  // [mm/min]/[mm/1] = [1/min]
	return rpm;
}

int motor_rpm_to_mmsec(int rpm) {
	const double wheel_circumference = (2.0*MOTOR_WHEEL_RADIUS)*M_PI;
	const double mm_per_rot = wheel_circumference/MOTOR_GEAR_RATIO;
	int mm_per_min = rpm*mm_per_rot;
	return mm_per_min/60.0;
}

int motor_enc_to_mm(int enc) {
	const double wheel_circumference = (2.0*MOTOR_WHEEL_RADIUS)*M_PI;
	const double mm_per_rot = wheel_circumference/MOTOR_GEAR_RATIO;
	int mm = enc*mm_per_rot;
	return mm;
}
