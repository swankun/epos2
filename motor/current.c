#include "current.h"
#include "motor.h"
#include "epos.h"

#include "socketcan/printd.h"
#include "canopen/canopen.h"



// Non-exported function
static int _current_transmit_PDO(int16_t milliamps, uint16_t nodeid) {
	int err = 0;
	Socketcan_t target[2] = {
		{2, Switch_On_And_Enable_Operation},
		{2, milliamps},
	};
	err = PDO_send(motor_pdo_fd, PDO_RX1_ID + nodeid, 2, target);
	return err;
}


int current_set(int16_t milliamps) {
	return _current_transmit_PDO(milliamps, MOTOR_EPOS_NODEID);
}

int current_halt(void) {
    return _current_transmit_PDO(0, MOTOR_EPOS_NODEID);
}

int current_read(int32_t* pos, int32_t* vel, int16_t* curr, int timeout) {

	int err;
	int status = 0;

	my_can_frame f;
	err = PDO_read(motor_pdo_fd, &f, timeout);

	if(err != 0) {
		return err;
	}

	uint32_t enc, rpm;
    uint16_t milliamps;
	switch(f.id) {
		case(PDO_TX1_ID + MOTOR_EPOS_NODEID):
			status = (f.data[0]<<0) | (f.data[1]<<8);
            milliamps = ((uint16_t)f.data[2]<<0) | ((uint16_t)f.data[3]<<8);
            // printd(LOG_WARN, "0x181: 0%x ", f.data[0]);
            // printd(LOG_WARN, "0%x ", f.data[1]);
            // printd(LOG_WARN, "0%x ", f.data[2]);
            // printd(LOG_WARN, "0%x ", f.data[3]);
            // printd(LOG_WARN, "0%x \n", f.data[4]);
            *curr = milliamps;
			break;
		case(PDO_TX2_ID + MOTOR_EPOS_NODEID):
			status = (f.data[0]<<0) | (f.data[1]<<8);
			break;
		case(PDO_TX3_ID + MOTOR_EPOS_NODEID):
            // printd(LOG_WARN, "0x381: 0%x ", f.data[0]);
            // printd(LOG_WARN, "0%x ", f.data[1]);
            // printd(LOG_WARN, "0%x ", f.data[2]);
            // printd(LOG_WARN, "0%x ", f.data[3]);
            // printd(LOG_WARN, "0%x ", f.data[4]);
            // printd(LOG_WARN, "0%x ", f.data[5]);
            // printd(LOG_WARN, "0%x ", f.data[6]);
            // printd(LOG_WARN, "0%x \n", f.data[7]);
            // status = (f.data[0]<<0) | (f.data[1]<<8);
			enc = ((uint32_t)f.data[2]<<0) | ((uint32_t)f.data[3]<<8) | ((uint32_t)f.data[4]<<16) | ((uint32_t)f.data[5]<<24);
            // printd(LOG_WARN, "enc = %d \n", enc);
			*pos = enc;
			break;
		case(PDO_TX4_ID + MOTOR_EPOS_NODEID):
			status = (f.data[0]<<0) | (f.data[1]<<8);
			rpm = ((uint32_t)f.data[2]<<0) | ((uint32_t)f.data[3]<<8) | ((uint32_t)f.data[4]<<16) | ((uint32_t)f.data[5]<<24);
			*vel = motor_rpm_to_mmsec(rpm);
			break;
		default:
			printd(LOG_WARN, "motor/current.c recived unkown PDO pkg 0x%x\n", f.id);
			break;
	}
    // printd(LOG_WARN, "Epos 0x%x status=0x%x\n", f.id, status);
	if(status & 0x08) {
		// The epos reported an error
		printd(LOG_ERROR, "The epos %d reported an error! status=0x%x\n", f.id-PDO_TX1_ID, status);
		return -11;  // TODO: uniq error codes
	} else if(status & 0x80) {
		// The epos reported an warning
		printd(LOG_ERROR, "The epos %d reported an warning! status=0x%x\n", f.id-PDO_TX1_ID, status);
		return -12;
	}

	return 0;
}
