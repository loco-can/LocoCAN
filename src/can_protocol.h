// =====================================
//
// CAN-communication
//
// =====================================

#define CAN_BUS_SPEED 500E3

// MASK: bit=0 > bit of packetId is ignored
// MASK: bit=1 > bit is compared
#define CAN_ID_MASK 0x770

// high priority values
#define CAN_ID_CURRENT 0x100
#define CAN_ID_MOTOR_CURRENT 0x110
#define CAN_ID_BATT_CURRENT 0x120
#define CAN_ID_LIGHT_CURRENT 0x130

// mid priority values 
#define CAN_ID_SPEED 0x200
#define CAN_ID_DIR 0x210
#define CAN_ID_SIGNAL 0x220
#define CAN_ID_TACHO 0x230

// low priority values
#define CAN_ID_VOLTAGE 0x300 // main supply voltage
#define CAN_ID_MOTOR_VOLTAGE 0x310
#define CAN_ID_BATT_VOLTAGE 0x320
#define CAN_ID_BATT_1_VOLTAGE 0x321
#define CAN_ID_BATT_2_VOLTAGE 0x322

// command values
#define CAN_ID_DRIVE 0x400
#define CAN_ID_LIGHT 0x410
#define CAN_ID_SWITCH 0x420

// status values
#define CAN_ID_DRIVE_STATUS 0x500

// message heartbeat
#define CAN_ID_MODULE_HEARTBEAT 0x510 // heartbeat sent from modules

#define MODULE_HEARTBEAT_TIMEOUT 1000

// heardbeat values
// The heartbeat is sent from the active controller and used
// to monitor the connection between controller and motor modules.
// The train end latern signals are registered in the controller
// when starting to drive. A change in the latern signal list while
// driving leads to a emergency stop (switch off the heartbeat signal)
#define CAN_ID_DRIVE_HEARTBEAT 0x010
#define CAN_ID_TRAINEND_HEARTBEAT 0x020 // sent from train end laterns


// =====================================
// 
// controller message
// 
// =====================================

/*
 * The controller message are sent to all vehicles, when it is activated (mains = on).
 */

/*
 * 8 byte package sent from the controller
 *
 * byte 0:   7      6      5      4      3      2      1      0
 *         error  aux-1   aux  horn-h  horn-l  dir   drive  mains
 * byte 1:   7      6      5      4      3      2      1      0
 *
 *
 * paired: the controller is paired (only set, if paired motor is present)
 *
 * DRIVE VALUE: 10-bit value of drive voltage
 * byte 2: drive bit 8-9
 * byte 3: drive bit 0-7
 *
 * POWER VALUE: 10-bit value of drive max power
 * byte 4: power bit 8-9
 * byte 5: power bit 0-7
 *
 * BREAK VALUE: 10-bit value of break intensity
 * byte 6: break bit 8-9
 * byte 7: break bit 0-7
 */


// =====================================
//
// SIGNAL DATA?????
//
// =====================================

/*
 * 1 byte package
 *
 * SIGNAL STATUS
 * byte 0:   7      6      5      4      3      2      1      0
 *                                             bell   low   high
 *
 * high:    hight frequence signal
 * low:     low frequence signal
 * bell:    signal bell
 */

// =====================================
// 
// vehicle messages
// 
// =====================================

/*
 * The vehicle message are sent from each vehicle when a mains on is received.
 */

/*
 * 1 byte package sent from the vehicle
 *
 * VEHICLE STATUS
 * byte 0:   7      6      5      4      3      2      1      0
 *         error  ready  moving         revs   dir   drive  mains
 *
 * error:   motor drive reports an error
 * ready:   ready to drive
 * moving:  loco is moving
 *
 * reverse: if true, reverse dir signal
 * dir:     drive direction (0=forward, 1=reverse)
 * drive:   drive is activated
 * mains:   main switch state
 */


// =====================================
//
// LIGHT DATA
//
// =====================================

/*
 * 1 byte package
 *
 * LIGHT_STATUS
 * byte 0:   7      6      5      4      3      2      1      0
 *          main  train   cab   instr   back    high   low   posit
 *
 * main:    main light switch
 * train:   train lights
 * cab:     cabine light
 * instr:   instrument light
 * back:    back light
 * high:    bright spot light
 * low:     low spot light
 * posit:   position light
 */


// =====================================
// 
// setup
// 
// =====================================

// the setup command is used to send and receive module setup data
//		0x7FF		request setup info from module (0 byte message length)
//					the identifier uuid is ignored and the info is sent any way

//		0x7nn		returns info packages (nn = data id)
//					+ 8 bytes text description
//					adding the own uuid in the extended identifier
//					so no other module can interpret it as a write command

// write commands are sent with the target uuid in the first two bytes
//		0x600		set module name

// 		0x6nn		set data
//					nn => data id + max 8 bytes of data

// message  bytes 0, 1 	=> UUID of module
//			byte 2		=> message id
//				0xFF = module name
//			bytes 3-7 	=> data

#define CAN_REQUEST_MASK 0x7FF
#define CAN_ID_REQUEST 0x7FF

#define CAN_REPLY_MASK 0x7FF
#define CAN_ID_REPLY 0x780

#define CAN_SETUP_MASK 0x700
#define CAN_ID_SETUP 0x700


#define CAN_NAME_MAX_SIZE 5
#define CAN_VALUE_MAX_SIZE 6