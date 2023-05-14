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

// vehicle status values
#define CAN_ID_VEHICLE_STATUS 0x500

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
// controller message CAN_ID_DRIVE
// 
// =====================================

/*
 * The controller message are sent to all vehicles, when it is activated (mains = on).
 *
 * If a multi traktion is recognised by the controller when activated, it 15 of byte 2
 * is set to 1 and the uuid of the main locomotive is sent in bytes 2, 4 and 6.
 * All other locomotives are running in slave mode with no lights or pantographs.
 */

/*
 * 8 byte package sent from the controller
 * the buttons for horn-h and horn-l are used for up and down signals
 *
 * byte 0:   7      6      5      4      3      2      1      0
 *         error  aux-1   aux    down   up     dir   drive  mains
 * byte 1:   7      6      5      4      3      2      1      0
 *                                                          
 *
 * DRIVE VALUE: 10-bit value of drive voltage
 * byte 2: bits 10-14 = multi traktion main UUID bits 11-15
 *         bit 16 = 1: multi traktion
 *         drive bit 8-9
 * byte 3: drive bit 0-7
 *
 * POWER VALUE: 10-bit value of drive max power
 * byte 4: bits 10-15 = multi traktion main UUID bits 6-10
 *         power bit 8-9
 * byte 5: power bit 0-7
 *
 * BREAK VALUE: 10-bit value of break intensity
 * byte 6: bits 10-15 = multi traktion main UUID bits 0-5
 *         break bit 8-9
 * byte 7: break bit 0-7
 */

// =====================================
// CONTROLLER
#define CONTROL_ERROR_FLAG 7

#define CONTROL_AUX_1_FLAG 6
#define CONTROL_AUX_FLAG 5

#define CONTROL_DOWN_FLAG 4
#define CONTROL_UP_FLAG 3

#define CONTROL_DIR_FLAG 2
#define CONTROL_DRIVE_FLAG 1
#define CONTROL_MAINS_FLAG 0


// =====================================
//
// SIGNAL DATA
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

// SIGNAL
#define SIGNAL_HIGH 0
#define SIGNAL_LOW 1
#define SIGNAL_BELL 2 // not yet impemented


// =====================================
// 
// vehicle messages
// 
// =====================================

/*
 * The vehicle message are sent from each vehicle when a mains on is received.
 */

/*
 * 3 byte package sent from the vehicle
 *
 * VEHICLE STATUS
 * byte 0:   7      6      5      4      3      2      1      0
 *         error  ready  moving         revs   dir   drive  mains
 * byte 1: MSB of UUID
 * byte 2: LSB of UUID
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
// VEHICLE

#define ERROR_FLAG 7
#define READY_FLAG 6
#define MOVING_FLAG 5

#define REVERSE 3

#define DIR_FLAG 2
#define CONTROL_FLAG 1
#define MAINS_FLAG 0


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

// LIGHT
#define LIGHT_MAIN 7
#define LIGHT_TRAIN 6
#define LIGHT_CAB 5
#define LIGHT_INSTR 4
#define LIGHT_BACK 3
#define LIGHT_HIGH 2
#define LIGHT_LOW 1
#define LIGHT_POSIT 0


// =====================================
// 
// loco setup
// 
// =====================================

/*
 * 3 byte package
 *
 * LOCO_SETUP
 * byte 0:   7      6      5      4      3      2      1      0
 *                                                  disable  status
 *
 * status:      loco setup status
                0 = select: the master loco is selected
                1 = direction: reverse the direction in the loco
 * disable:     0: loco is active
                1: loco is deactivated
                    sends mains = on, drive = on, ready = true, moving = false
 * byte 1:  LSB of the locomotive UUID to setup
 * byte 2:  MSB of the locomotive UUID to setup
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