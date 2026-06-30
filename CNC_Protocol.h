// CNC_Protocol.h
// ---------------
// This file defines the protocol used between the CNC machine and the host computer
// over UDP. It also defines the state flags used to report the status of the machine.
// This file is included by both the firmware and the host software.

#define CNC_PROTOCOL_VERSION              3

// Arbitrary non-privileged port
#define CNC_UDP_PORT                      50042

// How often the CNC sends a ping (POS) to the host when active or idle to
// keep the connection alive
#define CNC_ACTIVE_POS_TIMEOUT_MS         500
#define CNC_IDLE_POS_TIMEOUT_MS           1500

// How long for waiting for the motors to go idle after flushing the queue
#define MOTOR_IDLE_TIMEOUT_MS             10000
                                          
#define CNC_HEADER                        "CNC-"
#define CNC_HEADER_LEN                    4
                                          
#define CNC_CMD_HEADER                    "CMD" 
#define CNC_CMD_HEADER_LEN                3
//                                         Seq,Count
#define CNC_CMD_HEADER_PARAMS             "%lu,%lu"
//                                           x,  y,  z,  d,flags
#define CNC_CMD_PARAMS                    "%ld,%ld,%ld,%lu,%lx" 

#define CNC_CMD_CALIBRATE                 "CALIBRATE"
#define CNC_CMD_CALIBRATE_LEN             9

#define CNC_CMD_FLUSH                     "FLUSH"
#define CNC_CMD_FLUSH_LEN                 5

#define CNC_CMD_REBOOT                    "REBOOT"
#define CNC_CMD_REBOOT_LEN                6

#define CNC_MANUAL_HEADER                 "MANUAL"
#define CNC_MANUAL_HEADER_LEN             6
#define CNC_MANUAL_PARAMS                 "%d,%d,%d"
                                          
#define CNC_INFO_HEADER                   "INFO"
#define CNC_INFO_HEADER_LEN               4
//                                        Ver,Rx,Qs,Xs,Ys,Zs, Xm, Ym, Zm, XM, YM, ZM,
#define CNC_INFO_PARAMS                   "%d,%u,%u,%f,%f,%f,%ld,%ld,%ld,%ld,%ld,%ld" 
                                          
#define CNC_POS_HEADER                    "POS"
#define CNC_ACK_HEADER                    "ACK"
#define CNC_NAK_HEADER                    "NAK"
#define CNC_POS_ACK_NAK_HEADER_LEN        3
//                                         Seq,  x,  y,  z,Ste,inQ,Dbg
#define CNC_POS_ACK_NAK_PARAMS            "%lu,%ld,%ld,%ld,%lx,%u,%llx"

// Command flags
// -------------
#define CMD_FLAGS_CRC_MASK                0x000000FFL
#define CMD_FLAG_SPINDLE_ON               0x00000100L
#define CMD_FLAG_CALIBRATION              0x00000200L
#define CMD_CALIBRATION_COMPLETE          0x00000400L // Used by CNC firmware. Not for host.
#define CMD_FLAG_MANUAL_MOVE              0x00000800L
#define CMD_SLOW_START_ENABLED            0x00800000L // Used by CNC firmware. Not for host.
#define CMD_SLOW_START_MASK               0xFF000000L // Used by CNC firmware. Not for host.
#define SLOW_FACTOR_FLAG_SHIFT            24

#define CMD_FLAGS_EXCLUDED_FROM_CRC_MASK (CMD_FLAGS_CRC_MASK | CMD_SLOW_START_MASK | CMD_SLOW_START_ENABLED)

//  E R R O R S
//  -----------
#define CNC_STATE_MOTOR_CRC_ERROR         0x80000000L // The CRC of a movement command did not match the
                                                      // physical position. The machine is not where it's
                                                      // supposed to be. A recalibration is required.
#define CNC_STATE_NETWORK_CRC_ERROR       0x40000000L // The CRC in a UDP command message did not match 
                                                      // with the network position. The command was rejected.
#define CNC_STATE_HARD_ERROR              0x20000000L // A physical limit was triggered or the X axis got 
                                                      // too slanted. Motors are powered down.
                                                      // A recalibration is required.
#define CNC_STATE_LOGICAL_LIMIT_ERROR     0x10000000L // The end position of a movement went outside of the 
                                                      // machine limits.
#define CNC_STATE_CALIBRATION_FAILED      0x08000000L // The calibration process failed because the machine was
                                                      // not idle, went too far or triggered a physical limit
#define CNC_STATE_COMMUNICATION_ERROR     0x04000000L // A UDP message with the correct header was not formatted
                                                      // properly.
#define CNC_STATE_IDLE_TIMEOUT_ERROR      0x02000000L // Following a command to "FLUSH", one of the axis didn't
                                                      // return to idle 

// 1 more here                            0x01000000L // Reserved for future use

#define CNC_STATE_ERROR_MASK              0xFF000000L // ANY OF THOSE FLAG SET WILL MAKE THE MACHINE REJECT ANY
                                                      // NEW COMMANDS
#define CNC_STATE_RECOVERABLE_ERROR_MASK  (CNC_STATE_ERROR_MASK & ~CNC_STATE_NETWORK_CRC_ERROR)

//  W A R N I N G S
//  --------------- 
// 5 more here
#define CNC_STATE_HARD_LIMIT              0x00080000L // The hard limit switch is triggered. This is not an error
                                                      // but indicates that the machine is at one of its limits.
                                                      // The error state is if the motor controllers are disabled
                                                      // and the hard limit is still triggered.
#define CNC_STATE_CAL_ORIGIN_ERROR        0x00040000L // From a calibrated state the origin position was off by
                                                      // more than 3 steps
#define CNC_STATE_COMMAND_QUEUE_FULL      0x00020000L // The command queue is currently full
#define CNC_STATE_POS_SENSOR_XL           0x00010000L // The left side position sensor for the X axis is triggered
#define CNC_STATE_POS_SENSOR_XR           0x00008000L // Same for the right side X axis.
#define CNC_STATE_POS_SENSOR_ZL           0x00004000L // Same for the left side Z axis
#define CNC_STATE_POS_SENSOR_ZR           0x00002000L // Same for the right side Z axis.
#define CNC_STATE_POS_SENSOR_Y            0x00001000L // The Y axis position sensor is triggered
#define CNC_STATE_WARNING_MASK            0x00FFF000L

//  I N F O R M A T I O N
//  ---------------------
// 4 more here
#define CNC_STATE_Z_CALIBRATED            0x00000080L // The Z axis is calibrated
#define CNC_STATE_Y_CALIBRATED            0x00000040L // Same for Y axis
#define CNC_STATE_X_CALIBRATED            0x00000020L // Same for X axis
#define CNC_STATE_ALL_CALIBRATED          ( CNC_STATE_X_CALIBRATED | CNC_STATE_Y_CALIBRATED | CNC_STATE_Z_CALIBRATED )
#define CNC_STATE_CALIBRATING             0x00000010L // The machine is performing its axis alibration
#define CNC_STATE_MANUAL_MODE             0x00000008L // The machine is currently operating in manual mode
#define CNC_STATE_IDLE                    0x00000004L // The maxhine is idle no motor is moving (nor dwelling)
#define CNC_STATE_LITTLE_ENDIAN           0x00000002L // The SOC running the machine is little endian (LSB first,
                                                      // like Intel proc)
#define CNC_STATE_CONNECTED               0x00000001L // The machine is connected. Any status from the machine must
                                                      // have this bit set
#define CNC_STATE_INFORMATION_MASK        0x00000FFFL

#define CNC_STATE_ALL_MASK                (CNC_STATE_INFORMATION_MASK|CNC_STATE_WARNING_MASK|CNC_STATE_ERROR_MASK)

#if CNC_STATE_ERROR_MASK & CNC_STATE_WARNING_MASK & CNC_STATE_INFORMATION_MASK
  #error "State flags overlap"
#endif
