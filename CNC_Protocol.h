
#define CNC_PROTOCOL_VERSION              3

// Arbitrary non-privileged port
#define CNC_UDP_PORT                      50042

// How often the CNC sends a ping (POS) to the host when idle to
// keep the connection alive
#define CNC_IDLE_POS_TIMEOUT_MS           1000
                                          
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
                                          
#define CNC_INFO_HEADER                   "INFO"
#define CNC_INFO_HEADER_LEN               4
//                                        Ver,Rx,Qsz,X, Y, Z
#define CNC_INFO_PARAMS                   "%d,%u,%u,%f,%f,%f" 
                                          
#define CNC_POS_HEADER                    "POS"
#define CNC_ACK_HEADER                    "ACK"
#define CNC_NAK_HEADER                    "NAK"
#define CNC_POS_ACK_NAK_HEADER_LEN        3
//                                         Seq,  x,  y,  z,Ste,inQueue
#define CNC_POS_ACK_NAK_PARAMS            "%lu,%ld,%ld,%ld,%lx,%u"

// Command flags
// -------------
#define CMD_FLAGS_CRC_MASK                0x000000FFL
#define CMD_FLAG_SPINDLE_ON               0x00000100L
#define CMD_FLAG_CALIBRATION              0x00000200L
#define CMD_CALIBRATION_COMPLETE          0x00000400L

// State flags
// -----------
#define CNC_STATE_MOTOR_CRC_ERROR         0x80000000L
#define CNC_STATE_NETWORK_CRC_ERROR       0x40000000L
#define CNC_STATE_LIMIT_ERROR             0x20000000L
#define CNC_STATE_CALIBRATION_FAILED      0x10000000L
#define CNC_STATE_COMMUNICATION_ERROR     0x08000000L
                                      
#define CNC_STATE_COMMAND_QUEUE_FULL      0x00002000L
#define CNC_STATE_POS_SENSOR_XL           0x00001000L
#define CNC_STATE_POS_SENSOR_XR           0x00000800L
#define CNC_STATE_POS_SENSOR_ZL           0x00000400L
#define CNC_STATE_POS_SENSOR_ZR           0x00000200L
#define CNC_STATE_POS_SENSOR_Y            0x00000100L
#define CNC_STATE_Z_CALIBRATED            0x00000080L
#define CNC_STATE_Y_CALIBRATED            0x00000040L
#define CNC_STATE_X_CALIBRATED            0x00000020L
#define CNC_STATE_CALIBRATING             0x00000010L
#define CNC_STATE_MANUAL_MODE             0x00000008L
#define CNC_STATE_IDLE                    0x00000004L
#define CNC_STATE_LITTLE_ENDIAN           0x00000002L
#define CNC_STATE_CONNECTED               0x00000001L
