
//  S T A T U S    F L A G S
//  ------------------------
#define STATUS_LIMIT              0x00000001l
#define STATUS_NUMBER             0x00000008l
#define STATUS_SYNTAX             0x00000010l
#define STATUS_MATH               0x00000020l
#define STATUS_COMM               0x00000040l
#define STATUS_ERROR_MASK         0x0000FFFFl
// Warnings                       
#define STATUS_LITLE_ENDIAN       0x80000000l
#define STATUS_GOT_POSITION       0x40000000l
#define STATUS_NEED_CALIBRATION   0x20000000l
