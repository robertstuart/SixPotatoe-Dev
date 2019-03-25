const unsigned int NO_DEBUG = 0x4242;  // If received, not displayed.

const int XBEE_2P_SH         = 0x13A200;    // TwoPotatoe Serial
const int XBEE_2P_SL         = 0x409BD79F;
const int XBEE_3P_SH         = 0x13A200;    // ThreePotatoe Serial
const int XBEE_3P_SL         = 0x409FEBCF;
const int XBEE_C1_SH         = 0x13A200;    // Controller1 Serial
const int XBEE_C1_SL         = 0x409FEBF8;

const int XBEE_DEST_2P       = 0;           // TwoPotatoe destination
const int XBEE_DEST_3P       = 1;           // ThreePotatoe destination
const int XBEE_DEST_C1       = 3;           // Controller 1 destination

const int SEND_MESSAGE     = 129;
const int SEND_FPS         = 130;
const int SEND_PITCH       = 131;
const int SEND_HEADING     = 132;
const int SEND_SONAR_R     = 133;
const int SEND_ROUTE_STEP  = 134;
const int SEND_DUMP_DATA   = 135;
const int SEND_STATE       = 136;
const int SEND_BATT_A      = 137;
const int SEND_BATT_B      = 138;
const int SEND_MODE        = 139;
const int SEND_VALSET      = 140;
const int SEND_ROUTE_NAME  = 141;
const int SEND_X           = 142;
const int SEND_Y           = 143;
const int SEND_SONAR_F     = 144;
const int SEND_SONAR_L     = 145;
const int SEND_DUMP_TICKS  = 146;
const int SEND_XPOS        = 190;
const int SEND_YPOS        = 191;

const int RCV_JOYX         = 129;
const int RCV_JOYY         = 130;
const int RCV_RUN          = 131;
const int RCV_LIGHTS       = 132;
const int RCV_LOG          = 133;
const int RCV_DUMP_START   = 134;
const int RCV_MODE         = 135;
const int RCV_RT_ENABLE    = 146; // toggle
const int RCV_RT_START     = 147;
const int RCV_RT_SET       = 148; // 0 to decrease, 1 to increase
const int RCV_GET_UP       = 150;
const int RCV_GET_DOWN     = 151;
const int RCV_DUMP_TICKS   = 157; 
const int RCV_JOYX_I       = 160;
const int RCV_JOYY_I       = 161;
const int RCV_V1           = 162;
const int RCV_V2           = 163;
const int RCV_LIFT         = 164;
const int RCV_KILL_UP      = 165;
const int RCV_MOT_DISABLE  = 166;

const int SEND_RCV_TERM    =   0;

// Modes of operation
const int MODE_XXXXXXXX       = 0;  // 
const int MODE_PWM_SPEED      = 1;  // pw values sent from controller
const int MODE_DRIVE          = 2;  // sequences batch-loaded from controller
const int MODE_TP4            = 3;  // Motor speed controlled from interrupts
const int MODE_TP_SPEED       = 4;  // on-board motor speed
const int MODE_TP_SEQUENCE    = 5;  // sequences batch-loaded from controller
const int MODE_PULSE_SEQUENCE = 6;  // motor test
const int MODE_PULSE          = 7;
const int MODE_TP5            = 8;
const int MODE_2P             = 9;
const int MODE_4P             = 10;
const int MODE_RW_ANGLE       = 11;  // Reaction wheel angle
const int BLOCK_DATA          = 100; 

// Motor Modes
const int MM_DRIVE_BRAKE = 0;
const int MM_DRIVE_COAST = 1;

// value sets
const int VAL_SET_A           = 0;
const int VAL_SET_B           = 1; 
const int VAL_SET_C           = 2;
