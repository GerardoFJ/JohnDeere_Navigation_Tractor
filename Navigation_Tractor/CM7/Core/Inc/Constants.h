#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//STRUCTURS
typedef struct {
    uint32_t id; //CAN ID
    uint8_t  data[8]; //CAN 8bytes message
} CanFrame_t;

typedef struct {
    float wheel_v_mps;    // wheel linear velocity [m/s]
    float yaw_val; 		  // angular yaw data [radians]
    int32_t last_ticks;   // last encoder count
} OdomData_t;

typedef union {
    float float_val;
    uint8_t binary[4];
} FloatConverter;

typedef struct {
    float x;
    float y;
    float theta;
} Pose2D_t;

typedef struct {
    float v_cmd;      // m/s
    float delta_cmd;  // rad
} ControlCmd_t;

typedef struct {
    float x;
    float y;
} Waypoint;


//CAN CONSTANTS
#define ENCODER_CAN_ID      0x20      //Encoder CAN ID
#define BNO_CAN_ID      0x21      //Encoder CAN ID
//PHYSICAL OFFSETS CONSTANTS
#define INITIAL_STATE 1500
#define SERVO_STATE 1589
//BLUETOOTH CONSTANTS
#define BUFFER_SIZE 100
//ODOMETRY CONSTANTS
#define WHEEL_RADIUS_M      0.0314f   // 3.14cm radius
#define TICKS_PER_REV       142       //TICKS_PER_REV
//CONTROL CONSTANTS
#define MAX_VEL  			500		  //Max vel (0 - 500) valid range


#define LOOKAHEAD_DIST 0.2f          // meters (tune)
#define WHEELBASE      0.1365f         // meters (measure your car)
#define MAX_STEER_ANGLE (26.0f * (M_PI/180.0f))   // rad (±25 degrees)

#define NUM_WP 3




#endif
