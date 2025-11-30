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
    float x;		//m
    float y;		//m
    float theta;	//rad
} Pose2D_t;

typedef struct {
    float x;		//m
    float y;		//m
    float angle;	//m
    int active;		//active message
} Pose2D_Camera;

typedef struct {
    float v_cmd;      // m/s
    float delta_cmd;  // rad
} ControlCmd_t;

typedef struct {
    float x;		//m
    float y;		//m
} Waypoint;


//CAN CONSTANTS
#define ENCODER_CAN_ID      0x20     //Encoder CAN ID
#define BNO_CAN_ID      	0x21     //Encoder CAN ID
#define CAMERA_CAN_ID      	0x22     //Encoder CAN ID
//PHYSICAL OFFSETS CONSTANTS
#define INITIAL_STATE 		1500
#define SERVO_STATE 		1589
#define SERVO_MIN			1000
#define SERVO_MAX 			2000
#define WHEELBASE      		0.1365f   // meters (measure your car)
#define MAX_STEER_ANGLE 	(26.0f * (M_PI/180.0f))   // rad (±25 degrees)
//BLUETOOTH CONSTANTS
#define BUFFER_SIZE 		100
//ODOMETRY CONSTANTS
#define WHEEL_RADIUS_M      0.0314f  // 3.14cm radius
#define TICKS_PER_REV       142      //TICKS_PER_REV
//CONTROL CONSTANTS
#define MAX_VEL  			500		 // Max vel (0 - 500) valid range
#define MIN_VEL				0.3f     //	min movement base pwm min_vel * MAX_VEL
#define STANDARD_VEL        0.3f     // Standard waypoint follower vel m/s
#define KP_CONSTANT			0.5f     // kp constant for vel PI control
#define KI_CONSTANT			0.001f	 // ki constant for vel PI control
//Task frequency constants
#define ACTUATOR_FREQUENCY  60 	     //Actuator Frequency task
#define CONTROL_FREQUENCY  	60       //Actuator Frequency task
#define DEBUG_FREQUENCY 	10       //debug Frequency task
#define WAYPOINT_MANAGER_FREQUENCY 50
//WAYPOINT MANAGER CONSTANTS
#define INITIAL_WAYPOINT 	0		  // initial waypoint index to follow
#define NUM_WP 				4		  // number of waypoints declared
#define WAYPOINT_TOLERANCE  0.17f	  //waypoint distance tolerance (m)
#define STOP_TIME 			3.0f 	  //Stop delay on each waypoint
#define LOOKAHEAD_DIST 		0.15f      // meters (tune)




#endif
