#ifndef CONSTANTS_H_
#define CONSTANTS_H_
//CAN CONSTANTS
#define ENCODER_CAN_ID      0x20      //Encoder CAN ID
//PHYSICAL OFFSETS CONSTANTS
#define INITIAL_STATE 1500
#define SERVO_STATE 1589
//BLUETOOTH CONSTANTS
#define BUFFER_SIZE 100
//ODOMETRY CONSTANTS
#define WHEEL_RADIUS_M      0.0314f   // 3.14cm radius
#define TICKS_PER_REV       121       //TICKS_PER_REV
//CONTROL CONSTANTS
#define CONTROL_FREQUENCY 	100.0f      //Control frequency hz
#define TARGET_DISTANCE     1.0f	  //Target distance in Y in M
#define KP_CONSTANT  		100		  //KP Constant for distance P control
#define MAX_VEL  			100		  //Max vel (0 - 500) valid range
#define MIN_VEL				70		  //Min vel (0 -500) valid range
#define POSITION_TOLERANCE  0.01f  	  //Tolerance distance in M

//STRUCTURS
typedef struct {
    uint32_t id; //CAN ID
    uint8_t  data[8]; //CAN 8bytes message
} CanFrame_t;

typedef struct {
    float wheel_v_mps;    // wheel linear velocity [m/s]
    int32_t last_ticks;   // last encoder count
} OdomData_t;

#endif
