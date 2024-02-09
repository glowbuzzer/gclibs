#ifndef __SHARED_MEM_TYPES_H
#define __SHARED_MEM_TYPES_H

#include "stdbool.h"
#include "stdint.h"
#pragma pack(4)

#define MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION 7
#define MAX_NUMBER_OF_LIMITS_IN_KINEMATICS_CONFIGURATION 7
#define MAX_NUMBER_OF_LIMITS_IN_JOINT_CONFIGURATION 7
#define MAX_NUMBER_OF_SUPERIMPOSED_ACTIVITIES 2
#define MAX_NUMBER_OF_SPLINE_POINTS 24
#define MAX_NUMBER_OF_KINEMATICS_CONFIGURATIONS 3
#define MAX_NUMBER_OF_JOINTS 12
#define MAX_NUMBER_OF_DO 32
#define MAX_NUMBER_OF_AO 32
#define MAX_SIZE_OF_MATRIX 100

#define GBC_MD5_SUM "3a50cd43ee49362d67686f17e66a33d2"

// DEFINES
#define DEFAULT_HLC_HEARTBEAT_TOLERANCE 2000
#define JOINT_CONTROL_WORD_CST_POS_VEL_DISABLE_BIT 1

// ENUMS
    enum FAULT_CAUSE {
        FAULT_CAUSE_SAFETY_BIT_NUM                          = (0),
        FAULT_CAUSE_DRIVE_FAULT_BIT_NUM                    = (1),
        FAULT_CAUSE_GBC_FAULT_REQUEST_BIT_NUM              = (2),
        FAULT_CAUSE_HEARTBEAT_LOST_BIT_NUM                 = (3),
        FAULT_CAUSE_LIMIT_REACHED_BIT_NUM                  = (4),
        FAULT_CAUSE_DRIVE_STATE_CHANGE_TIMEOUT_BIT_NUM     = (5),
        FAULT_CAUSE_DRIVE_FOLLOW_ERROR_BIT_NUM             = (6),
        FAULT_CAUSE_DRIVE_NO_REMOTE_BIT_NUM                = (7),
        FAULT_CAUSE_ECAT_BIT_NUM                           = (8),
        FAULT_CAUSE_DRIVE_WARNING_BIT_NUM                  = (9),
        FAULT_CAUSE_GBC_OPERATION_ERROR_BIT_NUM            = (10),
        FAULT_CAUSE_DRIVE_MOOERROR_BIT_NUM                 = (11),
        FAULT_CAUSE_ECAT_SLAVE_ERROR_BIT_NUM               = (12),
        FAULT_CAUSE_PLC_SIGNALLED_ERROR_BIT_NUM            = (13),
        FAULT_CAUSE_HOMING_ERROR_BIT_NUM                   = (14),
        FAULT_CAUSE_GBC_TO_PLC_CON_ERROR_BIT_NUM           = (15),
        FAULT_CAUSE_MOVE_NOT_OP_EN_BIT_NUM                 = (16),
        FAULT_CAUSE_DRIVE_STATE_MISMATCH_BIT_NUM           = (17),
        FAULT_CAUSE_FSOE_ERROR_BIT_NUM                     = (18),
    };
    enum STATUS_WORD_GBEM {
        STATUS_WORD_GBEM_ALIVE_BIT_NUM                      = (16),
        STATUS_WORD_GBEM_BOOT_IN_PROGRESS_BIT_NUM           = (17),
        STATUS_WORD_GBEM_BOOTED_BIT_NUM                     = (18),
        STATUS_WORD_GBEM_HOMING_NEEDED_BIT_NUM              = (19),
        STATUS_WORD_GBEM_WAITING_FOR_START_HOMING_BIT_NUM   = (20),
        STATUS_WORD_GBEM_HOMING_IN_PROGRESS_BIT_NUM         = (21),
        STATUS_WORD_GBEM_HOMING_ERROR_BIT_NUM               = (23),
        STATUS_WORD_GBEM_HOMING_ATTAINED_BIT_NUM            = (24),
    };
    enum CONTROL_WORD_GBC_GBEM {
        CONTROL_WORD_GBC_OPERATION_ERROR_BIT_NUM           = (16),
        CONTROL_WORD_GBEM_START_HOMING_BIT_NUM             = (17),
        CONTROL_WORD_GBC_REQUEST_FAULT_BIT_NUM             = (18),
        CONTROL_WORD_GBEM_REBOOT_BIT_NUM                   = (20),
        CONTROL_WORD_GBEM_DOWNLOAD_DRIVE_LOGS_BIT_NUM      = (21),
    };
    enum FSOE_SLAVE_HIGH_LEVEL_STATE {
        FSOE_SLAVE_HIGH_LEVEL_STATE_NONE                   = (0),
        FSOE_SLAVE_HIGH_LEVEL_STATE_PROCESS_DATA           = (1),
        FSOE_SLAVE_HIGH_LEVEL_STATE_RESET                  = (2),
        FSOE_SLAVE_HIGH_LEVEL_STATE_SESSION                = (3),
        FSOE_SLAVE_HIGH_LEVEL_STATE_CONNECTION             = (4),
        FSOE_SLAVE_HIGH_LEVEL_STATE_PARAMETER              = (5),
        FSOE_SLAVE_HIGH_LEVEL_STATE_FAILSAFEDATA           = (6),
        FSOE_SLAVE_HIGH_LEVEL_STATE_UNKNOWN                = (7),
    };
    enum FSOE_SLAVE_TYPE {
        FSOE_SLAVE_TYPE_NONE                               = (0),
        FSOE_SLAVE_TYPE_SYNAPTICON                         = (1),
        FSOE_SLAVE_TYPE_EL1904                             = (2),
        FSOE_SLAVE_TYPE_EL2904                             = (3),
        FSOE_SLAVE_TYPE_SCU_1_EC                           = (4),
        FSOE_SLAVE_TYPE_EL6900                             = (5),
        FSOE_SLAVE_TYPE_EL6910                             = (6),
    };
    enum FSOE_MASTER_HIGH_LEVEL_STATE {
        FSOE_MASTER_HIGH_LEVEL_STATE_NONE                  = (0),
        FSOE_MASTER_HIGH_LEVEL_STATE_START_UP              = (1),
        FSOE_MASTER_HIGH_LEVEL_STATE_SENDCONFIG            = (2),
        FSOE_MASTER_HIGH_LEVEL_STATE_STARTUP_BUS           = (3),
        FSOE_MASTER_HIGH_LEVEL_STATE_RUN                   = (4),
        FSOE_MASTER_HIGH_LEVEL_STATE_STOP                  = (5),
        FSOE_MASTER_HIGH_LEVEL_STATE_ERROR                 = (6),
        FSOE_MASTER_HIGH_LEVEL_STATE_ALARM                 = (7),
        FSOE_MASTER_HIGH_LEVEL_STATE_NO_NETWORK            = (8),
    };
    enum CONFIG_STATUS {
        CONFIG_STATUS_NONE,
        CONFIG_STATUS_RELOAD,
        CONFIG_STATUS_RELOAD_FULL,
        CONFIG_STATUS_LOADED,
    };
    enum LIMITPROFILE {
        LIMITPROFILE_DEFAULT,
        LIMITPROFILE_JOGGING,
        LIMITPROFILE_RAPIDS,
    };
    enum MACHINETARGET {
        MACHINETARGET_NONE,
        MACHINETARGET_FIELDBUS,
        MACHINETARGET_SIMULATION,
    };
    enum OPERATION_ERROR {
        OPERATION_ERROR_NONE,
        OPERATION_ERROR_HLC_HEARTBEAT_LOST,
        OPERATION_ERROR_OPERATION_NOT_ENABLED,
        OPERATION_ERROR_INVALID_ARC,
        OPERATION_ERROR_TOOL_INDEX_OUT_OF_RANGE,
        OPERATION_ERROR_JOINT_LIMIT_EXCEEDED,
        OPERATION_ERROR_KINEMATICS_FK_INVALID_VALUE,
        OPERATION_ERROR_KINEMATICS_IK_INVALID_VALUE,
        OPERATION_ERROR_KINEMATICS_INVALID_KIN_CHAIN_PARAMS,
        OPERATION_ERROR_JOINT_DISCONTINUITY,
        OPERATION_ERROR_JOINT_OVER_SPEED,
        OPERATION_ERROR_INVALID_ROTATION,
        OPERATION_ERROR_CONFIG_RELOADED,
        OPERATION_ERROR_KINEMATICS_ENVELOPE_VIOLATION,
        OPERATION_ERROR_KINEMATICS_NEAR_SINGULARITY,
    };
    enum POSITIONREFERENCE {
        ABSOLUTE,
        RELATIVE,
        MOVESUPERIMPOSED,
    };
    enum ROTATIONINTERPOLATION {
        ROTATIONINTERPOLATION_SHORT_SLERP,
        ROTATIONINTERPOLATION_LONG_SLERP,
    };
    enum TASK_STATE {
        TASK_NOTSTARTED,
        TASK_RUNNING,
        TASK_FINISHED,
        TASK_PAUSED,
        TASK_STOPPING,
        TASK_CANCELLED,
        TASK_ERROR,
    };
    enum TASK_COMMAND {
        TASK_IDLE,
        TASK_RUN,
        TASK_CANCEL,
        TASK_PAUSE,
        TASK_RESUME,
    };
    enum GTLT {
        GREATERTHAN,
        LESSTHAN,
    };
    enum ACTIVITYTYPE {
        ACTIVITYTYPE_NONE,
        ACTIVITYTYPE_PAUSEPROGRAM,
        ACTIVITYTYPE_ENDPROGRAM,
        ACTIVITYTYPE_MOVEJOINTS,
        ACTIVITYTYPE_MOVEJOINTSATVELOCITY,
        ACTIVITYTYPE_MOVELINE,
        ACTIVITYTYPE_MOVEVECTORATVELOCITY,
        ACTIVITYTYPE_MOVEROTATIONATVELOCITY,
        ACTIVITYTYPE_MOVEARC,
        ACTIVITYTYPE_MOVEINSTANT,
        ACTIVITYTYPE_MOVETOPOSITION,
        ACTIVITYTYPE_SETDOUT,
        ACTIVITYTYPE_SETIOUT,
        ACTIVITYTYPE_SETAOUT,
        ACTIVITYTYPE_DWELL,
        ACTIVITYTYPE_SPINDLE,
        ACTIVITYTYPE_MOVEJOINTSINTERPOLATED,
        ACTIVITYTYPE_SET_UIOUT,
        ACTIVITYTYPE_SET_EXTERNAL_IOUT,
        ACTIVITYTYPE_GEARINPOS,
        ACTIVITYTYPE_GEARINVELO,
        ACTIVITYTYPE_SET_EXTERNAL_DOUT,
        ACTIVITYTYPE_TOOLOFFSET,
        ACTIVITYTYPE_SET_EXTERNAL_UIOUT,
        ACTIVITYTYPE_STRESSTEST,
    };
    enum ACTIVITYSTATE {
        ACTIVITY_INACTIVE,
        ACTIVITY_ACTIVE,
        ACTIVITY_COMPLETED,
        ACTIVITY_BLEND_ACTIVE,
        ACTIVITY_CANCELLED,
    };
    enum STRATEGYGEARINPOS {
        PHASESHIFT,
        EARLY,
        LATE,
        SLOW,
    };
    enum TRIGGERTYPE {
        TRIGGERTYPE_RISING,
        TRIGGERTYPE_FALLING,
        TRIGGERTYPE_NONE,
    };
    enum ARCTYPE {
        ARCTYPE_CENTRE,
        ARCTYPE_RADIUS,
    };
    enum ARCDIRECTION {
        ARCDIRECTION_CW,
        ARCDIRECTION_CCW,
    };
    enum SPINDLEDIRECTION {
        SPINDLEDIRECTION_CW,
        SPINDLEDIRECTION_CCW,
    };
    enum JOINT_TYPE {
        JOINT_PRISMATIC,
        JOINT_REVOLUTE,
    };
    enum JOINT_MODEOFOPERATION {
        JOINT_MODEOFOPERATION_NONE   = 0,
        JOINT_MODEOFOPERATION_CSP    = 1,
        JOINT_MODEOFOPERATION_CSV    = 2,
        JOINT_MODEOFOPERATION_CST    = 4,
        JOINT_MODEOFOPERATION_HOMING = 8,
    };
    enum JOINT_FINITECONTINUOUS {
        JOINT_FINITE,
        JOINT_CONTINUOUS,
    };
    enum JOINT_TORQUE_MODE {
        JOINT_TORQUE_MODE_DEFAULT   = 0,
        JOINT_TORQUE_MODE_GRAVITY   = 1,
        JOINT_TORQUE_MODE_DIRECT    = 2,
    };
    enum KC_KINEMATICSCONFIGURATIONTYPE {
        KC_NAKED,
        KC_SIXDOF,
        KC_IGUS,
        KC_SCARA,
        KC_CARTESIAN,
        KC_CARTESIAN_SLAVED,
        KC_TWO_LINK,
        KC_CUSTOM,
        KC_REVOLUTE_DELTA,
        KC_ANGLED_LINEAR_DELTA,
        KC_RRPR_SCARA,
        KC_PRISMATIC_STEWART,
        KC_PUMA,
        KC_FIVE_AXIS,
        KC_WMR,
        KC_MOVEABLE_SIXDOF,
    };
    enum KC_SHOULDERCONFIGURATION {
        KC_LEFTY,
        KC_RIGHTY,
    };
    enum KC_ELBOWCONFIGURATION {
        KC_EPOSITIVE,
        KC_ENEGATIVE,
    };
    enum KC_WRISTCONFIGURATION {
        KC_WPOSITIVE,
        KC_WNEGATIVE,
    };
    enum BLENDTYPE {
        BLENDTYPE_NONE,
        BLENDTYPE_OVERLAPPED,
    };
    enum SYNCTYPE {
        SYNCTYPE_NONE,
        SYNCTYPE_DURATION_MS,
        SYNCTYPE_AT_TICK,
    };
    enum OPENCLOSED {
        OPEN,
        CLOSED,
    };
    enum STREAMCOMMAND {
        STREAMCOMMAND_RUN,
        STREAMCOMMAND_PAUSE,
        STREAMCOMMAND_STOP,
    };
    enum STREAMSTATE {
        STREAMSTATE_IDLE,
        STREAMSTATE_ACTIVE,
        STREAMSTATE_PAUSED,
        STREAMSTATE_PAUSED_BY_ACTIVITY,
        STREAMSTATE_STOPPING,
        STREAMSTATE_STOPPED,
    };
    enum TRIGGERON {
        TRIGGERON_NONE,
        TRIGGERON_ANALOG_INPUT,
        TRIGGERON_DIGITAL_INPUT,
        TRIGGERON_INTEGER_INPUT,
        TRIGGERON_TIMER,
        TRIGGERON_TICK,
    };
    enum TRIGGERACTION {
        TRIGGERACTION_NONE,
        TRIGGERACTION_CANCEL,
        TRIGGERACTION_START,
    };
    enum DIN_SAFETY_TYPE {
        DIN_SAFETY_TYPE_NORMAL,
        DIN_SAFETY_TYPE_HIDDEN,
    };
    enum SERIAL_CONTROL_WORD {
        SERIAL_TRANSMIT_REQUEST_BIT_NUM                     = (0),
        SERIAL_RECEIVE_ACCEPTED_BIT_NUM                     = (1),
        SERIAL_INIT_REQUEST_BIT_NUM                         = (2),
    };
    enum SERIAL_STATUS_WORD {
        SERIAL_TRANSMIT_ACCEPTED_BIT_NUM                    = (0),
        SERIAL_RECEIVE_REQUEST_BIT_NUM                      = (1),
        SERIAL_INIT_ACCEPTED_BIT_NUM                        = (2),
        SERIAL_ERROR_BIT_NUM                                = (3),
    };


// STRUCTS
struct sharedMemHeader {
        enum CONFIG_STATUS status;
};

struct limitConfiguration {
        double vmax;
        double amax;
        double jmax;
};

struct machineConfig {
        uint16_t busCycleTime;
        uint16_t statusFrequency;
        bool estopEnabled;
        uint8_t estopInput;
        uint32_t heartbeatTimeout;
};

struct machineStatus {
        uint32_t statusWord;
        uint32_t activeFault;
        uint32_t faultHistory;
        uint32_t heartbeat;
        enum MACHINETARGET target;
        uint16_t targetConnectRetryCnt;
        enum OPERATION_ERROR operationError;
        char operationErrorMessage[256];
};

struct machineCommand {
        uint32_t controlWord;
        uint32_t hlcControlWord;
        uint32_t heartbeat;
        enum MACHINETARGET target;
};

struct streamConfig {
        bool enableEndProgram;
};

struct streamStatus {
        enum STREAMSTATE streamState;
        uint32_t tag;
        uint32_t time;
};

struct streamCommand {
        enum STREAMCOMMAND streamCommand;
};

struct moveParametersConfig {
        double vmax;
        uint8_t vmaxPercentage;
        uint8_t amaxPercentage;
        uint8_t jmaxPercentage;
        uint8_t limitConfigurationIndex;
        enum BLENDTYPE blendType;
        uint16_t blendTimePercentage;
        double blendTolerance;
        uint8_t toolIndex;
        enum SYNCTYPE syncType;
        uint32_t syncValue;
        bool optimizeJointDistance;
};

struct vector3 {
        double x;
        double y;
        double z;
};

struct quat {
        double w;
        double x;
        double y;
        double z;
};

struct cartesianPosition {
        enum POSITIONREFERENCE positionReference;
        struct vector3 translation;
        struct quat rotation;
        uint16_t frameIndex;
};

struct positionAbsRel {
        enum POSITIONREFERENCE positionReference;
        struct vector3 translation;
};

struct cartesianVector {
        struct vector3 vector;
        uint16_t frameIndex;
};

struct doubleValue {
        double value;
};

struct arcsConfig {
        enum ARCTYPE arcType;
        enum ARCDIRECTION arcDirection;
        struct cartesianPosition destination;
        struct quat plane;
        enum ROTATIONINTERPOLATION rotationInterpolation;
    union {
        struct positionAbsRel centre;
        struct doubleValue radius;
    };
};

struct cartesianPositionsConfig {
        struct cartesianPosition position;
        uint32_t configuration;
};

struct triggerOnAnalogInput {
        uint8_t input;
        enum GTLT when;
        double value;
};

struct triggerOnDigitalInput {
        uint8_t input;
        bool safeInput;
        enum TRIGGERTYPE when;
};

struct triggerOnIntegerInput {
        uint8_t input;
        enum GTLT when;
        uint32_t value;
};

struct triggerOnTimer {
        uint32_t delay;
};

struct triggerOnTick {
        uint32_t value;
};

struct triggerParams {
        enum TRIGGERON type;
        enum TRIGGERACTION action;
    union {
        struct triggerOnAnalogInput analog;
        struct triggerOnDigitalInput digital;
        struct triggerOnIntegerInput integer;
        struct triggerOnTimer timer;
        struct triggerOnTick tick;
    };
};

struct taskConfig {
        uint16_t activityCount;
        uint16_t firstActivityIndex;
        struct triggerParams triggers[3];
};

struct taskStatus {
        enum TASK_STATE taskState;
        uint16_t currentActivityIndex;
};

struct taskCommand {
        enum TASK_COMMAND taskCommand;
};

struct pidConfig {
        double kp;
        double ki;
        double kd;
        double maxIntegral;
        double minIntegral;
        uint16_t sampleTime;
};

struct jointConfig {
        enum JOINT_TYPE jointType;
        struct limitConfiguration limits[MAX_NUMBER_OF_LIMITS_IN_JOINT_CONFIGURATION];
        enum JOINT_MODEOFOPERATION preferredMode;
        uint8_t supportedModes;
        uint8_t supportedTorqueModes;
        double scale;
        double scalePos;
        double scaleVel;
        double scaleTorque;
        uint8_t pow10;
        double negLimit;
        double posLimit;
        bool inverted;
        enum JOINT_FINITECONTINUOUS finiteContinuous;
        bool isVirtualInternal;
        bool isVirtualFromEncoder;
        uint8_t correspondingJointNumberOnPhysicalFieldbus;
        uint8_t correspondingJointNumberOnVirtualFieldbus;
        struct pidConfig pidConfig[3];
};

struct jointStatus {
        uint16_t statusWord;
        double actPos;
        double actVel;
        double actTorque;
        int32_t actControlEffort;
};

struct jointCommand {
        uint16_t controlWord;
        double setTorque;
        enum JOINT_TORQUE_MODE torqueMode;
};

struct matrixInstanceDouble {
        uint16_t numRows;
        uint16_t numCols;
        double data[MAX_SIZE_OF_MATRIX];
        uint8_t invJointAngles[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
};

struct rollPitchYaw {
        double r;
        double p;
        double y;
};

struct urdfFrame {
        struct vector3 translation;
        struct rollPitchYaw rpy;
};

struct rigidBodyInertia {
        double m;
        struct vector3 h;
        double Ixx;
        double Iyy;
        double Izz;
        double Ixy;
        double Ixz;
        double Iyz;
};

struct inverseDynamicParameters {
        struct urdfFrame urdfFrame;
        struct rigidBodyInertia rigidBodyInertia;
        double jointOffset;
        double jointScale;
        double jointInertia;
        struct vector3 jointAxis;
        double damping;
        double friction;
};

struct sphericalEnvelope {
        struct vector3 center;
        double radius[2];
};

struct velocityScaling {
        bool enabled;
        struct triggerOnDigitalInput trigger;
        double scaleFactor;
};

struct kinematicsConfigurationConfig {
        enum KC_KINEMATICSCONFIGURATIONTYPE kinematicsConfigurationType;
        uint8_t supportedConfigurationBits;
        uint16_t frameIndex;
        uint8_t participatingJoints[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        uint8_t participatingJointsCount;
        double extentsX[2];
        double extentsY[2];
        double extentsZ[2];
        double scaleX;
        double scaleY;
        double scaleZ;
        struct limitConfiguration linearLimits[MAX_NUMBER_OF_LIMITS_IN_KINEMATICS_CONFIGURATION];
        struct limitConfiguration angularLimits[MAX_NUMBER_OF_LIMITS_IN_KINEMATICS_CONFIGURATION];
        struct velocityScaling velocityScaling;
        struct matrixInstanceDouble kinChainParams;
        struct inverseDynamicParameters inverseDynamicParams[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        struct sphericalEnvelope sphericalEnvelope;
        double cylindricalEnvelope[2];
};

struct kinematicsConfigurationStatus {
        double froTarget;
        double froActual;
        uint8_t configuration;
        struct vector3 cartesianActPos;
        struct quat cartesianActOrientation;
        struct vector3 cartesianActVel;
        struct vector3 cartesianActAcc;
        bool limitsDisabled;
        uint8_t isNearSingularity;
        uint8_t toolIndex;
};

struct kinematicsConfigurationCommand {
        bool doStop;
        bool disableLimits;
        double fro;
        struct vector3 translation;
        struct quat rotation;
};

struct dinConfig {
        bool inverted;
};

struct dinStatus {
        bool actValue;
};

struct dinCommand {
        bool override;
        bool setValue;
};

struct safetyDinConfig {
        bool inverted;
        enum DIN_SAFETY_TYPE type;
};

struct safetyDinStatus {
        bool actValue;
};

struct safetyDinCommand {
        bool override;
        bool setValue;
};

struct externalDinConfig {
        bool inverted;
};

struct externalDinStatus {
        bool actValue;
};

struct externalDinCommand {
        bool override;
        bool setValue;
};

struct doutConfig {
        bool inverted;
        uint8_t loopback;
};

struct doutStatus {
        bool effectiveValue;
};

struct doutCommand {
        bool override;
        bool setValue;
};

struct safetyDoutConfig {
        bool inverted;
        uint8_t loopback;
};

struct safetyDoutStatus {
        bool effectiveValue;
};

struct safetyDoutCommand {
        bool override;
        bool setValue;
};

struct externalDoutConfig {
        bool inverted;
        uint8_t loopback;
};

struct externalDoutStatus {
        bool effectiveValue;
};

struct externalDoutCommand {
        bool override;
        bool setValue;
};

struct ainConfig {
        bool useForVirtualAxis;
        uint8_t jointIndexForVirtualAxis;
};

struct ainStatus {
        float actValue;
};

struct ainCommand {
        bool override;
        float setValue;
};

struct aoutConfig {
};

struct aoutStatus {
        float effectiveValue;
};

struct aoutCommand {
        bool override;
        float setValue;
};

struct uiinConfig {
};

struct uiinStatus {
        uint32_t actValue;
};

struct uiinCommand {
        bool override;
        uint32_t setValue;
};

struct iinConfig {
};

struct iinStatus {
        int32_t actValue;
};

struct iinCommand {
        bool override;
        int32_t setValue;
};

struct externalUiinConfig {
};

struct externalUiinStatus {
        uint32_t actValue;
};

struct externalUiinCommand {
        bool override;
        uint32_t setValue;
};

struct externalIinConfig {
};

struct externalIinStatus {
        int32_t actValue;
};

struct externalIinCommand {
        bool override;
        int32_t setValue;
};

struct uioutConfig {
};

struct uioutStatus {
        uint32_t effectiveValue;
};

struct uioutCommand {
        bool override;
        uint32_t setValue;
};

struct ioutConfig {
};

struct ioutStatus {
        int32_t effectiveValue;
};

struct ioutCommand {
        bool override;
        int32_t setValue;
};

struct externalUioutConfig {
};

struct externalUioutStatus {
        uint32_t effectiveValue;
};

struct externalUioutCommand {
        bool override;
        uint32_t setValue;
};

struct externalIoutConfig {
};

struct externalIoutStatus {
        int32_t effectiveValue;
};

struct externalIoutCommand {
        bool override;
        int32_t setValue;
};

struct moveJointsActivityParams {
        uint8_t kinematicsConfigurationIndex;
        double jointPositionArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        enum POSITIONREFERENCE positionReference;
        uint16_t moveParamsIndex;
};

struct moveJointsActivityStatus {
        uint8_t percentageComplete;
};

struct moveJointsActivityCommand {
        bool skipToNext;
};

struct moveJointsStream {
        uint8_t kinematicsConfigurationIndex;
        enum POSITIONREFERENCE positionReference;
        double jointPositionArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        struct moveParametersConfig moveParams;
};

struct moveJointsInterpolatedActivityParams {
        uint8_t kinematicsConfigurationIndex;
        double jointPositionArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        double jointVelocityArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        double timecode;
        uint16_t moveParamsIndex;
};

struct moveJointsInterpolatedActivityStatus {
};

struct moveJointsInterpolatedActivityCommand {
};

struct moveJointsInterpolatedStream {
        uint8_t kinematicsConfigurationIndex;
        double jointPositionArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        double jointVelocityArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
        double duration;
        struct moveParametersConfig moveParams;
};

struct moveJointsAtVelocityActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint16_t moveParamsIndex;
        double jointVelocityArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
};

struct moveJointsAtVelocityActivityStatus {
};

struct moveJointsAtVelocityActivityCommand {
        bool skipToNext;
};

struct moveJointsAtVelocityStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        double jointVelocityArray[MAX_NUMBER_OF_JOINTS_IN_KINEMATICS_CONFIGURATION];
};

struct moveLineActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint16_t moveParamsIndex;
        struct cartesianPosition line;
        uint8_t superimposedIndex;
};

struct moveLineActivityStatus {
};

struct moveLineActivityCommand {
        bool skipToNext;
};

struct moveLineStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct cartesianPosition line;
        uint8_t superimposedIndex;
};

struct moveVectorAtVelocityActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint16_t moveParamsIndex;
        struct cartesianVector vector;
};

struct moveVectorAtVelocityActivityStatus {
};

struct moveVectorAtVelocityActivityCommand {
        bool skipToNext;
};

struct moveVectorAtVelocityStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct cartesianVector vector;
};

struct moveRotationAtVelocityActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint16_t moveParamsIndex;
        struct cartesianVector axis;
};

struct moveRotationAtVelocityActivityStatus {
};

struct moveRotationAtVelocityActivityCommand {
        bool skipToNext;
};

struct moveRotationAtVelocityStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct cartesianVector axis;
};

struct moveArcActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint8_t superimposedIndex;
        uint16_t moveParamsIndex;
        struct arcsConfig arc;
};

struct moveArcActivityStatus {
};

struct moveArcActivityCommand {
        bool skipToNext;
};

struct moveArcStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct arcsConfig arc;
        uint8_t superimposedIndex;
};

struct moveInstantActivityParams {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct cartesianPosition position;
};

struct moveInstantActivityStatus {
};

struct moveInstantActivityCommand {
};

struct moveInstantStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct cartesianPosition position;
};

struct moveToPositionActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint16_t moveParamsIndex;
        struct cartesianPositionsConfig cartesianPosition;
};

struct moveToPositionActivityStatus {
};

struct moveToPositionActivityCommand {
        bool skipToNext;
};

struct moveToPositionStream {
        uint8_t kinematicsConfigurationIndex;
        struct moveParametersConfig moveParams;
        struct cartesianPositionsConfig cartesianPosition;
};

struct setDoutActivityParams {
        uint8_t doutToSet;
        bool valueToSet;
};

struct setDoutActivityStatus {
};

struct setDoutActivityCommand {
};

struct setAoutActivityParams {
        uint8_t aoutToSet;
        float valueToSet;
};

struct setAoutActivityStatus {
};

struct setAoutActivityCommand {
};

struct setIoutActivityParams {
        uint8_t ioutToSet;
        int32_t valueToSet;
};

struct setIoutActivityStatus {
};

struct setIoutActivityCommand {
};

struct setUioutActivityParams {
        uint8_t ioutToSet;
        uint32_t valueToSet;
};

struct setUioutActivityStatus {
};

struct setUioutActivityCommand {
};

struct dwellActivityParams {
        uint32_t ticksToDwell;
};

struct dwellActivityStatus {
};

struct dwellActivityCommand {
        bool skipToNext;
};

struct spindleConfig {
        uint16_t enableDigitalOutIndex;
        uint16_t directionDigitalOutIndex;
        bool directionInvert;
        uint16_t speedAnalogOutIndex;
};

struct spindleActivityParams {
        uint16_t spindleIndex;
        bool enable;
        enum SPINDLEDIRECTION direction;
        float speed;
};

struct spindleActivityStatus {
};

struct spindleActivityCommand {
};

struct toolOffsetActivityParams {
        uint8_t kinematicsConfigurationIndex;
        uint8_t toolIndex;
};

struct gearInVeloActivityParams {
        uint8_t masterKinematicsConfigurationIndex;
        uint8_t slaveKinematicsConfigurationIndex;
        uint32_t gearingFrameIndex;
        double gearRatio;
        uint32_t syncActivationDelay;
};

struct gearInVeloActivityStatus {
        uint8_t percentageComplete;
        bool gearInFailed;
        bool gearedIn;
};

struct gearInVeloActivityCommand {
        bool skipToNext;
        double updatedRatio;
        bool updateRation;
};

struct gearInPosActivityParams {
        uint8_t masterKinematicsConfigurationIndex;
        uint8_t slaveKinematicsConfigurationIndex;
        uint8_t gearingFrameIndex;
        double gearRatio;
        enum STRATEGYGEARINPOS strategyToUse;
        double gearRatioMaster;
        double gearRatioSlave;
        struct cartesianPosition masterSyncPosition;
        struct cartesianPosition slaveSyncPosition;
        uint32_t syncActivationDelay;
};

struct gearInPosActivityStatus {
        uint8_t percentageComplete;
        bool gearInFailed;
        bool gearedIn;
};

struct gearInPosActivityCommand {
        bool skipToNext;
        double updatedRatioMaster;
        double updatedRatioSlave;
        struct cartesianPosition updatedMasterSyncPosition;
        struct cartesianPosition updatedSlaveSyncPosition;
};

struct stressTestActivityParams {
};

struct stressTestActivityStatus {
};

struct stressTestActivityCommand {
};

struct stressTestActivityStream {
};

struct activityConfig {
        enum ACTIVITYTYPE activityType;
        struct triggerParams triggers[3];
    union {
        struct moveJointsActivityParams moveJoints;
        struct moveJointsInterpolatedActivityParams moveJointsInterpolated;
        struct moveJointsAtVelocityActivityParams moveJointsAtVelocity;
        struct moveLineActivityParams moveLine;
        struct moveVectorAtVelocityActivityParams moveVectorAtVelocity;
        struct moveRotationAtVelocityActivityParams moveRotationAtVelocity;
        struct moveArcActivityParams moveArc;
        struct moveInstantActivityParams moveInstant;
        struct moveToPositionActivityParams moveToPosition;
        struct gearInPosActivityParams gearInPos;
        struct gearInVeloActivityParams gearInVelo;
        struct setDoutActivityParams setDout;
        struct setDoutActivityParams setExternalDout;
        struct setAoutActivityParams setAout;
        struct setIoutActivityParams setIout;
        struct setUioutActivityParams setUiout;
        struct setIoutActivityParams setExternalIout;
        struct setUioutActivityParams setExternalUiout;
        struct dwellActivityParams dwell;
        struct spindleActivityParams spindle;
        struct stressTestActivityParams stressTest;
    };
};

struct activityStatus {
        enum ACTIVITYSTATE state;
        uint32_t tag;
    union {
        struct moveJointsActivityStatus moveJoints;
        struct moveJointsInterpolatedActivityStatus moveJointsInterpolated;
        struct moveJointsAtVelocityActivityStatus moveJointsAtVelocity;
        struct moveLineActivityStatus moveLine;
        struct moveVectorAtVelocityActivityStatus moveVectorAtVelocity;
        struct moveRotationAtVelocityActivityStatus moveRotationAtVelocity;
        struct moveArcActivityStatus moveArc;
        struct moveInstantActivityStatus moveInstant;
        struct moveToPositionActivityStatus moveToPosition;
        struct gearInPosActivityStatus gearInPos;
        struct gearInVeloActivityStatus gearInVelo;
        struct setDoutActivityStatus setDout;
        struct setDoutActivityStatus setExternalDout;
        struct setAoutActivityStatus setAout;
        struct setIoutActivityStatus setIout;
        struct setUioutActivityStatus setUiout;
        struct setIoutActivityStatus setExternalIout;
        struct setUioutActivityStatus setExternalUiout;
        struct dwellActivityStatus dwell;
        struct spindleActivityStatus spindle;
        struct stressTestActivityStatus stressTest;
    };
};

struct activityCommand {
    union {
        struct moveJointsActivityCommand moveJoints;
        struct moveJointsInterpolatedActivityCommand moveJointsInterpolated;
        struct moveJointsAtVelocityActivityCommand moveJointsAtVelocity;
        struct moveLineActivityCommand moveLine;
        struct moveVectorAtVelocityActivityCommand moveVectorAtVelocity;
        struct moveRotationAtVelocityActivityCommand moveRotationAtVelocity;
        struct moveArcActivityCommand moveArc;
        struct moveInstantActivityCommand moveInstant;
        struct moveToPositionActivityCommand moveToPosition;
        struct gearInPosActivityCommand gearInPos;
        struct gearInVeloActivityCommand gearInVelo;
        struct setDoutActivityCommand setDout;
        struct setDoutActivityCommand setExternalDout;
        struct setAoutActivityCommand setAout;
        struct setIoutActivityCommand setIout;
        struct setUioutActivityCommand setUiout;
        struct setIoutActivityCommand setExternalIout;
        struct setUioutActivityCommand setExternalUiout;
        struct dwellActivityCommand dwell;
        struct spindleActivityCommand spindle;
        struct stressTestActivityCommand stressTest;
    };
};

struct activityStreamItem {
        enum ACTIVITYTYPE activityType;
        uint32_t tag;
        struct triggerParams triggers[3];
    union {
        struct moveJointsStream moveJoints;
        struct moveJointsInterpolatedStream moveJointsInterpolated;
        struct moveJointsAtVelocityStream moveJointsAtVelocity;
        struct moveLineStream moveLine;
        struct moveVectorAtVelocityStream moveVectorAtVelocity;
        struct moveRotationAtVelocityStream moveRotationAtVelocity;
        struct moveArcStream moveArc;
        struct moveInstantStream moveInstant;
        struct moveToPositionStream moveToPosition;
        struct setDoutActivityParams setDout;
        struct setDoutActivityParams setExternalDout;
        struct setAoutActivityParams setAout;
        struct setIoutActivityParams setIout;
        struct setUioutActivityParams setUiout;
        struct setIoutActivityParams setExternalIout;
        struct setUioutActivityParams setExternalUiout;
        struct dwellActivityParams dwell;
        struct spindleActivityParams spindle;
        struct toolOffsetActivityParams setToolOffset;
        struct stressTestActivityStream stressTest;
    };
};

struct soloActivityConfig {
};

typedef struct activityStatus soloActivityStatus;

typedef struct activityStreamItem soloActivityCommand;

struct framesConfig {
        struct vector3 translation;
        struct quat rotation;
        uint16_t parentFrameIndex;
        enum POSITIONREFERENCE positionReference;
        uint8_t workspaceOffset;
};

struct framesCommand {
};

struct framesStatus {
};

struct pointsConfig {
        uint8_t frameIndex;
        struct vector3 translation;
        struct quat rotation;
        uint8_t configuration;
};

struct toolConfig {
        struct vector3 translation;
        struct quat rotation;
        double diameter;
};

struct serialConfig {
};

struct serialStatus {
        uint8_t statusWord;
        uint8_t length;
        uint8_t data[22];
};

struct serialCommand {
        uint8_t controlWord;
        uint8_t length;
        uint8_t data[22];
};


// OFFSETS
struct offsets {
    uint32_t addrCounts;
    uint32_t addrMachineConfig;
    uint32_t addrMachineStatus;
    uint32_t addrMachineCommand;


    uint32_t addrStreamConfig;
    uint32_t addrStreamStatus;
    uint32_t addrStreamCommand;


    uint32_t addrMoveParametersConfig;


    uint32_t addrJointConfig;
    uint32_t addrJointStatus;
    uint32_t addrJointCommand;


    uint32_t addrKinematicsConfigurationConfig;
    uint32_t addrKinematicsConfigurationStatus;
    uint32_t addrKinematicsConfigurationCommand;


    uint32_t addrDoutConfig;
    uint32_t addrDoutStatus;
    uint32_t addrDoutCommand;


    uint32_t addrSafetyDoutConfig;
    uint32_t addrSafetyDoutStatus;
    uint32_t addrSafetyDoutCommand;


    uint32_t addrExternalDoutConfig;
    uint32_t addrExternalDoutStatus;
    uint32_t addrExternalDoutCommand;


    uint32_t addrDinConfig;
    uint32_t addrDinStatus;
    uint32_t addrDinCommand;


    uint32_t addrSafetyDinConfig;
    uint32_t addrSafetyDinStatus;
    uint32_t addrSafetyDinCommand;


    uint32_t addrExternalDinConfig;
    uint32_t addrExternalDinStatus;
    uint32_t addrExternalDinCommand;


    uint32_t addrAoutConfig;
    uint32_t addrAoutStatus;
    uint32_t addrAoutCommand;


    uint32_t addrAinConfig;
    uint32_t addrAinStatus;
    uint32_t addrAinCommand;


    uint32_t addrIoutConfig;
    uint32_t addrIoutStatus;
    uint32_t addrIoutCommand;


    uint32_t addrUioutConfig;
    uint32_t addrUioutStatus;
    uint32_t addrUioutCommand;


    uint32_t addrExternalIoutConfig;
    uint32_t addrExternalIoutStatus;
    uint32_t addrExternalIoutCommand;


    uint32_t addrExternalUioutConfig;
    uint32_t addrExternalUioutStatus;
    uint32_t addrExternalUioutCommand;


    uint32_t addrIinConfig;
    uint32_t addrIinStatus;
    uint32_t addrIinCommand;


    uint32_t addrUiinConfig;
    uint32_t addrUiinStatus;
    uint32_t addrUiinCommand;


    uint32_t addrExternalIinConfig;
    uint32_t addrExternalIinStatus;
    uint32_t addrExternalIinCommand;


    uint32_t addrExternalUiinConfig;
    uint32_t addrExternalUiinStatus;
    uint32_t addrExternalUiinCommand;


    uint32_t addrSpindleConfig;


    uint32_t addrSoloActivityConfig;
    uint32_t addrSoloActivityStatus;
    uint32_t addrSoloActivityCommand;


    uint32_t addrToolConfig;


    uint32_t addrTaskConfig;
    uint32_t addrTaskStatus;
    uint32_t addrTaskCommand;


    uint32_t addrActivityConfig;
    uint32_t addrActivityStatus;
    uint32_t addrActivityCommand;


    uint32_t addrFramesConfig;
    uint32_t addrFramesStatus;
    uint32_t addrFramesCommand;


    uint32_t addrPointsConfig;


    uint32_t addrSerialConfig;
    uint32_t addrSerialStatus;
    uint32_t addrSerialCommand;


};

// COUNTS
struct counts {
            uint8_t numMachine;
            uint8_t numStream;
            uint16_t numMoveParameters;
            uint16_t numJoint;
            uint16_t numKinematicsConfiguration;
            uint16_t numDout;
            uint16_t numSafetyDout;
            uint16_t numExternalDout;
            uint16_t numDin;
            uint16_t numSafetyDin;
            uint16_t numExternalDin;
            uint16_t numAout;
            uint16_t numAin;
            uint16_t numIout;
            uint16_t numUiout;
            uint16_t numExternalIout;
            uint16_t numExternalUiout;
            uint16_t numIin;
            uint16_t numUiin;
            uint16_t numExternalIin;
            uint16_t numExternalUiin;
            uint16_t numSpindle;
            uint16_t numSoloActivity;
            uint16_t numTool;
            uint16_t numTask;
            uint16_t numActivity;
            uint16_t numFrames;
            uint16_t numPoints;
            uint8_t numSerial;
};

#pragma pack()

#endif