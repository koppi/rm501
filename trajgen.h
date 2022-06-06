/*******************************************************************************
 * @file trajgen.h
 *
 * Allows to move the machine tool tip to specified positions, velocities and
 * accelerations by modifying a virtual machine position every generator cycle.
 * The output is always an updated position for every joint. The closed loop
 * controllers of the joints follow this position or, if the cannot, raise a
 * trailing error fault.
 *
 * @author: Stefan Wilhelm (c erberos@atwillys.de)
 * @license: BSD (see below)
 * @version: 1.0b
 * @standard C99
 *
 * -----------------------------------------------------------------------------
 *
 * Coordinated motion (coord planer, manual) are related the planer pose (tool
 * tip position and direction), t.m. a new pose is calculated first, then the
 * joint position are determined using inverse kinematics. The joint planers
 * update the scalar position of every joint individually, the planer pose is
 * updated (for status or equivalence) using forward kinematics. After the new
 * joint positions are computed, they are used to feed the interpolators. The
 * output of the interpolatoes is the final output of the trajgen and can be
 * used as command position for the closed loop controllers.
 *
 * As wrapper for the different trajectory planers, the main trajgen organises
 * the initialisation, reset and synchronisation of the submodules. It is based
 * on a finite state machine. This means the generator can be only in  one defined
 * state, and switching between states causes leaving the current state (wait
 * for the current motion to be finished and cleanup after the sub modules), and
 * entering a new state (initialise and switch to module run state). Hence,
 * switching takes at least one tick (cycle period).
 *
 * Usage:
 *
 *  Variables:
 *
 *      All data of the trajgen are stored in one single data structure of the
 *      type trajgen_t. This structure contains the configuration, the current
 *      state, the (requested) state to switch to, the last error occurred,
 *      current pose, kinematic data, coordinated planer data, manual op data,
 *      as well as data for individual joints. Latter are joint data structures
 *      containing data of the interpolators, joint planers, and the
 *      joint command position outputs.
 *
 *      You can decide how and where this structure is located, either as part
 *      of a bigger data structure, global, local, stack, heap, etc. Only few
 *      aspects are important:
 *
 *          (1) The location must not change, as the pointer is saved in a
 *              static variable.
 *
 *          (2) There can be only one main trajgen (but multiple sub-planers
 *              e.g. for simulation purposes).
 *
 *  Synchronisation and threading
 *
 *      Thread safety is not implemented due to platform dependencies. You must
 *      take care yourself that no other thread or shared memory write process
 *      interferes with any function of the trajgen. It absolutely assumes that
 *      the data are not changed during a function call.
 *
 *  Configuration and initialisation:
 *
 *      The trajgen is configured once in the startup phase of the machine.
 *      Create a variable of type trajgen_config_t and pass it together with
 *      the pointer to the main trajgen structure to trajgen_initialize();
 *      The config will not be modified by any internal function.
 *
 *  Reset:
 *
 *      Calling trajgen_reset() will reset the internal state and internal
 *      variables of the trajgen and all submodules to a "clean" disabled state.
 *      The config will not be changed. The state will be 0 (TRAJ_STATE_DISABLED)
 *      and is_done will be 1 (true).
 *
 *  Cyclic function:
 *
 *      Call trajgen_tick() with the sample rate that the servo position
 *      need to be updated. The configuration variable "interpolation_rate"
 *      decides how frequently the trajectory needs to be recalculated. For a
 *      closed loop sample rate of 2KHz and an interpolation rate of 4, the
 *      trajectory will be recalculated with 500Hz, the missing cycles will be
 *      filled by the cubic interpolator of each joint. This means also that
 *      synchronous outputs and condition handling will be done with 500Hz.
 *
 *  State switching:
 *
 *      Use trajgen_switch_state() to switch between the submodules. After calling
 *      the function, is_done will be 0 and remain zero until the current planer
 *      stopped moving, is cleaned up and the new state is in running state.
 *      However, the state might remain zero if the newly selected generator
 *      started moving already.
 *
 *      States:
 *
 *          TRAJ_STATE_DISABLED:
 *
 *              The trajgen is idle and does not perform any calculations or
 *              position updates.
 *
 *          TRAJ_STATE_COORDINATED:
 *
 *              The coordinated motion planer is running and processes its
 *              queue.
 *
 *          TRAJ_STATE_MAN:
 *
 *              The (coordinated) teleoperation planer is running and processes
 *              according its given input velocities.
 *
 *          TRAJ_STATE_JOINT:
 *
 *              The joint individual planers are active.
 *
 *  Submodule access:
 *
 *      You have full access to the individual planners and get in touch with
 *      each you want to use if you define the export-enable macros described
 *      below.
 *
 *  Errors:
 *
 *      Each of the trajgen and submodule functions that can cause errors return
 *      an error code. The convention is: code 0 (zero) means OK, nonzero means
 *      error. You should always treat trajgen errors that are called cyclic in
 *      trajgen_coord_tick(), as well as initialisation errors, as fatal and cause
 *      an e-stop by opening the machine's intermediate circuit immediately.
 *
 *      Every generator has a variable last_error, where you can query the last
 *      error occurred. This values are only valid if a function returned nonzero
 *      before (the variable is not explicitly reset to 0 if a function returns
 *      OK). The main trajgen's last_error is a structure containing one single
 *      encoded 16bit error value (type trajgen_error_details_t). This error
 *      code contains the
 *
 *          (1) the code
 *          (2) the joint, which is only valid if an interpolator or joint planer
 *              failed.
 *
 *      This error code allows to identify quite accurately what went wrong using
 *      a single number that can be saved or passed to the user space easily.
 *
 * -----------------------------------------------------------------------------
 * License header: Copyright (c) 2008-2014, Stefan Wilhelm. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met: (1) Redistributions
 * of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer. (2) Redistributions in binary form must reproduce
 * the above copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the distribution.
 * (3) Neither the name of atwillys.de nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ******************************************************************************/
 
#ifndef SW_TRAJGEN_H
#define SW_TRAJGEN_H
#ifdef __cplusplus
extern "C" {
#endif
 
/* <editor-fold defaultstate="collapsed" desc="Build config: types and libraries"> */
 
#if defined __linux__
#include <stdint.h>
#include <sys/types.h>
#elif defined __MACH__
#include <stdint.h>
#include <sys/types.h>
#else
#error "Can you please define the rttypes for this platform and send a patch? :)"
/*
#define int8_t char
#define uint8_t unsigned char
#define int16_t short
#define uint16_t unsigned short
#define int32_t int
#define uint32_t unsigned int
#define int64_t long
#define uint64_t unsigned long
#define float32_t float
#define float64_t double
#define size_t unsigned int
 */
#endif
 
/**
 * Allows to specify 32 bit or 64 bot float, unfortunately "float_t" is already
 * reserved on many frames, so "real_t" is used instead. Can be also redefined
 * as typedef. The macro is used to be able to switch it easily using the pre
 * compiler.
 */
#ifndef real_t
#define real_t double
#endif
 
#ifndef float32_t
#define float32_t float
#endif
 
/**
 * Explicit declaration of a boolean type. Can be switched for memory optimizsation
 * (e.g. char) or best data type for the processor (e.g. int)
 */
#ifndef bool_t
#define bool_t unsigned
#endif
 
/**
 * Explicit declaration of a byte, no matter if signed or unsigned. Normally
 * used for bit operations or 8bit port i/o.
 */
#ifndef byte_t
#define byte_t uint8_t
#endif
 
/**
 * "Auto include" float.h
 * Note: This allows you to include float.h or a RT optimised one yourself.
 */
#ifndef FLT_MAX
#include <float.h>
#endif
 
/**
 * "Auto include" math.h, characteristic macro: M_PI
 * Note: This allows you to include math.h or a RT optimised one yourself.
 */
#ifndef M_PI
#include <math.h>
#endif
 
 
/*
 * Defines the maximum number of joints the program can handle.
 */
#ifndef JOINT_MAX_JOINTS
#define JOINT_MAX_JOINTS 9
#endif
 
/*
 * The maximum number of joints that the trajectory generator supports
 */
#ifndef TG_MAX_JOINTS
#define TG_MAX_JOINTS JOINT_MAX_JOINTS
#endif
 
/*
 * The maximum size if the coordinated motion generator queue (fixed allocated)
 * Note: This is an intermediate fifo with big data structures and should be small.
 * You are supposed to dispatch and push new values from a realtime fifo whenever
 * possible.
 */
#ifndef TG_QUEUE_SIZE
#define TG_QUEUE_SIZE 32
#endif
 
/*
 * Defines then threshold when the coordinated motion planer sees numbers as
 * zero or not.
 */
#ifndef TG_RESOLUTION
#define TG_RESOLUTION 1e-6
#endif
 
/*
 * Smallest floating point number that is not zero. (Epsilon)
 */
#ifndef FLOAT_EPS
#define FLOAT_EPS (sizeof(real_t) == sizeof(double) ? FLT_EPSILON : DBL_EPSILON)
#endif
 
/*
 * Spline/bezier iterations to find the length
 */
#ifndef TG_CURVE_ACC
#define TG_CURVE_ACC (16)
#endif
 
/*
 * Sin/cos, can be replaced platform dependent with the corresponding builtin.
 * #define SINCOS(x, sx, cx) do { *(sx)=sin((x)); *(cx)=cos((x)); } while(0);
 */
 
/*
 * You can set your own memory clear (set to 0) function. By default a static
 * inline function is used that is independent of any external library.
 * #define ZERO_MEMORY(ptr, size)
 */
 
/*
 * Define this to force the trajgen to check argument pointers
 * #define ARG_POINTER_CHECKS
 */
 
/*
 * Normally you like to define your own joint type and config for the machine,
 * but if you to it has to be compliant with the joint_t here.
 * #define JOINT_DECL_NONE
 */
 
/*
 * You can switch off the interpolator feature here. Any settings related to
 * it will be ignored. The interpolation rate must be set to 1.
 * #define NO_INTERPOLATION
 */
 
/*
 * You can switch off the trajectory position/velocity synchronisation using:
 * #define NO_TRAJECTORY_SYNC
 */
 
/*
 * You can optionally export the interpolators or leave them
 * inline static in the implementation file to improve code optimisation.
 * Interpolator types and error definitions are not affected by this setting,
 * they are always exported.
 * #define EXPORT_INTERPOLATORS
 */
 
/*
 * You can optionally not export the joint planers or leave them
 * inline static in the implementation file to improve code optimisation.
 * Types and error definitions are not affected by this setting, only functions.
 * #define EXPORT_JOINT_TG
 */
 
/*
 * You can optionally not export the manual/joystick generator or leave it
 * inline static in the implementation file to improve code optimisation.
 * Types and error definitions are not affected by this setting, only functions.
 * #define EXPORT_MANUAL_TG
 */
 
/*
 * You can optionally not export the coordinated motion generator or leave it
 * inline static in the implementation file to improve code optimisation.
 * Types and error definitions are not affected by this setting, only functions.
 * #define EXPORT_COORD_TG
 */
 
/*
 * You can switch off the interpolator feature here. Any settings related to
 * it will be ignored. The interpolation rate must be set to 1.
 * #define NO_MOTION_BLENDING
 */
 
/*
 * Defining this will enable stderr debug messages. Note: this can be quite
 * detailed. (command line option -DTG_DEBUG_LEVEL=<1,2,3>)
 * #define TG_DEBUG_LEVEL <0,1,2...> (default undefined)
 */
 
/*
 * Math "resolution", values define when a dimension or angle is treated
 * as zero.
 * #define TG_D_RES (default TG_RESOLUTION)
 * #define TG_A_RES (default TG_RESOLUTION)
 */
 
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="pose_*: Robot pose types and operations"> */
 
/********************************************************************
 *
 * Data structure defining a position of logical axes:
 *
 *  Translation : x,z,y
 *  Rotation    : a,b,c
 *  Auxiliary translation: u,v,w
 *
 ********************************************************************/
 
/* Axis enumerators for double vector access */
typedef enum
{
  POSE_X_AXIS = 0, POSE_Y_AXIS, POSE_Z_AXIS, POSE_A_AXIS, POSE_B_AXIS,
  POSE_C_AXIS, POSE_U_AXIS, POSE_V_AXIS, POSE_W_AXIS
} pose_axis_t;
 
/* Pose by axes */
typedef struct { real_t x, y, z, a, b, c, u, v, w; } pose_t;
typedef struct { real_t x, y, z; } pose_vector_t;
#define pose_position_t pose_vector_t
 
/* Line definition */
typedef struct
{
  pose_position_t start, end; /* Start/end point */
  pose_vector_t u;        /* Unit vector of translation */
  real_t tm;              /* Magnitude /length */
} pose_line_t;
 
/* Circle definition */
typedef struct
{
  pose_position_t cp;     /* Circle centre point */
  pose_vector_t nv;       /* Circle normal vector */
  pose_vector_t rt,rp,rh; /* radius vectors: tangent, perpendicular, helix */
  real_t l, r, a, s;      /* Length, start radius, angle, parallel component coeff */
} pose_circle_t;
 
/* Curve (bezier/spline) definition */
typedef struct
{
  #ifdef WITH_CURVES
  pose_position_t start,end; /* End point */
  pose_vector_t suv, euv;    /* Start/end unit vectors */
  real_t len;                /* Length of the segment */
  real_t c[3][4];            /* Path component coefficients */
  real_t d[TG_CURVE_ACC];    /* Correction */
  #endif
} pose_curve_t;
 
typedef real_t pose_array_t[9];
 
/**
 * Inliners (These macros are not upper case)
 * Using byval for sources and by pointer for dst, so if somthing is switched
 * the compiler will raise an error.
 */
/* Assignment p=0 */
#define pose_set_zero(p) { \
  (p)->x=(p)->y=(p)->z=(p)->a=(p)->b=(p)->c=(p)->u=(p)->v=(p)->w= 0.0; \
}
 
#define pose_set_all(p, val) { \
  (p)->x=(p)->y=(p)->z=(p)->a=(p)->b=(p)->c=(p)->u=(p)->v=(p)->w=val; \
}
 
/**
 * Checks if all elements are valid numbers (not nan) and finite.
 */
#define pose_isfinite(p) ( \
  isfinite((p).x) && isfinite((p).y) && isfinite((p).z) && \
  isfinite((p).a) && isfinite((p).b) && isfinite((p).c) && \
  isfinite((p).u) && isfinite((p).v) && isfinite((p).w) \
)
 
/* Assignment dst = src */
/* stfwi: no memcpy, let the compiler handle optimisation */
#define pose_set(dst, src) { \
  (dst)->x = (src).x; (dst)->y = (src).y; (dst)->z = (src).z; \
  (dst)->a = (src).a; (dst)->b = (src).b; (dst)->c = (src).c; \
  (dst)->u = (src).u; (dst)->v = (src).v; (dst)->w = (src).w; \
}
 
/* Difference po = p1-p2 */
#define pose_diff(p1, p2, po) { \
  (po)->x = (p1).x-(p2).x; (po)->y = (p1).y-(p2).y; (po)->z=(p1).z - (p2).z; \
  (po)->a = (p1).a-(p2).a; (po)->b = (p1).b-(p2).b; (po)->c=(p1).c - (p2).c; \
  (po)->u = (p1).u-(p2).u; (po)->v = (p1).v-(p2).v; (po)->w=(p1).w - (p2).w; \
}
 
/* Sum po = p1+p2 */
#define pose_sum(p1, p2, po){ \
  (po)->x = (p1).x+(p2).x; (po)->y = (p1).y+(p2).y; (po)->z = (p1).z+(p2).z; \
  (po)->a = (p1).a+(p2).a; (po)->b = (p1).b+(p2).b; (po)->c = (p1).c+(p2).c; \
  (po)->u = (p1).u+(p2).u; (po)->v = (p1).v+(p2).v; (po)->w = (p1).w+(p2).w; \
}
 
/* Accumulate po += p2 */
#define pose_acc(po, p2){ \
  (po)->x += (p2).x; (po)->y += (p2).y; (po)->z += (p2).z; \
  (po)->a += (p2).a; (po)->b += (p2).b; (po)->c += (p2).c; \
  (po)->u += (p2).u; (po)->v += (p2).v; (po)->w += (p2).w; \
}
 
/* Substract po -= p2 */
#define pose_sub(po, p2){ \
  (po)->x -= (p2).x; (po)->y -= (p2).y; (po)->z -= (p2).z; \
  (po)->a -= (p2).a; (po)->b -= (p2).b; (po)->c -= (p2).c; \
  (po)->u -= (p2).u; (po)->v -= (p2).v; (po)->w -= (p2).w; \
}
 
/* Negation po = -po */
#define pose_neg(po){ \
  (po)->x = -(po)->x; (po)->y = -(po)->y; (po)->z = -(po)->z; \
  (po)->a = -(po)->a; (po)->b = -(po)->b; (po)->c = -(po)->c; \
  (po)->u = -(po)->u; (po)->v = -(po)->v; (po)->w = -(po)->w; \
}
 
/* Scalar multiply po *= (double) d */
#define pose_scale(po, d){ \
  (po)->x *= (d); (po)->y *= (d); (po)->z *= (d); \
  (po)->a *= (d); (po)->b *= (d); (po)->c *= (d); \
  (po)->u *= (d); (po)->v *= (d); (po)->w *= (d); \
}
 
/* Trim every axis to a maximum value (defined for every axis) */
#define pose_trim_all_upper(po, max) { \
  if ((po)->x > (max).x) (po)->x = (max).x; \
  if ((po)->y > (max).y) (po)->y = (max).y; \
  if ((po)->z > (max).z) (po)->z = (max).z; \
  if ((po)->a > (max).a) (po)->a = (max).a; \
  if ((po)->b > (max).b) (po)->b = (max).b; \
  if ((po)->c > (max).c) (po)->c = (max).c; \
  if ((po)->u > (max).u) (po)->u = (max).u; \
  if ((po)->v > (max).v) (po)->v = (max).v; \
  if ((po)->w > (max).w) (po)->w = (max).w; \
}
 
/* Trim every axis to a minimum value (defined for every axis) */
#define pose_trim_all_lower(po, min) { \
  if ((po)->x < (min).x) (po)->x = (min).x; \
  if ((po)->y < (min).y) (po)->y = (min).y; \
  if ((po)->z < (min).z) (po)->z = (min).z; \
  if ((po)->a < (min).a) (po)->a = (min).a; \
  if ((po)->b < (min).b) (po)->b = (min).b; \
  if ((po)->c < (min).c) (po)->c = (min).c; \
  if ((po)->u < (min).u) (po)->u = (min).u; \
  if ((po)->v < (min).v) (po)->v = (min).v; \
  if ((po)->w < (min).w) (po)->w = (min).w; \
}
 
/* Scalar multiply po *= (double) d */
/* StfWi: memcmp() with reference 0[9]? */
#define pose_is_zero(p) ( \
  ((p).x == 0.0 && (p).y == 0.0 && (p).z == 0.0 && \
  (p).a == 0.0 && (p).b == 0.0 && (p).c == 0.0 && (p).u == 0.0 && \
  (p).v == 0.0 && (p).w == 0.0) \
)
 
/* Scalar multiply po *= (double) d */
#define pose_is_all_greater_equal_zero(p) ( \
  ((p).x >= 0.0 && (p).y >= 0.0 && \
  (p).z >= 0.0 && (p).a >= 0.0 && (p).b >= 0.0 && (p).c >= 0.0 && \
  (p).u >= 0.0 && (p).v >= 0.0 && (p).w >= 0.0) \
)
 
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="joint_*: Robot joint decls"> */
 
/*******************************************************************************
 *
 * Represents one joint of the machine, including configuration, status and
 * command transfer.
 *
 ******************************************************************************/
 
#ifdef JOINT_DECL_NONE
/*
 * NOTE: You have to define the joints compatible to the minimum settings
 */
#elif defined JOINT_DECL_FULL
 
/**
 * Type of joint
 */
typedef enum
{
  JOINT_TYPE_NONE = 0,    /* Not defined */
  JOINT_AXIS_TYPE_LINEAR, /* Linear axis, unit will be meters */
  JOINT_AXIS_TYPE_ROTARY  /* Polar axis, unit will be degrees */
} joint_axis_type_t;
 
/**
 * Feedback type of a joint
 */
typedef enum
{
  /* E.g. stepper without encoder */
  JOINT_POSITION_FEEDBACK_NONE = 0,
  /* Incremental encoder, requires homing for all except axis free motion */
  JOINT_POSITION_FEEDBACK_INCREMENTAL,
  /* Incremental encoder, requires no homing */
  JOINT_POSITION_FEEDBACK_ABSOLUTE
} joint_position_feedback_t;
 
/**
 * I/O logic of various joint inputs/outputs, e.g. limit switches, index ...
 */
typedef enum
{
  JOINT_LOGIC_ACTIVE_HIGH = 0x00,
  JOINT_LOGIC_ACTIVE_LOW  = 0x01, /* used with xor */
  JOINT_LOGIC_DISABLED
} joint_logic_t;
 
/**
 * Bit field of joint-related
 */
typedef union
{
  uint16_t all;
  struct
  {
    /* The joint/axis is enabled */
    unsigned enabled : 1;
    /* The amplifier is enabled */
    unsigned amplifier_enabled : 1;
    /* The joint is homed */
    unsigned homed : 1;
    /* The trajectory planer is at the goal position */
    unsigned trajectory_done : 1;
    /* The joint is in the position tolerance of the goal position */
    unsigned in_position : 1;
    /* The joint does not move (in standstill velocity tolerance) */
    unsigned standstill : 1;
    /* The joint amplifier has an error */
    unsigned amplifier_fault : 1;
    /* The joint encoder has an error */
    unsigned encoder_fault : 1;
    /* The home switch is activated (also depends on config) */
    unsigned home_index : 1;
    /* The positive limit switch is activated (depends also on config) */
    unsigned positive_limit : 1;
    /* The negative limit switch is activated (depends also on config) */
    unsigned negative_limit : 1;
    /* Fill up remaining bits */
    unsigned reserved : 5;
  } bits;
} joint_status_t;
 
typedef struct
{
  /* Joint type, linear or polar/rotary */
  joint_axis_type_t type;
  /* The type of feedback */
  joint_position_feedback_t feedback_type;
  /* The positive software limit in meters/degrees */
  real_t positive_software_limit;
  /* The negative software limit in meters/degrees */
  real_t negative_software_limit;
  /* Scales the axis position (normally increments) to meters/degrees */
  real_t position_to_meters_scaler;
  /* The maximum velocity the joint can do in m/s / degrees/s */
  real_t max_velocity;
  /* The maximum acceleration the joint can do in m/s / degrees/s */
  real_t max_acceleration;
  /* Max trailing error at speed=0 in meters/degrees */
  real_t max_trailing_error_min;
  /* Max trailing error at speed=max_velocity in meters/degrees */
  real_t max_trailing_error_max;
  /* The deceleration used for ESTOP full break. It can exceed the max_acceleration. */
  real_t estop_deceleration;
  /* Max position deviation at standstill to say the measured joint position is */
  /* accurate enough, in m/s / degrees/s */
  real_t in_position_tolerance;
  /* Max speed deviation at standstill to say the joint does not move anymore, */
  /* m/s / degrees/s */
  real_t standstill_tolerance;
  /* 1 = a limit switch exists and has to be respected */
  joint_logic_t positive_hardware_limit;
  /* 1 = a limit switch exists and has to be respected */
  joint_logic_t negative_hardware_limit;
  /* If not, a limit switch will be used OR, if not limit switch, the mechanical */
  /* joint endpoint (measured using trailing error) */
  joint_logic_t home_index;
} joint_config_t;
 
typedef struct
{
  /* Axis configuration */
  joint_config_t config;
  /* State of the joint, see joint_state_t */
  joint_status_t status;
  /* The interpolated position of the trajectory generator (for all kinds of trajgen) */
  real_t command_position;
  /* The interpolated velocity of the trajectory generator (for all kinds of trajgen) */
  real_t command_velocity;
  /* The interpolated acceleration of the trajectory generator (for all kinds of trajgen) */
  real_t command_acceleration;
  /* The scaled position measured with the encoders */
  real_t position;
  /* Position captured by a latch operation, e.g. homing index */
  real_t latched_position;
} joint_t;
 
#else
 
/**
 * Minimum joint definition required to use the trajgen
 */
typedef struct
{
  struct { real_t max_velocity, max_acceleration; } config;
  real_t command_position, command_velocity, command_acceleration, position;
} joint_t;
 
#endif
 
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="trajgen_error_*: Trajgen error main decls"> */
 
/*******************************************************************************
 *
 *
 * Main trajectory generator error types and definitions.
 *
 *
 ******************************************************************************/
 
/**
 * Error declarations for the trajgen itself. However, because this is the "master"
 * File of the generator, it will return error codes defined in trajgen_error.h,
 * which includes joint, code and error source.
 */
enum
{
  /* Main controller */
  TRAJ_ERROR_OK = 0,
  TRAJ_ERROR_ERROR,
  TRAJ_ERROR_NULL_POINTER,
  TRAJ_ERROR_NUMERIC,
  TRAJ_ERROR_CONFIG_LAST_USED_JOINT_INVALID,
  TRAJ_ERROR_CONFIG_INTERPOLATION_RATE_INVALID,
  TRAJ_ERROR_CONFIG_OVERRIDE_INVALID,
  TRAJ_ERROR_CONFIG_COMPILE_SETTING_INVALID,
  TRAJ_ERROR_INVALID_STATE, /* The motion state is not valid (Internal problem) */
  TRAJ_ERROR_INVALID_SWITCHING_STATE, /* The operating mode to set is invalid */
  TRAJ_ERROR_INVALID_JOINT_NO,
 
  /* Kinematics */
  KINEMATICS_ERR_ERROR,
  KINEMATICS_ERR_NULL_POINTER,
  KINEMATICS_ERR_INIT_FUNCTION_NULL,
  KINEMATICS_ERR_FORWARD_FAILED,
  KINEMATICS_ERR_INVERSE_FAILED,
  KINEMATICS_ERR_RESET_FAILED,
 
  /* Interpolators */
  INTERPOLATOR_ERROR,
  INTERPOLATOR_ERROR_INIT_ARG_INVALID,
  INTERPOLATOR_ERROR_QUEUE_FULL,
  INTERPOLATOR_ERROR_OFFSET_IP_NULLPOINTER,
  INTERPOLATOR_ERROR_INTERPOLATE_ARG_NULLPOINTER,
  INTERPOLATOR_ERROR_NOT_RESET,
  INTERPOLATOR_ERROR_NULLPOINTER,
 
  /* Coordinated planer */
  TP_ERR_ERROR,
  TP_ERR_TP_NULL_POINTER,
  TP_ERR_ABORTING,
  TP_ERR_QUEUE_PUT_FAILED,
  TP_ERR_INVALID_PARAM,
  TP_ERR_QUEUE_FULL,
  TP_ERR_QUEUE_TO_MANY_ELEMENTS_TO_REMOVE,
  TP_ERR_INVALID_MOTION_TYPE,
  TP_ERR_INVALID_SPEED,
  TP_ERR_INVALID_ACCEL,
  TP_ERR_INVALID_POSE,
  TP_ERR_SEGMENT_LENGTH_ZERO,
  TP_ERR_INVALID_SAMPLE_INTERVAL,
  TP_ERR_ALREADY_MOVING,
  TP_ERR_UNIT_VECTOR_CALC_INVALID_TYPE,
  TP_ERR_REF_POSITION_INVALIDATED_DURING_MOTION,
 
  /* Joint planers */
  TRAJGEN_FREE_ERROR_ERROR,
  TRAJGEN_FREE_ERROR_INIT_NULLPOINTER,
  TRAJGEN_FREE_ERROR_INIT_INVALID_MAX_ACCEL,
  TRAJGEN_FREE_ERROR_INIT_INVALID_MAX_VELOCITY,
  TRAJGEN_FREE_ERROR_INIT_INVALID_SAMPLE_INTERVAL,
  TRAJGEN_FREE_ERROR_INVALID_ACCELERATION,
 
  /* Manual operation planer */
  TG_MAN_ERROR_ERROR,
  TG_MAN_ERROR_INIT_NULLPOINTER,
  TG_MAN_ERROR_INIT_INVALID_MAX_ACCEL,
  TG_MAN_ERROR_INIT_INVALID_MAX_VELOCITY,
  TG_MAN_ERROR_INIT_INVALID_SAMPLE_INTERVAL,
  TG_MAN_ERROR_INVALID_ACCELERATION
};
 
enum { KINEMATICS_ERR_OK=0 };
enum { INTERPOLATOR_OK=0 };
enum { TP_ERR_OK=0 };
enum { TRAJGEN_FREE_ERROR_OK=0 };
enum { TG_MAN_ERROR_OK=0 };
 
 
typedef uint16_t trajgen_error_t;
#define kinematics_error_t trajgen_error_t
#define trajgen_coord_error_t trajgen_error_t
#define interpolator_error_t trajgen_error_t
#define interpolator_error_t trajgen_error_t
#define trajgen_jointtg_error_t trajgen_error_t
#define trajgen_man_error_t trajgen_error_t
 
/**
 * Main error type
 */
typedef union
{
  int16_t errnom;
  struct
  {
    unsigned code : 12;
    unsigned joint : 4;
  } sel;
} trajgen_error_details_t;
 
/**
 * Writes a text version of a trajgen error into *errstr
 * @param uint16_t errnum
 * @param char* errstr
 * @param unsigned length
 */
extern const char* trajgen_errstr(uint16_t errnum, char* errstr, unsigned length);
 
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="interpolator_*: Interpolation"> */
 
/********************************************************************
 
 * Cyclic Cubic interpolation, used to interpolate joint command position for
 * the closed loop controllers between the way points produced by the trajectory
 * generators. This allows to enhance the system performance, especially when
 * high filter sample rates are required.
 *
 ********************************************************************/
 
/**
 * Cubic interpolator
 */
typedef struct
{
  uint16_t n;           /* Specifies number of queued points */
  real_t T;             /* The trajectory planer sample time */
  real_t Ti;            /* The time between two interpolated points */
  real_t t;             /* Current time, reset on push */
  real_t x0, x1, x2, x3;/* Buffer for the interpolation, used like a FIFO */
  real_t w0, w1;        /* Way points */
  real_t v0, v1;        /* Velocities at the way points */
  real_t a, b, c, d;    /* Coefficients polynomial, a*x^3 + b*x^2 + c*x + d */
} interpolator_t;
 
 
#ifdef EXPORT_INTERPOLATORS
/**
 * Initialise the interpolator.
 */
extern interpolator_error_t interpolator_init(interpolator_t*, real_t sample_interval,
    unsigned interpolation_rate);
 
/**
 * Reset the interpolator, clear the queue to the specified position value.
 */
extern interpolator_error_t interpolator_reset(interpolator_t*, real_t initial_position);
/**
 * Performs one interpolation tick. All pointer argument must refer to existing
 * variables (not NULL).
 */
extern interpolator_error_t interpolator_interpolate(interpolator_t*, real_t *x,
    real_t *v, real_t *a, real_t *j);
/**
 * Push a new position into the fifo. That is only allowed when the fifo is not full.
 */
extern interpolator_error_t interpolator_push(interpolator_t*, real_t point);
/**
 * Returns nonzero if the fifo is not full. You must push a value before the
 * next interpolation step, or the last value in the fifo will be pushed
 * automatically.
 */
extern bool_t interpolator_need_update(interpolator_t*);
 
#endif
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="kinematics_*: Kinematics"> */
 
/*******************************************************************************
 *
 * Calculates the position and direction of the end effector (where the tool is
 * mounted) in three dimensional space based in the position of all relevant
 * machine joints and vice versa. Several kinematic machine models are available
 * and implementing own models is possible.
 *
 ******************************************************************************/
 
typedef struct
{
  /**
   * The reset kinematics function sets all its arguments to their proper
   * values at the known initial position. When called, these should be set,
   * when known, to initial values, e.g., from an INI file. If the home
   * kinematics can accept arbitrary starting points, these initial values
   * should be used.
   */
  kinematics_error_t(*reset)(pose_t *pose, real_t *joint_positions[]);
 
  /**
   * The forward kinematics take joint values and determine world coordinates,
   * given forward kinematics flags to resolve any ambiguities. The inverse
   * flags are set to indicate their value appropriate to the joint values
   * passed in.
   */
  kinematics_error_t(*forward)(real_t *joint_positions[], pose_t *pose);
 
  /**
   * The inverse kinematics take world coordinates and determine joint values,
   * given the inverse kinematics flags to resolve any ambiguities. The forward
   * are set to indicate their value appropriate to the world coordinates
   * passed in.
   */
  kinematics_error_t(*inverse)(pose_t *pose, real_t *joint_positions[]);
 
} kinematics_t;
 
extern kinematics_error_t kinematics_initialize(kinematics_t *kin, kinematics_t
    device_kinematics);
 
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="trajgen_coord_*, trajgen_sg_*: Coordinated Motion"> */
 
/********************************************************************
 *
 * Planer for coordinated queued motion with blending, i/o synchronisation and
 * motion synchronisation.
 *
 ********************************************************************/
 
/**
 * Naming of motion task types (typedef is int'ed for bit packing)
 */
#define trajgen_sg_motion_type_t int
enum
{
  SG_INVALID  = 0x00,
  SG_LINEAR   = 0x01,
  SG_CIRCULAR = 0x02,
  SG_CURVE    = 0x03,
  SG_END_OF_MOTION
};
 
/**
 * Type naming of motion synchronisation (typedef is int'ed for bit packing)
 */
#define trajgen_coord_motion_sync_type_t int
enum
{
  TP_SYNC_NONE = 0x00,
  TP_SYNC_VELOCITY = 0x01,
  TP_SYNC_POSITION = 0x02
};
 
/**
 * Type for specification of synchronised motion (motion dependent on external
 * positions/velocities).
 */
typedef struct
{
  /**
   * Scales from the reference position/velocity (e.g. spindle position/speed)
   * to the scalar position/velocity of the trajectory when multiplied.
   */
  real_t scaler;
  /*
   * Position sync: The current (input) reference position
   * Velocity sync: The current (input) reference velocity
   */
  real_t reference;
  /* INTERNAL USE: Accumulates the position-synchronized segment lengths of
   * all surpassed segments for position sync.
   */
  real_t i_offset;
  /**
   * The type of synchronisation, for subsequent motion, default is TP_SYNC_NONE
   */
  trajgen_coord_motion_sync_type_t type: 4;
  /**
   * INTERNAL USE: Saves if the position offset does not require to be updated
   * before starting a position synced motion.
   */
  bool_t i_isset: 1;
} trajgen_coord_motion_sync_t;
 
/**
 * Coordinated planer configuration
 */
typedef struct
{
  /* Sample interval (cycle interval time) */
  real_t sample_interval;
  /* Max velocity allowed by machine constraints (normally set in a config file) */
  real_t max_velocity;
  /* Max acceleration allowed by machine constraints (normally set in a config file) */
  real_t max_acceleration;
} trajgen_coord_config_t;
 
/**
 * Mask type for synchronised digital I/O states (bit field)
 */
typedef uint16_t trajgen_coord_syncdio_t;
 
/**
 * Synchronised digital I/Os to set/reset when the next queued task is shifted.
 */
typedef struct
{
  /* the bits to be set to HIGH */
  trajgen_coord_syncdio_t set;
  /* the bits to be set to LOW */
  trajgen_coord_syncdio_t clear;
} trajgen_coord_syncdio_cmd_t;
 
/**
 * Task data specialisation for line motion
 */
typedef struct
{
  pose_line_t xyz, abc, uvw;
} trajgen_sg_line_t;
 
/**
 * Task data specialisation for circle motion
 */
typedef struct
{
  pose_circle_t xyz;
  pose_line_t abc, uvw;
} trajgen_sg_arc_t;
 
/**
 * Task data specialisation for curve motion
 */
typedef struct
{
  #ifdef WITH_CURVES
  pose_curve_t xyz;
  #else
  pose_line_t xyz;
  #endif
  pose_line_t abc, uvw;
} trajgen_sg_curve_t;
 
/**
 * Coordinated planer task task type, used in the queue.
 */
typedef struct
{
  union {
    trajgen_sg_line_t line;     /* Specific line data */
    trajgen_sg_arc_t arc;       /* Specific arc/circle/helix data */
    trajgen_sg_curve_t curve;  /* Specific curve/nurbs data */
  } coords;
 
  /**
   * segment's serial number
   */
  uint32_t id;
  /**
   * The scalar path length that the pose moved already in this segment, values
   * are from 0..target
   */
  real_t progress;
  /**
   * Scalar length of the path segment
   */
  real_t length;
  /**
   * Speed set by the user speed (requested speed)
   */
  real_t v;
  /**
   * Acceleration set by the user (requested acceleration)
   */
  real_t a;
  /**
   * Internally calculated (keep track of current step (vel * cycle_time) )
   */
  real_t current_velocity;
  /**
   * during the blend at the end of this move, stay within this distance from
   * the path
   */
  real_t blending_tolerance;
  /**
   * velocity below which we should start blending
   */
  real_t blending_velocity;
  /**
   * synched DIO's for this move. what to turn on/off
   */
  trajgen_coord_syncdio_cmd_t sync_dio;
  /**
   *  SG_LINEAR (coords.line) or
   */
  trajgen_sg_motion_type_t motion_type: 4;
  /**
   * The type of synchronisation, default is TP_SYNC_NONE
   */
  trajgen_coord_motion_sync_type_t sync_type: 4;
  /**
   * this motion is being executed
   */
  bool_t is_active: 1;
  /**
   * segment is being blended into following segment
   */
  bool_t is_blending: 1;
  /**
   * Feed scale, etc, enable bits for this move
   */
  bool_t override_enabled: 1;
} trajgen_sg_t;
 
typedef struct
{
  /* ptr to the tcs */
  trajgen_sg_t queue[TG_QUEUE_SIZE];
  /* size of queue */
  uint16_t size;
  /* number of tcs now in queue */
  uint16_t num_elements;
  /* indices to next to get, next to put */
  uint16_t start, end;
} trajgen_queue_t;
 
typedef struct
{
  real_t override;
  pose_t last_position;
} trajgen_coord_internals_t;
 
typedef struct
{
  /* Static configuration, set during initialisation */
  trajgen_coord_config_t config;
  /* Last error that occurred, only valid if a function does not return 0 */
  trajgen_coord_error_t last_error;
  /* The buffer containing queued segments */
  trajgen_queue_t queue;
  /* Defines for subsequent motion where spindle synchronisation is required */
  trajgen_coord_motion_sync_t sync;
  /* Will be added to (only) the next pushed segments and transferred to
   * sync_dio_current when the elements are processed */
  trajgen_coord_syncdio_cmd_t sync_dio_command;
  /* Contains the actual digital i/o word that has to be transferred to the HAL
   * or hardware port register */
  trajgen_coord_syncdio_t sync_dio_current;
  /* Internally used current values, DO NOT MODIFY */
  trajgen_coord_internals_t _i;
  /* number of motions blending */
  uint16_t num_queued_active;
  /* Id of the actual queued segment, can be used to identify the program line
   * in gnc programs */
  uint32_t sg_id;
  /* Id of the next segment that is pushed into the queue */
  uint32_t next_sg_id;
  /* Subsequent motions: stay within this distance of the programmed path during
   * blends */
  real_t blending_tolerance;
  /* Actual position/pose of the trajectory */
  pose_t pose;
  /* End position of the last queued segment */
  pose_t end_pose;
  /* Velocity scaling, normal value is 1.0 (means 100% = specified speed) */
  real_t override;
  /* Abort in progress */
  bool_t is_aborting;
  /* Motion paused */
  bool_t is_pausing;
  /* Queue empty and goal position reached */
  bool_t is_done;
  /* Subsequent motion: Flags defining which speed overrides are allowed */
  uint8_t override_enabled;
} trajgen_coord_t;
 
#ifdef EXPORT_COORD_TG
/**
 * Initialise the planer
 */
extern trajgen_coord_error_t trajgen_coord_init(trajgen_coord_t*, trajgen_coord_config_t config);
/**
 * Reset all mutable states, clear queue, but not the config.
 */
extern trajgen_coord_error_t trajgen_coord_reset(trajgen_coord_t*);
/**
 * Set the current pose. Use this for initialising or if axes moved manually.
 */
extern trajgen_coord_error_t trajgen_coord_set_position(trajgen_coord_t*, pose_t pos);
/**
 * Move a line
 */
extern trajgen_coord_error_t trajgen_coord_add_line(trajgen_coord_t*, pose_t end,
    real_t velocity, real_t acceleration);
/**
 * Move a circle
 */
extern trajgen_coord_error_t trajgen_coord_add_arc(trajgen_coord_t*, pose_t end,
    pose_position_t center, pose_vector_t normal, unsigned n_turns, real_t velocity,
    real_t acceleration);
/**
 * Move a curve/nurbs segment
 */
extern trajgen_coord_error_t trajgen_coord_add_curve(trajgen_coord_t* tp,
    pose_t end, pose_vector_t start_ctrl_point, pose_vector_t end_ctrl_point,
    real_t velocity, real_t acceleration);
/**
 * Pause motion
 */
extern trajgen_coord_error_t trajgen_coord_pause(trajgen_coord_t*);
/**
 * Resume from pause
 */
extern trajgen_coord_error_t trajgen_coord_resume(trajgen_coord_t*);
/**
 * Stop motion, clear the queue
 */
extern trajgen_coord_error_t trajgen_coord_abort(trajgen_coord_t*);
/**
 * Tick: has to be called with the configured sample frequency.
 */
extern trajgen_coord_error_t trajgen_coord_tick(trajgen_coord_t*);
/**
 * Returns nonzero if the queue is full, so you have to wait until pushing
 * next motion tasks.
 */
extern bool_t trajgen_coord_is_queue_full(trajgen_coord_t*);
/**
 * Returns nonzero if all motion has finished and the planer pose reached the
 * destination position.
 */
extern bool_t trajgen_coord_is_done(trajgen_coord_t*);
/**
 * Returns the current pose
 */
extern pose_t trajgen_coord_get_position(trajgen_coord_t*);
/**
 * Returns the ID of the actually processed task
 */
extern uint32_t trajgen_coord_get_current_id(trajgen_coord_t*);
/**
 * Returns the number of currently queued tasks.
 */
extern uint16_t trajgen_coord_get_num_queued(trajgen_coord_t*);
/**
 * Returns maximum size of the queue
 */
extern uint16_t trajgen_coord_get_queue_size(trajgen_coord_t*);
/**
 * Returns returns the number of currently processed tasks. That can be greater
 * one e.g. when blending.
 */
extern uint16_t trajgen_coord_get_num_queued_active(trajgen_coord_t*);
 
#endif
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="trajgen_jointtg_*: Separate joint motion"> */
 
/********************************************************************
 *
 * Simple trajectory planner for one joint. When enabled, corrects the joint
 * position based on a given command position and command velocity (maximum
 * velocity when override == 1.0).
 *
 * Can be used for absolute motion, relative motion, position correction (
 * (e.g. mouse or jogwheel).
 *
 ********************************************************************/
 
typedef struct
{
  /* Maximum acceleration of this joint for the joint trajgen */
  real_t max_acceleration;
  /* Maximim positive and negative velocity */
  real_t max_velocity;
  /* Sample interval, seconds */
  real_t sample_interval;
} trajgen_jointtg_config_t;
 
typedef struct
{
  /* The last error that occurred, only valid if a function returns nonzero */
  trajgen_jointtg_error_t last_error;
  /* Configuration */
  trajgen_jointtg_config_t config;
  /* Internal actual speed */
  real_t velocity;
  /* Actual position (also returned by trajgen_jointtg_get_position()) */
  real_t position;
  /* The position where the tp shall move to */
  real_t command_position;
  /* The maximum velocity of this move */
  real_t command_velocity;
  /* Enabled flag */
  bool_t is_enabled;
  /* Pause flag */
  bool_t is_pause;
  /* Done flag */
  bool_t is_done;
} trajgen_jointtg_t;
 
#ifdef EXPORT_JOINT_TG
 
/**
 * Initializes the joint generator
 * @param trajgen_jointtg_t *tp
 * @param trajgen_jointtg_config_t config
 * @return trajgen_jointtg_error_t
 */
extern trajgen_jointtg_error_t trajgen_jointtg_initialize(trajgen_jointtg_t *tp,
    trajgen_jointtg_config_t config);
 
/**
 * Resets the manual operation generator except the configuration.
 * @return trajgen_jointtg_error_t
 */
extern trajgen_jointtg_error_t trajgen_jointtg_reset(trajgen_jointtg_t *tp);
 
/**
 * Aborts the motion (force velocity to 0.0)
 * @return
 */
extern trajgen_jointtg_error_t trajgen_jointtg_set_enabled(trajgen_jointtg_t *tp,
    bool_t enabled);
 
/**
 * Input a new command position for the joint
 * @return trajgen_jointtg_error_t
 */
extern trajgen_jointtg_error_t trajgen_jointtg_set_command_position(trajgen_jointtg_t *tp,
    real_t position);
 
/**
 * Input a new velocity for the joint
 * @return trajgen_jointtg_error_t
 */
extern trajgen_jointtg_error_t trajgen_jointtg_set_command_velocity(trajgen_jointtg_t *tp,
    real_t velocity);
 
/**
 * Returns the last generated pose (output)
 * @return pose_t
 */
extern real_t trajgen_jointtg_get_current_position(trajgen_jointtg_t *tp);
 
/**
 * Sets the internal position. Used to update when the amplifiers/closed loop
 * controls are off
 * @param pose_t position
 * @return pose_t
 */
extern trajgen_jointtg_error_t trajgen_jointtg_set_current_position(trajgen_jointtg_t *tp,
    real_t position);
 
/**
 * Calculates a new position from the command velocity
 * @return pose_t
 */
extern trajgen_jointtg_error_t trajgen_jointtg_tick(trajgen_jointtg_t *tp);
 
/**
 * Returns true (nonzero) if the command velocity and the actual total velocity
 * is zero.
 * @return bool_t
 */
extern bool_t trajgen_jointtg_is_done(trajgen_jointtg_t *tp);
 
#endif
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="trajgen_man_*: Remote/manually operated motion"> */
 
/********************************************************************
 *
 * Trajectory generator coordinated motion based on input velocities
 * (e.g. Joystick). Output is a pose, which has to be converted to the individual
 * joint positions using inverse kinematics.
 *
 ********************************************************************/
 
typedef struct
{
  /* Maximum positive and negative velocity for all axes */
  pose_t max_velocity;
  /* Maximum acceleration for each axis */
  pose_t max_acceleration;
  /* Sample interval, seconds */
  real_t sample_interval;
} trajgen_man_config_t;
 
typedef struct
{
  /* The last error that occurred, only valid if a function returns nonzero */
  trajgen_man_error_t last_error;
  /* Configuration, used to init */
  trajgen_man_config_t config;
  /* The (input) velocities that the endeffector/axes shall move with */
  pose_t velocity;
  /* Internal actual speed */
  pose_t current_velocity;
  /* Actual position (also returned by trajgen_man_get_position()) */
  pose_t pose;
  /* Enabled flag */
  bool_t is_enabled;
} trajgen_man_t;
 
#ifdef EXPORT_MANUAL_TG
/**
 * Initializes the manual operation generator
 * @param trajgen_man_t *tp
 * @param trajgen_man_config_t config
 * @return trajgen_man_error_t
 */
extern trajgen_man_error_t trajgen_man_initialize(trajgen_man_t *mtp, trajgen_man_t *_tp,
    trajgen_man_config_t config);
 
/**
 * Resets the manual operation generator except the configuration.
 * @return trajgen_man_error_t
 */
extern trajgen_man_error_t trajgen_man_reset(trajgen_man_t *mtp);
 
/**
 * Aborts the motion (force velocity to 0.0)
 * @return
 */
extern trajgen_man_error_t trajgen_man_abort(trajgen_man_t *mtp);
 
/**
 * Input a new velocity for every axis
 * @return trajgen_man_error_t
 */
extern trajgen_man_error_t trajgen_man_set_velocity(trajgen_man_t *mtp, pose_t velocity);
 
/**
 * Returns the last generated pose (output)
 * @return pose_t
 */
extern pose_t trajgen_man_get_position(trajgen_man_t *mtp);
 
/**
 * Sets the internal position. Used to update when the amplifiers/closed loop
 * controls are off
 * @param pose_t position
 * @return pose_t
 */
extern trajgen_man_error_t trajgen_man_set_position(trajgen_man_t *mtp, pose_t position);
 
/**
 * Calculates a new position from the command velocity
 * @return pose_t
 */
extern trajgen_man_error_t trajgen_man_tick(trajgen_man_t *mtp);
 
/**
 * Returns true (nonzero) if the command velocity and the actual total velocity
 * is zero.
 * @return bool_t
 */
extern bool_t trajgen_man_is_done(trajgen_man_t *mtp);
 
#endif
/* </editor-fold> */
 
/* <editor-fold defaultstate="collapsed" desc="trajgen_*: Main trajectory generator"> */
 
/**
 * Operating mode state, including switching between modes
 */
typedef enum
{
  TRAJ_STATE_DISABLED = 0,  /* Controller disabled, implicitly stops all motion */
  TRAJ_STATE_JOINT,         /* Free joint motion */
  TRAJ_STATE_COORDINATED,   /* Coordinated motion, requires homing */
  TRAJ_STATE_MAN,           /* Manual operation, requires homing */
  TRAJ_______INTERNAL_STATES,
  TRAJ_STATE_OK_TO_SWITCH,
  TRAJ_STATE_DISABLING,
  TRAJ_STATE_JOINT_ENTER,
  TRAJ_STATE_JOINT_LEAVE,
  TRAJ_STATE_COORDINATED_ENTER,
  TRAJ_STATE_COORDINATED_LEAVE,
  TRAJ_STATE_TG_MAN_ENTER,
  TRAJ_STATE_TG_MAN_LEAVE
} trajgen_state_t;
 
/**
 * Configuration
 */
typedef struct
{
  /* Own machine kinematics or NULL for "identity" (x=x,y=y, ...) */
  kinematics_t kinematics_functions;
  /* Max (axes, not joints) accelerations for manual operation, but corresponds */
  /* to joints if the machine type is cartesian */
  pose_t max_manual_accelerations;
  /* Max (axes, not joints) velocities for manual operation, but corresponds to */
  /* joints if the machine type is cartesian */
  pose_t max_manual_velocities;
  /* Maximum trajectory velocity of the end effector ("tool tip"/"pose") */
  real_t max_coord_velocity;
  /* Maximum trajectory acceleration of the end effector ("tool tip"/"pose") */
  real_t max_coord_acceleration;
  /* The interval time of the positioning, equals 1/sample rate */
  real_t sample_interval;
  /* The number of used joints, NOTE: it corresponds to the LAST USED JOINT + 1) */
  /* if there are inactive joints between, they still need to be calculated. */
  uint8_t number_of_used_joints;
  /* The trajectory generators will run interpolation_rate times slower than the */
  /* positioning. Position between are interpolated. */
  uint8_t interpolation_rate;
  /* All configurations for the joint planers */
  trajgen_jointtg_config_t joints[TG_MAX_JOINTS];
} trajgen_config_t;
 
/**
 * Joint definition
 */
typedef struct
{
  /* Interpolator data */
  interpolator_t interpolator;
  /* The trajgen for this axis */
  trajgen_jointtg_t tg;
  /* Uninterpolated positions */
  real_t planer_position;
  /* The interpolated position */
  real_t position;
  /* The interpolated velocity */
  real_t velocity;
  /* The interpolated acceleration */
  real_t acceleration;
  /* The interpolated jerk */
  real_t jerk;
} trajgen_joint_t;
 
/**
 * Internal state variables
 */
typedef struct
{
  uint32_t tick;
  real_t* joint_positions[TG_MAX_JOINTS];
  real_t* planer_positions[TG_MAX_JOINTS];
} trajgen_internals_t;
 
/**
 * Main trajgen type
 */
typedef struct
{
  /* Configuration of the trajectory generator */
  trajgen_config_t config;
  /* The last error occurred, only valid if a function returns nonzero */
  trajgen_error_details_t last_error;
  /* The actual operating state of the trajectory generator */
  trajgen_state_t state;
  /* The externally requested */
  trajgen_state_t requested_state;
  /* The joint data that the planer needs */
  trajgen_joint_t joints[TG_MAX_JOINTS];
  /* Instance of the coordinated motion planer */
  trajgen_coord_t coord_planer;
  /* Instance of the manual operation planer */
  trajgen_man_t man_planer;
  /* Internal state variables, NO NOT MODIFY */
  trajgen_internals_t _internals;
  /* The kinematics data structure */
  kinematics_t kinematics;
  /* The pose of the last planer calculation, not the interpolated one */
  pose_t planer_pose;
  /* Specifies if the planed motion is done */
  bool_t is_done;
} trajgen_t;
 
 
/**
 * Initialises the trajectory generator and all sub systems. This function
 * MUST BE CALLED BEFORE ANY OTHER OPERATIONS with the trajgen, as internal
 * pointer require initialisation.
 * @param trajgen_t *trajectory_generator_data_location
 * @param trajgen_config_t config
 * @return trajgen_error_t
 */
extern trajgen_error_t trajgen_initialize(trajgen_t *trajectory_generator_data_location,
    trajgen_config_t config);
 
/**
 * Resets the internal state, except the configuration
 * @return trajgen_error_t
 */
extern trajgen_error_t trajgen_reset();
 
/**
 * Sets the new requested trajgen state.
 * @param trajgen_state_t state
 * @return trajgen_error_t
 */
extern trajgen_error_t trajgen_switch_state(trajgen_state_t state);
 
/**
 * Abort the current generator, decelerate to standstill. After aborting you
 * should wait for done state.
 * @return trajgen_error_t
 */
extern trajgen_error_t trajgen_abort();
 
/**
 * Called frequently with the sample period tg.config.sample_interval.
 * @return trajgen_error_t
 */
extern trajgen_error_t trajgen_tick();
 
/**
 * Returns the current tick counter of the TG, which is increased every time
 * trajgen_tick() is called.
 * @return uint32_t
 */
extern uint32_t trajgen_get_tick();
 
/**
 * Writes build information into the *s.
 * @param char* s
 * @param unsigned length
 * @return const char*
 */
extern const char* trajgen_info(char* s, unsigned length);
 
/**
 * Coordinated planer accessors
 * APPLY COORDINATED MOTION PLANER ONLY. See trajgen_coord functions for
 * documentation.
 */
extern bool_t trajgen_queue_full();
extern uint32_t trajgen_current_id();
extern uint16_t trajgen_num_queued();
extern uint16_t trajgen_queue_size();
extern trajgen_error_t trajgen_pause();
extern trajgen_error_t trajgen_resume();
extern trajgen_error_t trajgen_add_line(pose_t end, real_t velocity, real_t acceleration);
extern trajgen_error_t trajgen_add_arc(pose_t end, pose_position_t center, pose_vector_t
    normal, unsigned n_turns, real_t velocity, real_t acceleration);
extern trajgen_coord_error_t trajgen_add_curve(pose_t end, pose_vector_t start_ctrl_point,
    pose_vector_t end_ctrl_point, real_t velocity, real_t acceleration);
 
/* </editor-fold> */
 
#ifdef __cplusplus
}
#endif
#endif

