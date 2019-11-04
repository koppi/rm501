/*******************************************************************************
 * @file trajgen.c
 * @author: Stefan Wilhelm (c erberos@atwillys.de)
 * @license: BSD (see below)
 * @version: 1.0b
 * @standard C99
 *
 * Allows to move the machine tool tip to specified positions, velocities and
 * accelerations by modifying a virtual machine position every generator cycle.
 * The output is always an updated position for every joint. The closed loop
 * controllers of the joints follow this position or, if the cannot, raise a
 * trailing error fault.
 *
 * More details in the header file trajgen.h
 *
 * Annotations:
 *
 *  - This file contains static variables pointing to your trajgen structure.
 *    That means it is assumed that you have only one trajectory generator
 *    in one process. This means as well that you instance of trajgen_t
 *    shall NOT be moved in memory. This disadvantage is the cost of much
 *    easier handling of the this piece of software. All static variables
 *    are assigned when you call trajgen_init(). The normal usage is to
 *    have a statically or dynamically allocated trajgen_t in the process
 *    heap or shared memory and then work with this memory block until the
 *    process quits.
 *    Interpolators, coordinated, manual and joint planer functions have
 *    the pointer to "this" as first function argument.
 *
 *  - For performance reasons this file does not use volatile declarations,
 *    interfacing it to the realtime system is your part. The normal usage
 *    procedure is:
 *
 *      - (Realtime (timed) thread function start (for the current sample interval)
 *      - Do pre-checks (motor states/estop/intermediate circuit/sync flags ...)
 *          - Conditionally trajgen_abort() on error conditions.
 *      - trajgen_tick();
 *      - Do post checks with your trajgen_t structure variable (errors etc)
 *      - Transfer new joint command positions into the motor control registers.
 *      - Transfer new digital I/O bits into the corresponding registers.
 *      - Process the real time FIFO/shared memory commands, e.g. to switch
 *        states, adapt override, add new motion commands, etc.
 *      - Leave thread function / wait for next sample in thread loop.
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
#include <stdio.h>
 
// <editor-fold defaultstate="collapsed" desc="Auxiliary functions/decls">
 
#include "trajgen.h"
 
/*******************************************************************************
 *
 * Auxiliary functions and declarations
 *
 ******************************************************************************/
 
// Version for info
#define TG_VERSION "1.0b"
 
// Definable memset(0)
#ifndef ZERO_MEMORY
#define ZERO_MEMORY zero_memory
static void zero_memory(void *dst, size_t length)
{
  char *p = (char*) dst;
  char *end = ((char*) p) + length - sizeof (long);
  while ((long*) p < (long*) end) {
    *((long*) p) = (long) 0;
    p += (unsigned long) sizeof (long);
  }
  end += sizeof (long);
  while (p < end) { *(char*) p = (char) 0; p++; }
}
#endif
 
// Definable sincos
#ifndef SINCOS
#define SINCOS(x, sx, cx) do { *(sx)=sin((x)); *(cx)=cos((x)); } while(0);
#endif
 
// Definable square root function
#ifndef TG_SQRT_FN
#define sqrtf sqrt
#else
#define sqrtf TG_SQRT_FN
#endif
 
// Pointer checks
#ifdef ARG_POINTER_CHECKS
#define CHK_PTR(X) if((X))
#else
#define CHK_PTR(X) if(0)
#endif
 
#ifndef NO_MOTION_BLENDING
#define NO_MOTION_BLENDING (0)
#else
#undef NO_MOTION_BLENDING
#define NO_MOTION_BLENDING (1)
#endif
 
/* Max angle between end->start vectors where blending is still allowed */
#ifndef MAX_ALLOWED_BLENDING_ANGLE_DEG
#define MAX_ALLOWED_BLENDING_ANGLE_DEG 85
#endif
 
#ifndef STRAIGHT_BLENDING_ANGLE
#define STRAIGHT_BLENDING_ANGLE 1
#endif
 
/* Define it if you don't like inlining */
#ifndef TG_INLINE
#define TG_INLINE __inline__
#endif
 
// stfwi: redo this a bit nicer
#ifdef TG_DEBUG_LEVEL
#if TG_DEBUG_LEVEL > 0
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
static void tg_debug(const char* fmt, ...)
{ va_list args; va_start (args, fmt); fflush(stdout);
  vfprintf(stderr, fmt, args); va_end(args); fflush(stderr); }
#define debug tg_debug
#else
#define debug  tg_nodebug
#endif
#if TG_DEBUG_LEVEL < 3
static void tg_nodebug(const char* pattern, ...) { ; }
#endif
#if TG_DEBUG_LEVEL > 1
#define debug2 tg_debug
#else
#define debug2 tg_nodebug
#endif
#if TG_DEBUG_LEVEL > 2
#define debug3 tg_debug
#else
#define debug3 tg_nodebug
#endif
#else
TG_INLINE static void tg_nodebug(const char* pattern, ...) { ; }
#define debug tg_nodebug
#define debug2 tg_nodebug
#define debug3 tg_nodebug
#endif
 
/*
 * Math "resolution", values define when a dimension or angle is treated
 * as zero.
 */
#ifndef TG_D_RES
#define TG_D_RES TG_RESOLUTION
#endif
#ifndef TG_A_RES
#define TG_A_RES TG_RESOLUTION
#endif
 
/*
 * Static switches, helps optimising the machine code
 */
#ifdef EXPORT_INTERPOLATORS
#define INTERPOLATION_STATIC
#else
#ifndef INTERPOLATION_STATIC
#define INTERPOLATION_STATIC static TG_INLINE
#endif
#endif
 
#ifdef EXPORT_JOINT_TG
#define JOINT_TG_STATIC
#else
#ifndef JOINT_TG_STATIC
#define JOINT_TG_STATIC static TG_INLINE
#endif
#endif
 
#ifdef EXPORT_MANUAL_TG
#define MANUAL_TG_STATIC
#else
#ifndef MANUAL_TG_STATIC
#define MANUAL_TG_STATIC static TG_INLINE
#endif
#endif
 
#ifdef EXPORT_COORD_TG
#define COORD_TG_STATIC
#define COORD_TG_STATICI
#else
#ifndef COORD_TG_STATIC
#define COORD_TG_STATIC static
#endif
#define COORD_TG_STATICI static TG_INLINE
#endif
 
static trajgen_error_t tg_err(unsigned code, unsigned joint);
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="Math/vector auxiliaries">
 
/********************************************************************
 *
 * Vector / robotic auxiliary functions
 *
 ********************************************************************/
 
#ifndef M_PI
#define M_PI (3.1415926535897932384626433832795029L)  /* pi */
#endif
#ifndef M_SQRT2
#define M_SQRT2 (sqrt(2.))  /* sqrt(2) */
#endif
 
#define sqr(x) ((x)*(x))
#define sqrtz(X) ((X)>0?sqrtf(X):((X)>-TG_RESOLUTION?0:NAN))
#define v_cpy(vs,vd) (*(vd)=(vs))
#define v_magnitude_sq(v) (sqr(v.x)+sqr(v.y)+sqr(v.z))
#define v_magnitude(v) (sqrtz(sqr(v.x)+sqr(v.y)+sqr(v.z)))
#define v_distance(v1,v2) sqrtz(sqr((v2).x-(v1).x)+sqr((v2).y-(v1).y)+sqr((v2).z-(v1).z))
#define v_dot(v1,v2) ((v1).x*(v2).x+(v1).y*(v2).y+(v1).z*(v2).z) /* "dot" scalar product */
#define v_err_t int
#define v_isfinite(p) (isfinite((p).x) && isfinite((p).y) && isfinite((p).z))
#define v_invalidate(p) { (p).x = (p).y = (p).z = NAN; }
#define v_setzero(p) { (p).x=(p).y=(p).z=0; }
 
#define v_pose_split(pose, xyz, abc, uvw) { \
  (xyz)->x=(pose).x; (xyz)->y=(pose).y; (xyz)->z=(pose).z; \
  (uvw)->x=(pose).u; (uvw)->y=(pose).v; (uvw)->z=(pose).w; \
  (abc)->x=(pose).a; (abc)->y=(pose).b; (abc)->z=(pose).c; \
}
#define v_pose_merge(xyz, abc, uvw, pose) { \
  (pose)->x=(xyz).x; (pose)->y=(xyz).y; (pose)->z=(xyz).z; \
  (pose)->u=(uvw).x; (pose)->v=(uvw).y; (pose)->w=(uvw).z; \
  (pose)->a=(abc).x; (pose)->b=(abc).y; (pose)->c=(abc).z; \
}
 
TG_INLINE static v_err_t v_scale(pose_vector_t v1, real_t d, pose_vector_t * r)
{ CHK_PTR(!r) return 1; r->x=v1.x*d; r->y=v1.y*d; r->z=v1.z*d; return 0; }
 
TG_INLINE static v_err_t v_sum(pose_vector_t v1, pose_vector_t v2, pose_vector_t * r)
{ CHK_PTR(!r) return 1; r->x=v1.x+v2.x; r->y=v1.y+v2.y; r->z=v1.z+v2.z; return 0; }
 
TG_INLINE static v_err_t v_sub(pose_vector_t v1, pose_vector_t v2, pose_vector_t * r)
{ CHK_PTR(!r) return 1; r->x=v1.x-v2.x; r->y=v1.y-v2.y; r->z=v1.z-v2.z; return 0; }
 
TG_INLINE static v_err_t v_cross(pose_vector_t v1, pose_vector_t v2, pose_vector_t * r)
{ CHK_PTR(!r) return 1; r->x=v1.y*v2.z-v1.z*v2.y; r->y=v1.z*v2.x-v1.x*v2.z;
  r->z=v1.x*v2.y-v1.y*v2.x; return 0; }
 
TG_INLINE static v_err_t v_unit(pose_vector_t v, pose_vector_t * r)
{ CHK_PTR(!r) return 1; real_t l; if ((l=v_magnitude(v)) == 0.0) {
  r->x=r->y=r->z=0; return 1; } r->x = v.x/l; r->y = v.y/l; r->z = v.z/l; return 0; }
 
TG_INLINE static v_err_t v_project(pose_vector_t v1, pose_vector_t v2, pose_vector_t *r)
{ CHK_PTR(!r) return 1; if(v_unit(v2, &v2)) return 1; v_scale(v2, v_dot(v1,v2), r);
  return 0; }
 
TG_INLINE static v_err_t v_plane_project(pose_vector_t v, pose_vector_t normal, pose_vector_t *r)
{ CHK_PTR(!r) return 1; pose_vector_t p; return v_project(v, normal, &p) || v_sub(v, p, r); }
 
/**
 * Defines a line, precalculates characteristic values.
 * @param pose_line_t *line
 * @param pose_position_t start
 * @param pose_position_t end
 * @return int
 */
static v_err_t v_lin_define(pose_line_t *line, pose_position_t start, pose_position_t end)
{
  CHK_PTR(!line) return 1;
  line->start = start;
  line->end = end;
  v_sub(end, start, &line->u);
  line->tm = v_magnitude(line->u);
  if (line->tm < TG_D_RES) {
    line->u.x = 1;
    line->tm = line->u.y = line->u.z = 0;
  } else if(v_unit(line->u, &line->u)) {
    line->tm = line->u.x = line->u.y = line->u.z = 0;
    return 1;
  }
  return 0;
}
 
/**
 * Calculates a point for a line motion from a progress (current one-dim position)
 * @param pose_line_t *line
 * @param real_t progress
 * @param pose_position_t *point
 * @return int
 */
static v_err_t v_lin_get(pose_line_t *line, real_t progress, pose_position_t *point)
{
  CHK_PTR(!line || !point) return 1;
  if (line->tm <= 0 || progress >= line->tm) {
    *point = line->end;
  } else {
    v_scale(line->u, progress, point);
    v_sum(line->start, *point, point);
  }
  return 0;
}
 
/**
 * Create a circle/arc/helix definition, precalculate characteristics.
 * @param pose_circle_t *circle
 * @param pose_position_t start
 * @param pose_position_t end
 * @param pose_vector_t center
 * @param pose_vector_t normal
 * @param int n_turns
 * @return int
 */
static v_err_t v_arc_define(pose_circle_t * circle, pose_position_t start,
    pose_position_t end, pose_vector_t center, pose_vector_t normal, unsigned n_turns)
{
  CHK_PTR(!circle) return 1;
  pose_vector_t v, e = { 0.0, 0.0, 0.0 }; // v:aux, e: rev. end vector
  real_t d;
  v_sub(start, center, &v); // determine projected centre point
  if(v_project(v, normal, &v)) return 1;
  v_sum(v, center, &circle->cp); // real center: point on normal in plane of the start point
  v_unit(normal, &circle->nv); // unit normal vector (below: negative turns -> other direction)
  circle->r = v_distance(start, circle->cp); // radius from start point
  if(circle->r < TG_D_RES) return 1; // start radius 0
  v_sub(end, center, &v); // determine projected centre point
  if(v_project(v, normal, &v)) return 1;
  v_sub(start, circle->cp, &circle->rt); // tangent normal vector
  v_cross(circle->nv, circle->rt, &circle->rp); // vector in helix direction
  v_sub(end, circle->cp, &circle->rh); // helix
  v_plane_project(circle->rh, circle->nv, &e);
  circle->s = v_magnitude(e) - circle->r;
  v_sub(circle->rh, e, &circle->rh);
  v_unit(e, &e);
  v_scale(e, circle->r, &e);
  if(v_magnitude(e) == 0) { v_scale(circle->nv, FLOAT_EPS, &v); v_sum(e, v, &e); }
  d = v_dot(circle->rt, e)/sqr(circle->r); // determine angle
  circle->a = d>1.0 ? 0.0 : (d<-1?M_PI : acos(d));
  v_cross(circle->rt, e, &v); // Angle correction (0->PI --> PI->2PI)
  if(v_dot(v, circle->nv) < 0) circle->a = (2.0*M_PI) - circle->a;
  if(circle->a > -(TG_A_RES) && circle->a < (TG_A_RES)) circle->a = 0;
  if(n_turns > 0) circle->a += n_turns * 2.0*M_PI; // add turns
  circle->l = sqrtz(sqr(circle->a * circle->r) + sqr(v_magnitude(circle->rh))); // Segment length
  return (circle->a == 0); // angle 0 -> error
}
 
/**
 * Returns a point on a circle at a given angle (progress).
 * @param pose_circle_t *circle
 * @param real_t angle
 * @param pose_position_t *point
 * @return int
 */
static v_err_t v_arc_get(pose_circle_t * circle, real_t progress, pose_position_t *point)
{
  CHK_PTR(!circle || !point) return 1;
  pose_vector_t p, s;
  real_t sx, cx;
  progress /= circle->l; // length --> 0..1
  SINCOS(progress * circle->a, &sx, &cx); // length --> angle
  v_scale(circle->rp, sx, &s);
  v_scale(circle->rt, cx, &p);
  v_sum(p, s, point);           // radius vector to center
  v_unit(*point, &p);    // add scaled radius vector
  v_scale(p, progress * circle->s, &p);// parallel component
  v_sum(*point, p, point);
  v_scale(circle->rh, progress, &s);   // perpendicular component
  v_sum(*point, s, point);
  v_sum(circle->cp, *point, point); // translate relative to center point
  return 0;
}
 
#ifdef WITH_CURVES
/* Experimental, not places this code yet. */
#include "curve.inc.c"
#endif
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="interpolator_*: Interpolator">
 
/*******************************************************************************
 *
 * Interpolator (cubic spline)
 *
 ******************************************************************************/
 
#ifndef NO_INTERPOLATION
 
INTERPOLATION_STATIC bool_t interpolator_need_update(interpolator_t *ci)
{
  CHK_PTR(!ci) return INTERPOLATOR_ERROR_NULLPOINTER;
  return ci->n<4;
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_push(interpolator_t * ip,
    real_t point)
{
  debug3("# ip: push %g\n", point);
  // TG_CC_FN_CALL(TG_CC_interpolator_push);
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  if (ip->n >= 4) {
    return INTERPOLATOR_ERROR_QUEUE_FULL;
  } else if (ip->n == 0) {
    return INTERPOLATOR_ERROR_NOT_RESET;
  } else {
    ip->x0 = ip->x1;
    ip->x1 = ip->x2;
    ip->x2 = ip->x3;
    ip->x3 = point;
    ip->n++;
  }
  real_t T = ip->T;
  ip->w0 = (ip->x0 + 4.0*ip->x1 + ip->x2) / 6.0;
  ip->w1 = (ip->x1 + 4.0*ip->x2 + ip->x3) / 6.0;
  ip->v0 = ((ip->x2) - (ip->x0)) / (2.0 * (T));
  ip->v1 = ((ip->x3) - (ip->x1)) / (2.0 * (T));
  ip->d = ip->w0;
  ip->c = ip->v0;
  ip->b = 3 * (ip->w1 - ip->w0)/(T*T) - (2 * ip->v0 + ip->v1)/T;
  ip->a = (ip->v1 - ip->v0)/(3.0*(T*T)) - (2.0 * ip->b)/(3.0*T);
  ip->t = 0.0;
  return INTERPOLATOR_OK;
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_reset(interpolator_t * ip,
    real_t initial_position)
{
  // TG_CC_FN_CALL(TG_CC_interpolator_reset);
  debug3("# ip: reset\n");
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  ip->n = 1;
  ip->t = 0.0;
  ip->x0 = ip->x1 = ip->x2 = ip->x3 = ip->w0 = ip->w1 = ip->d = initial_position;
  ip->v0 = ip->v1 = ip->a = ip->b = ip->c = 0.0;
  return interpolator_push(ip, initial_position);
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_init(interpolator_t * ip,
    real_t sample_interval, unsigned interpolation_rate)
{
  debug3("# ip: init\n");
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  if (sample_interval <= 0.0 || interpolation_rate < 1) {
    return INTERPOLATOR_ERROR_INIT_ARG_INVALID;
  }
  ip->T = sample_interval;
  ip->Ti = ip->T / interpolation_rate;
  interpolator_reset(ip, 0);
  return INTERPOLATOR_OK;
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_interpolate(interpolator_t *ip,
    real_t *x, real_t *v, real_t *a, real_t *j)
{
  // TG_CC_FN_CALL(TG_CC_interpolator_interpolate);
  CHK_PTR(!ip||!x||!v||!a||!j) return INTERPOLATOR_ERROR_INTERPOLATE_ARG_NULLPOINTER;
  if (ip->n<4) interpolator_push(ip, ip->x3); // auto refill
  if(ip->x3 == ip->x2 && ip->x1 == ip->x3) {
    *x = ip->x3; *v = *a = *j = 0;
    debug3("# ip: interpolate: positions equal: %g (v-0, a=0)\n", ip->x3);
  } else {
    real_t t = ip->t; // let the compiler optimise all this
    *x = (((ip->a*t + ip->b)*t) + ip->c) * t + ip->d;
    *v = ((3.0 * ip->a*t) + 2.0 * ip->b) * t + ip->c;
    *a = 6.0 * ip->a*t + 2.0 * ip->b;
    *j = 6.0 * ip->a;
    debug3("# ip: interpolate: s%g v%g a%g\n", *x, *v, *a);
  }
  ip->t += ip->Ti;
  if (fabs(ip->T - ip->t) < 0.5 * ip->Ti) ip->n--;
  return INTERPOLATOR_OK;
}
 
#else
 
INTERPOLATION_STATIC bool_t interpolator_need_update(interpolator_t *ip)
{
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  return ip->n<4;
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_push(interpolator_t * ip,
    real_t point)
{
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  if (ip->n >= 4) {
    return INTERPOLATOR_ERROR_QUEUE_FULL;
  } else if (ip->n == 0) {
    return INTERPOLATOR_ERROR_NOT_RESET;
  } else {
    ip->x0 = ip->x1;
    ip->x1 = ip->x2;
    ip->x2 = ip->x3;
    ip->x3 = point;
    ip->n++;
  }
  return INTERPOLATOR_OK;
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_reset(interpolator_t * ip,
    real_t initial_position)
{
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  ip->n = 2;
  ip->t = 0.0;
  ip->x0 = ip->x1 = ip->x2 = ip->x3 = ip->w0 = ip->w1 = ip->d = initial_position;
  ip->v0 = ip->v1 = ip->a = ip->b = ip->c = 0.0;
  return interpolator_push(ip, initial_position);
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_init(interpolator_t * ip,
    real_t sample_interval, unsigned interpolation_rate)
{
  CHK_PTR(!ip) return INTERPOLATOR_ERROR_NULLPOINTER;
  if (sample_interval <= 0.0 || interpolation_rate != 1) {
    return INTERPOLATOR_ERROR_INIT_ARG_INVALID;
  }
  ip->T = sample_interval;
  ip->Ti = 1.0/ip->T;
  interpolator_reset(ip, 0);
  return INTERPOLATOR_OK;
}
 
INTERPOLATION_STATIC interpolator_error_t interpolator_interpolate(interpolator_t *ip,
    real_t *x, real_t *v, real_t *a, real_t *j)
{
  CHK_PTR(!ip || !x || !v || !a || !j) return INTERPOLATOR_ERROR_INTERPOLATE_ARG_NULLPOINTER;
  if (ip->n<4) interpolator_push(ip, ip->x3);
  if(ip->x3 == ip->x2 && ip->x1 == ip->x3) {
    *x = ip->x3; *v = *a = *j = 0;
  } else {
    *x = ip->x3;
    *v = (ip->x3 - ip->x2) * ip->Ti;
    *a = (*v-ip->v0) * ip->Ti;
    *j = (*a-ip->v1) * ip->Ti;
    ip->v0 = *v;
    ip->v1 = *a;
  }
  ip->n--;
  return INTERPOLATOR_OK;
}
 
#endif
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="kinematics_*: Default identity kinematic model">
 
/*******************************************************************************
 *
 * Kinematics manager. Built-in default is an identity system.
 * See header file for documentation about how to set your own kinematics.
 *
 ******************************************************************************/
 
/**
 * Default passthrough forward kinematics
 * @param real_t *joint_positions
 * @param pose_t *world
 * @return kinematics_error_t
 */
static kinematics_error_t kinematics_identity_forward(real_t **joint_positions,
    pose_t *world)
{
  CHK_PTR(!joint_positions || !world) return KINEMATICS_ERR_NULL_POINTER;
  world->x = *(joint_positions[0]);
  world->y = *(joint_positions[1]);
  world->z = *(joint_positions[2]);
  world->a = *(joint_positions[3]);
  world->b = *(joint_positions[4]);
  world->c = *(joint_positions[5]);
  world->u = *(joint_positions[6]);
  world->v = *(joint_positions[7]);
  world->w = *(joint_positions[8]);
  return KINEMATICS_ERR_OK;
}
 
/**
 * Default passthrough inverse kinematics
 * @param pose_t *world
 * @param real_t *joint_positions
 * @return kinematics_error_t
 */
static kinematics_error_t kinematics_identity_inverse(pose_t *world, real_t **joint_positions)
{
  CHK_PTR(!joint_positions || !world) return KINEMATICS_ERR_NULL_POINTER;
  *(joint_positions[0]) = world->x;
  *(joint_positions[1]) = world->y;
  *(joint_positions[2]) = world->z;
  *(joint_positions[3]) = world->a;
  *(joint_positions[4]) = world->b;
  *(joint_positions[5]) = world->c;
  *(joint_positions[6]) = world->u;
  *(joint_positions[7]) = world->v;
  *(joint_positions[8]) = world->w;
  return KINEMATICS_ERR_OK;
}
 
/**
 * Default passthrough kinematics homeing
 * @param pose_t *world
 * @param real_t *joint_positions
 * @return kinematics_error_t
 */
static kinematics_error_t kinematics_identity_home(pose_t *world, real_t **joint_positions)
{
  CHK_PTR(!joint_positions || !world) return KINEMATICS_ERR_NULL_POINTER;
  return kinematics_identity_forward(joint_positions, world);
}
 
/**
 * Selects and initializes a machine type, sets the appropriate pointers to the
 * associated functions.
 * @param kinematics_t *kin
 * @param kinematics_t *device_kinematics
 * @return kinematics_error_t
 */
kinematics_error_t kinematics_initialize(kinematics_t *kin, kinematics_t device_kinematics)
{
  if (!kin) return KINEMATICS_ERR_NULL_POINTER;
  kin->forward = kinematics_identity_forward;
  kin->inverse = kinematics_identity_inverse;
  kin->reset = kinematics_identity_home;
  if(device_kinematics.forward) kin->forward = device_kinematics.forward;
  if(device_kinematics.inverse) kin->inverse = device_kinematics.inverse;
  if(device_kinematics.reset) kin->reset = device_kinematics.reset;
  if((!device_kinematics.forward) ^ (!device_kinematics.inverse)) {
    return KINEMATICS_ERR_INIT_FUNCTION_NULL;
  }
  return KINEMATICS_ERR_OK;
}
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="trajgen_coord_*: Coordinated motion planer">
 
/*******************************************************************************
 *
 *
 * Coordinated motion planner.
 *
 *
 ******************************************************************************/
 
/**
 * Error callback wrapper, passes through the error code without changing.
 * @param trajgen_coord_error_t e
 * @return trajgen_coord_error_t
 */
static TG_INLINE trajgen_coord_error_t trajgen_coord_raise_error(trajgen_coord_t* tp,
    trajgen_coord_error_t e)
{
  if (!tp) return e;
  tg_err(e, 0);
  return tp->last_error = e;
}
 
/**
 * Calculates a unit vector for starting a segment.
 * @param trajgen_sg_t *sg
 * @param pose_vector_t* result
 * @return trajgen_coord_error_t
 */
static trajgen_coord_error_t trajgen_sg_start_direction(trajgen_coord_t* tp,
    trajgen_sg_t *sg, pose_vector_t *result)
{
  CHK_PTR(!sg || !result) return TP_ERR_TP_NULL_POINTER;
  switch (sg->motion_type) {
    case SG_LINEAR:
    {
      v_sub(sg->coords.line.xyz.end, sg->coords.line.xyz.start, result);
      v_unit(*result, result);
      return TP_ERR_OK;
    }
    case SG_CIRCULAR:
    {
      pose_position_t startpoint;
      pose_vector_t radius, tan, perp;
      v_arc_get(&sg->coords.arc.xyz, 0.0, &startpoint);
      v_sub(startpoint, sg->coords.arc.xyz.cp, &radius);
      v_cross(sg->coords.arc.xyz.nv, radius, &tan);
      v_unit(tan, &tan);
      v_sub(sg->coords.arc.xyz.cp, startpoint, &perp);
      v_unit(perp, &perp);
      v_scale(tan, sg->a, &tan);
      v_scale(perp, sqr(0.5 * sg->v) / sg->coords.arc.xyz.r, &perp);
      v_sum(tan, perp, result);
      v_unit(*result, result);
      return TP_ERR_OK;
    }
    #ifdef WITH_CURVES
    case SG_CURVE:
    {
      v_cpy(sg->coords.curve.xyz.suv, result);
      return TP_ERR_OK;
    }
    #endif
  }
  return trajgen_coord_raise_error(tp, TP_ERR_INVALID_MOTION_TYPE);
}
 
/**
 * Calculates a unit vector at the end of a segment.
 * @param trajgen_sg_t *sg
 * @param pose_vector_t* result
 * @return trajgen_coord_error_t
 */
static trajgen_coord_error_t trajgen_sg_end_direction(trajgen_coord_t* tp,
    trajgen_sg_t *sg, pose_vector_t *result)
{
  CHK_PTR(!sg || !result) return TP_ERR_TP_NULL_POINTER;
  switch (sg->motion_type) {
    case SG_LINEAR:
    {
      v_sub(sg->coords.line.xyz.end, sg->coords.line.xyz.start, result);
      v_unit(*result, result);
      return TP_ERR_OK;
    }
    case SG_CIRCULAR:
    {
      pose_position_t endpoint;
      pose_vector_t radius;
      v_arc_get(&sg->coords.arc.xyz, sg->coords.arc.xyz.a, &endpoint);
      v_sub(endpoint, sg->coords.arc.xyz.cp, &radius);
      v_cross(sg->coords.arc.xyz.nv, radius, result);
      v_unit(*result, result);
      return TP_ERR_OK;
    }
    #ifdef WITH_CURVES
    case SG_CURVE:
    {
      v_cpy(sg->coords.curve.xyz.euv, result);
      return TP_ERR_OK;
    }
    #endif
  }
  return trajgen_coord_raise_error(tp, TP_ERR_INVALID_MOTION_TYPE);
}
 
/**
 * Push a new segment into the queue.
 */
static TG_INLINE trajgen_coord_error_t trajgen_coord_queue_push_back(trajgen_coord_t* tp,
    trajgen_sg_t *sg)
{
  CHK_PTR(sg) return trajgen_coord_raise_error(tp, TP_ERR_TP_NULL_POINTER);
  unsigned next = (tp->queue.end + 1) % tp->queue.size;
  if (next == tp->queue.start) return trajgen_coord_raise_error(tp, TP_ERR_QUEUE_FULL);
  tp->queue.queue[tp->queue.end] = *sg;
  tp->queue.num_elements++;
  tp->queue.end = next;
  debug2("# tq: pushed segment %lu\n", (unsigned long)sg->id);
  return TP_ERR_OK;
}
 
/**
 * Removes n elements from the front of the queue
 * @return trajgen_coord_error_t
 */
static TG_INLINE trajgen_coord_error_t trajgen_coord_queue_pop_front(trajgen_coord_t* tp,
    int n)
{
  if (n <= 0) return TP_ERR_OK;
  if (n > tp->queue.num_elements) {
    return trajgen_coord_raise_error(tp, TP_ERR_QUEUE_TO_MANY_ELEMENTS_TO_REMOVE);
  }
  debug2("# tq: popped segment %lu\n", (unsigned long)(tp->queue.queue[tp->queue.start].id));
  tp->queue.start = (tp->queue.start + n) % tp->queue.size;
  tp->queue.num_elements -= n;
  return TP_ERR_OK;
}
 
/**
 * Returns the head item of the queue
 * @return trajgen_sg_t *
 */
static TG_INLINE trajgen_sg_t *trajgen_coord_queue_front(trajgen_coord_t* tp)
{ return (tp->queue.num_elements==0) ? (trajgen_sg_t*) 0 :
  (&(tp->queue.queue[tp->queue.start])); }
 
/**
 * Returns a queue item by value without modifying the queue
 * @return trajgen_sg_t *
 */
static TG_INLINE trajgen_sg_t *trajgen_coord_queue_get(trajgen_coord_t* tp, unsigned n)
{ return (n>=tp->queue.num_elements) ? (trajgen_sg_t*) 0 :
  &(tp->queue.queue[(tp->queue.start + n) % tp->queue.size]); }
 
/**
 * Returns true if the queue is completely full. For margin checks, the function
 * trajgen_coord_get_num_queued() can be used in combination with the known size.
 * @return bool_t
 */
COORD_TG_STATICI bool_t trajgen_coord_is_queue_full(trajgen_coord_t* tp)
{ return tp->queue.num_elements >= tp->queue.size-1; }
//{ return (bool_t) (((tp->queue.end+1) % tp->queue.size) == tp->queue.start); }
 
/**
 * Returns the size (maximum length) of the queue
 * @return uint16_t
 */
COORD_TG_STATICI uint16_t trajgen_coord_get_queue_size(trajgen_coord_t* tp)
{ return tp->queue.size; }
 
/**
 * Resets the planner without overwriting the configuration and actual position
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_reset(trajgen_coord_t* tp)
{
  CHK_PTR(!tp) return TP_ERR_TP_NULL_POINTER;
  tp->last_error = TP_ERR_OK;
  tp->end_pose = tp->_i.last_position = tp->pose;
  tp->queue.num_elements = tp->queue.start = tp->queue.end = 0;
  tp->sg_id = tp->next_sg_id = tp->num_queued_active = 0;
  tp->is_aborting = tp->is_pausing = 0;
  tp->sync_dio_current = tp->sync_dio_command.clear = tp->sync_dio_command.set = 0;
  tp->override = tp->_i.override = 1.0;
  tp->override_enabled = 1;
  tp->sync.type = TP_SYNC_NONE;
  tp->sync.reference = 0;
  tp->sync.i_offset = tp->sync.i_isset = 0;
  tp->is_done = 1;
  return TP_ERR_OK;
}
 
/**
 * Initializes the planner
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_init(trajgen_coord_t *tp,
    trajgen_coord_config_t config)
{
  if (!tp) return TP_ERR_TP_NULL_POINTER;
  ZERO_MEMORY(tp, sizeof (trajgen_coord_t)); // Set all values and pointers to 0
  trajgen_coord_reset(tp);
  tp->queue.size = TG_QUEUE_SIZE;
  tp->config = config;
  if (config.max_acceleration <= 0.0) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_ACCEL);
  } else if (config.max_velocity <= 0.0) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_SPEED);
  } else if (config.sample_interval <= 0.0) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_SAMPLE_INTERVAL);
  } else {
    return TP_ERR_OK;
  }
}
 
/**
 * Set the current position of the planer. DO NOT TO THIS WHEN THE planer is
 * running (done!=1).
 * @param pose_t pos
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_set_position(trajgen_coord_t* tp,
    pose_t p)
{ tp->pose = tp->end_pose = p; return TP_ERR_OK; }
 
/**
 * Pause motion
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_pause(trajgen_coord_t* tp)
{ tp->is_pausing = 1; return TP_ERR_OK; }
 
/**
 * Resume from pause state
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_resume(trajgen_coord_t* tp)
{ tp->is_pausing = 0; return TP_ERR_OK; }
 
/**
 * Abort all coordinated motion
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_abort(trajgen_coord_t* tp)
{
  CHK_PTR(!tp) return TP_ERR_TP_NULL_POINTER;
  if (!tp->is_aborting) tp->is_pausing = tp->is_aborting = 1;
  tp->sync_dio_command.clear = tp->sync_dio_command.set = 0;
  return TP_ERR_OK;
}
 
/**
 * Returns the id of the last motion that is currently executing.
 * @return uint32_t
 */
COORD_TG_STATICI uint32_t trajgen_coord_get_current_id(trajgen_coord_t* tp)
{ return tp->sg_id; }
 
/**
 * Returns the actual output position of the generator
 * @return pose_t
 */
COORD_TG_STATICI pose_t trajgen_coord_get_position(trajgen_coord_t* tp)
{ return tp->pose; }
 
/**
 * Returns nonzero if the generator finished processing the queue and
 * reached the target position.
 * @return bool_t
 */
COORD_TG_STATICI bool_t trajgen_coord_is_done(trajgen_coord_t* tp)
{ return tp->is_done; }
 
/**
 * Returns the number of currently queued items
 * @return uint16_t
 */
COORD_TG_STATICI uint16_t trajgen_coord_get_num_queued(trajgen_coord_t* tp)
{ return tp->queue.num_elements; }
 
/**
 * Returns the number of currently processed items in the queue
 * @return uint16_t
 */
COORD_TG_STATICI uint16_t trajgen_coord_get_num_queued_active(trajgen_coord_t* tp)
{ return tp->num_queued_active; }
 
/**
 * Add a straight line into the queue. Current speed/accel/io settings apply.
 * @param pose_t end_point
 * @param real_t velocity
 * @param real_t acceleration
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_add_line(trajgen_coord_t* tp,
    pose_t end_point, real_t velocity, real_t acceleration)
{
  trajgen_sg_t sg;
  //pose_line_t line_xyz, line_uvw, line_abc;
  pose_position_t start_xyz, end_xyz, start_uvw, end_uvw, start_abc, end_abc;
  sg.sync_dio = tp->sync_dio_command;
  tp->sync_dio_command.clear = tp->sync_dio_command.set = 0;
  if (tp->is_aborting) return trajgen_coord_raise_error(tp, TP_ERR_ABORTING);
  if (trajgen_coord_is_queue_full(tp)) return trajgen_coord_raise_error(tp, TP_ERR_QUEUE_FULL);
  acceleration = acceleration == 0 ? tp->config.max_acceleration : acceleration;
  if (velocity < TG_RESOLUTION || velocity > tp->config.max_velocity) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_SPEED);
  } else if (acceleration < TG_RESOLUTION || acceleration > tp->config.max_acceleration) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_ACCEL);
  } else if(!pose_isfinite(end_point)) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_POSE);
  }
  v_pose_split(tp->end_pose, &start_xyz, &start_abc, &start_uvw);
  v_pose_split(end_point, &end_xyz, &end_abc, &end_uvw);
  if(
    v_lin_define(&sg.coords.line.xyz, start_xyz, end_xyz)
    #ifndef NO_UVW
    || v_lin_define(&sg.coords.line.uvw, start_uvw, end_uvw)
    #endif
    #ifndef NO_ABC
    || v_lin_define(&sg.coords.line.abc, start_abc, end_abc)
    #endif
  ) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_PARAM);
  }
  #ifdef NO_ABC
  v_setzero(sg.coords.line.abc.start);
  v_setzero(sg.coords.line.abc.end);
  v_setzero(sg.coords.line.abc.u);
  sg.coords.line.abc.tm = 0;
  #endif
  #ifdef NO_UVW
  v_setzero(sg.coords.line.uvw.start);
  v_setzero(sg.coords.line.uvw.end);
  v_setzero(sg.coords.line.uvw.u);
  sg.coords.line.uvw.tm = 0;
  #endif
  sg.length = (sg.coords.line.xyz.tm>TG_D_RES) ? sg.coords.line.xyz.tm :
    (sg.coords.line.uvw.tm>TG_D_RES) ? (sg.coords.line.uvw.tm) : (sg.coords.line.abc.tm);
  if(sg.length < TG_RESOLUTION) {
    sg.length = 0;
    return trajgen_coord_raise_error(tp, TP_ERR_SEGMENT_LENGTH_ZERO);
  }
  sg.id = tp->next_sg_id;
  sg.is_active = 0;
  sg.v = velocity;
  sg.a = acceleration;
  sg.blending_tolerance = tp->blending_tolerance<TG_RESOLUTION ? 0:tp->blending_tolerance;
  sg.sync_type = tp->sync.type;
  sg.override_enabled = tp->override_enabled;
  sg.motion_type = SG_LINEAR;
  if (trajgen_coord_queue_push_back(tp, &sg) != 0) {
    return tp->last_error;
  }
  tp->end_pose = end_point; // remember the end of this move, as it's the start of the next one.
  tp->next_sg_id++;
  tp->is_done = 0;
  return TP_ERR_OK;
}
 
/**
 * Push an arc/circle/helix motion into the queue. Current speed/accel/io
 * settings apply.
 * @param pose_t end
 * @param pose_vector_t center
 * @param pose_vector_t normal
 * @param unsigned n_turns
 * @param real_t velocity
 * @param real_t acceleration
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_add_arc(trajgen_coord_t* tp,
    pose_t end, pose_position_t center, pose_vector_t normal, unsigned n_turns,
    real_t velocity, real_t acceleration)
{
  trajgen_sg_t sg;
  pose_position_t start_xyz, end_xyz, start_uvw, end_uvw, start_abc, end_abc;
  sg.sync_dio = tp->sync_dio_command;
  tp->sync_dio_command.clear = tp->sync_dio_command.set = 0;
  sg.motion_type = SG_CIRCULAR;
  sg.id = tp->next_sg_id;
  sg.is_active = 0;
  sg.sync_type = tp->sync.type;
  sg.override_enabled = tp->override_enabled;
  sg.blending_tolerance = tp->blending_tolerance<TG_RESOLUTION ? 0:tp->blending_tolerance;
  if (tp->is_aborting) return trajgen_coord_raise_error(tp, TP_ERR_ABORTING);
  if (trajgen_coord_is_queue_full(tp)) return trajgen_coord_raise_error(tp, TP_ERR_QUEUE_FULL);
  acceleration = acceleration == 0 ? tp->config.max_acceleration : acceleration;
  if (velocity < TG_RESOLUTION || velocity > tp->config.max_velocity) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_SPEED);
  } else if (acceleration < TG_RESOLUTION || acceleration > tp->config.max_acceleration) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_ACCEL);
  } else if(!pose_isfinite(end) || !v_isfinite(center) || !v_isfinite(normal)) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_POSE);
  }
  sg.v = velocity;
  sg.a = acceleration;
  v_pose_split(tp->end_pose, &start_xyz, &start_abc, &start_uvw);
  v_pose_split(end, &end_xyz, &end_abc, &end_uvw);
 
  #ifdef NO_ABC
  v_setzero(sg.coords.arc.abc.start);
  v_setzero(sg.coords.arc.abc.end);
  v_setzero(sg.coords.arc.abc.u);
  sg.coords.arc.abc.tm = 0;
  #endif
  #ifdef NO_UVW
  v_setzero(sg.coords.arc.uvw.start);
  v_setzero(sg.coords.arc.uvw.end);
  v_setzero(sg.coords.arc.uvw.u);
  sg.coords.arc.uvw.tm = 0;
  #endif
  if(
    v_distance(start_xyz, center) < TG_RESOLUTION ||
    v_distance(end_xyz, center) < TG_RESOLUTION
  ) {
    debug("# tp: trajgen_coord_add_arc(): distance from center ~== 0\n");
    return trajgen_coord_raise_error(tp, TP_ERR_SEGMENT_LENGTH_ZERO);
  } else if(
    v_arc_define(&sg.coords.arc.xyz, start_xyz, end_xyz, center, normal, n_turns)
    #ifndef NO_ABC
    || v_lin_define(&sg.coords.arc.abc, start_abc, end_abc)
    #endif
    #ifndef NO_UVW
    || v_lin_define(&sg.coords.arc.uvw, start_uvw, end_uvw)
    #endif
  ) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_PARAM);
  } else if(sg.coords.arc.xyz.a == 0) {
    debug("# tp: trajgen_coord_add_arc(): arc.a===0\n");
    return trajgen_coord_raise_error(tp, TP_ERR_SEGMENT_LENGTH_ZERO);
  }
  sg.length = sg.coords.arc.xyz.l;
  if(sg.length < TG_RESOLUTION) {
    sg.length = 0;
    debug("# tp: trajgen_coord_add_arc(): sg.length~==0\n");
    return trajgen_coord_raise_error(tp, TP_ERR_SEGMENT_LENGTH_ZERO);
  }
  if (trajgen_coord_queue_push_back(tp, &sg) != 0) {
    return tp->last_error;
  }
  tp->end_pose = end;
  tp->next_sg_id++;
  tp->is_done = 0;
  return TP_ERR_OK;
}
 
/**
 * Push a curve/nurbs segment.
 * @param pose_t end
 * @param pose_vector_t start_handle
 * @param pose_vector_t end_handle
 * @param real_t velocity
 * @param real_t acceleration
 * @return trajgen_coord_error_t
 */
COORD_TG_STATICI trajgen_coord_error_t trajgen_coord_add_curve(trajgen_coord_t* tp,
    pose_t end, pose_vector_t start_ctrl_point, pose_vector_t end_ctrl_point,
    real_t velocity, real_t acceleration)
{
  #ifdef WITH_CURVES
  trajgen_sg_t sg;
  pose_position_t start_xyz, end_xyz, start_uvw, end_uvw, start_abc, end_abc;
  sg.sync_dio = tp->sync_dio_command;
  tp->sync_dio_command.clear = tp->sync_dio_command.set = 0;
  sg.motion_type = SG_CURVE;
  sg.id = tp->next_sg_id;
  sg.is_active = 0;
  sg.sync_type = tp->sync.type;
  sg.override_enabled = tp->override_enabled;
  sg.blending_tolerance = tp->blending_tolerance<TG_RESOLUTION ? 0:tp->blending_tolerance;
  if (tp->is_aborting) return trajgen_coord_raise_error(tp, TP_ERR_ABORTING);
  if (trajgen_coord_is_queue_full(tp)) return trajgen_coord_raise_error(tp, TP_ERR_QUEUE_FULL);
  acceleration = acceleration == 0 ? tp->config.max_acceleration : acceleration;
  if (velocity < TG_RESOLUTION || velocity > tp->config.max_velocity) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_SPEED);
  } else if (acceleration < TG_RESOLUTION || acceleration > tp->config.max_acceleration) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_ACCEL);
  } else if(!pose_isfinite(end) || !v_isfinite(start_ctrl_point) || !v_isfinite(end_ctrl_point)) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_POSE);
  }
  sg.v = velocity;
  sg.a = acceleration;
  v_pose_split(tp->end_pose, &start_xyz, &start_abc, &start_uvw);
  v_pose_split(end, &end_xyz, &end_abc, &end_uvw);
  #ifdef NO_ABC
  v_setzero(sg.coords.curve.abc.start);
  v_setzero(sg.coords.curve.abc.end);
  v_setzero(sg.coords.curve.abc.u);
  sg.coords.curve.abc.tm = 0;
  #endif
  #ifdef NO_UVW
  v_setzero(sg.coords.curve.uvw.start);
  v_setzero(sg.coords.curve.uvw.end);
  v_setzero(sg.coords.curve.uvw.u);
  sg.coords.curve.uvw.tm = 0;
  #endif
  if(
    v_distance(start_xyz, end_xyz) < TG_RESOLUTION
  ) {
    debug("# tp: trajgen_coord_add_curve(): segment length ~== 0\n");
    return trajgen_coord_raise_error(tp, TP_ERR_SEGMENT_LENGTH_ZERO);
  } else if(
    v_curve_define(&sg.coords.curve.xyz, start_xyz, end_xyz, start_ctrl_point, end_ctrl_point)
    #ifndef NO_ABC
    || v_lin_define(&sg.coords.curve.abc, start_abc, end_abc)
    #endif
    #ifndef NO_UVW
    || v_lin_define(&sg.coords.curve.uvw, start_uvw, end_uvw)
    #endif
  ) {
    return trajgen_coord_raise_error(tp, TP_ERR_INVALID_PARAM);
  }
  sg.length = sg.coords.curve.xyz.len;
  if(sg.length < TG_RESOLUTION) {
    sg.length = 0;
    debug("# tp: trajgen_coord_add_curve(): sg.length~==0\n");
    return trajgen_coord_raise_error(tp, TP_ERR_SEGMENT_LENGTH_ZERO);
  } else if (trajgen_coord_queue_push_back(tp, &sg) != 0) {
    return tp->last_error;
  }
  tp->end_pose = end;
  tp->next_sg_id++;
  tp->is_done = 0;
  return TP_ERR_OK;
  #else
  return TP_ERR_INVALID_MOTION_TYPE;
  #endif
}
 
/**
 * Initialises auxiliary variables of a segment in the queue.
 * @param trajgen_sg_t *sg
 * @return trajgen_coord_error_t
 */
static TG_INLINE trajgen_coord_error_t trajgen_sg_activate(trajgen_sg_t *sg)
{
  CHK_PTR(!sg) return TP_ERR_TP_NULL_POINTER;
  if(sg->is_active) return TP_ERR_OK;
  sg->is_active = 1;
  sg->is_blending = 0;
  sg->blending_velocity = 0.0;
  sg->progress = 0.0;
  sg->current_velocity = 0.0;
  return TP_ERR_OK;
}
 
/**
 * Writes the start/end position of a trajectory segment into pos.
 * @param trajgen_sg_t *sg
 * @param bool_t of_endpoint
 * @param pose_t* pos
 * @return trajgen_coord_error_t
 */
typedef enum { sg_of_curr=0, sg_of_end } trajgen_sg_get_pose_sel;
static trajgen_coord_error_t trajgen_sg_get_pose(trajgen_coord_t* tp, trajgen_sg_t *sg,
    trajgen_sg_get_pose_sel of_endpoint, pose_t* pos)
{
  CHK_PTR(!tp || !sg || !pos) return TP_ERR_TP_NULL_POINTER;
  pose_position_t xyz, abc, uvw;
  real_t progress = of_endpoint==sg_of_end ? sg->length : sg->progress;
  #ifdef NO_ABC
  v_setzero(abc);
  #endif
  #ifdef NO_UVW
  v_setzero(uvw);
  #endif
  switch (sg->motion_type) {
    case SG_LINEAR:
      if (sg->coords.line.xyz.tm > 0.) {
        v_lin_get(&sg->coords.line.xyz, progress, &xyz);
        #ifndef NO_ABC
        v_lin_get(&sg->coords.line.abc, progress * sg->coords.line.abc.tm/sg->length, &abc);
        #endif
        #ifndef NO_UVW
        v_lin_get(&sg->coords.line.uvw, progress * sg->coords.line.uvw.tm/sg->length, &uvw);
        #endif
      #ifndef NO_ABC
      } else if (sg->coords.line.abc.tm > 0.) {
        v_lin_get(&sg->coords.line.abc, progress, &abc);
        xyz = sg->coords.line.xyz.start;
        #ifndef NO_UVW
        uvw = sg->coords.line.uvw.start;
        #endif
      #endif
      #ifndef NO_UVW
      } else if (sg->coords.line.uvw.tm > 0.) {
        v_lin_get(&sg->coords.line.uvw, progress, &uvw);
        xyz = sg->coords.line.xyz.start;
        #ifndef NO_ABC
        abc = sg->coords.line.abc.start;
        #endif
      #endif
      } else {
        xyz = sg->coords.line.xyz.start;
        #ifndef NO_ABC
        abc = sg->coords.line.abc.start;
        #endif
        #ifndef NO_UVW
        uvw = sg->coords.line.uvw.start;
        #endif
      }
      break;
    case SG_CIRCULAR:
      // Args checked in add function
      //v_arc_get(&sg->coords.arc.xyz, progress * sg->coords.arc.xyz.a/sg->length, &xyz);
      v_arc_get(&sg->coords.arc.xyz, progress, &xyz);
      #ifndef NO_ABC
      v_lin_get(&sg->coords.arc.abc, progress * sg->coords.arc.abc.tm/sg->length, &abc);
      #endif
      #ifndef NO_UVW
      v_lin_get(&sg->coords.arc.uvw, progress * sg->coords.arc.uvw.tm/sg->length, &uvw);
      #endif
      break;
    case SG_CURVE:
      #ifdef WITH_CURVES
      // Curve length checked in add function
      v_curve_get(&sg->coords.curve.xyz, progress, &xyz);
      #ifndef NO_ABC
      v_lin_get(&sg->coords.curve.abc, progress * sg->coords.curve.abc.tm/sg->length, &abc);
      #endif
      #ifndef NO_UVW
      v_lin_get(&sg->coords.curve.uvw, progress * sg->coords.curve.uvw.tm/sg->length, &uvw);
      #endif
      break;
      #endif
    default:
      v_invalidate(xyz); v_invalidate(abc); v_invalidate(uvw);
      trajgen_coord_raise_error(tp, TP_ERR_INVALID_MOTION_TYPE);
  }
  v_pose_merge(xyz, abc, uvw, pos);
  return TP_ERR_OK;
}
 
/**
 * Calculates the next linearised position for a segment
 * v= new velocity, a=new acceleration, v0=velocity without override constrain.
 *
 * @param trajgen_sg_t *sg
 * @param real_t *overrun
 * @param real_t *v
 * @param real_t *a
 * @param real_t *v0
 * @return trajgen_coord_error_t
 */
static trajgen_coord_error_t trajgen_sg_tick(trajgen_coord_t* tp, trajgen_sg_t *sg,
    real_t *overrun, real_t *v, real_t *a, real_t *v0)
{
  CHK_PTR(!tp || !sg) return TP_ERR_TP_NULL_POINTER;
  real_t d=0, vn0=0, vn, an, T=tp->config.sample_interval;
  if ((sg->progress >= sg->length)
  || ((d = sg->current_velocity*T + 2.*(sg->progress-sg->length)) >= 0)
  || ((vn0=(sqrtz(T*T/4-d/sg->a) - T/2) * sg->a) < TG_RESOLUTION)
  ) {
    an = vn = vn0 = 0.0;
    d = sg->progress - sg->length;
    sg->progress = sg->length;
  } else {
    if (vn0 > tp->config.max_velocity) vn0 = tp->config.max_velocity;
    if (vn0 > sg->v) vn0 = sg->v;
    if ((vn=vn0) > sg->v * tp->_i.override) vn = sg->v * tp->_i.override;
    an = (vn - sg->current_velocity) / T;
    real_t aa = sg->a ;
    an = (an > aa) ? (aa) : ((an < -aa) ? (-aa) : (an));
    vn = sg->current_velocity + an * T;
    sg->progress += (vn + sg->current_velocity) * T/2;
    d = sg->progress - sg->length;
    if(d >= 0) { sg->progress = sg->length; vn = an = vn0 = 0; }
  }
  sg->current_velocity = vn;
  // if(next_sg && next_sg->progress == 0) next_sg->progress = d;
  if(overrun && d > 0) *overrun = d;
  if (v) *v = vn;
  if (a) *a = an;
  if (v0) *v0 = vn0;
  return TP_ERR_OK;
}
 
/**
 * Perform one planer interval tick.
 * @return trajgen_coord_error_t
 */
COORD_TG_STATIC trajgen_coord_error_t trajgen_coord_tick(trajgen_coord_t* tp)
{
  trajgen_coord_error_t err = TP_ERR_OK;
  trajgen_sg_t *sg, *nextsg;
 
  // Initialisation
  tp->_i.override = tp->override;
  tp->_i.last_position = tp->pose;
 
  // Read the queue front
  if (!(sg = trajgen_coord_queue_front(tp))) {
    // Queue empty, nothing to do
    tp->queue.num_elements = tp->queue.start = tp->queue.end = 0;
    tp->end_pose = tp->pose;
    tp->is_done = 1;
    tp->num_queued_active = tp->sg_id = tp->is_pausing = tp->is_aborting = 0;
    tp->sync.i_offset = tp->sync.i_isset = 0;
    return TP_ERR_OK;
  } else if (sg->progress == sg->length) {
    trajgen_coord_queue_pop_front(tp, 1);
    if (!(sg = trajgen_coord_queue_front(tp))) return TP_ERR_OK;
  }
 
  // Fetch next segment for blending
  // NO_MOTION_BLENDING: Compiler will subsequently remove blocks below as nextsg
  // evaluates to 0.
  nextsg = ((!NO_MOTION_BLENDING) && sg->blending_tolerance>0) ?
    trajgen_coord_queue_get(tp, 1) : (trajgen_sg_t*) 0;
 
  // Abort handling
  if (tp->is_aborting) {
    if (sg->current_velocity == 0.0 && (!nextsg ||
        (nextsg && nextsg->current_velocity == 0.0))) {
      tp->queue.num_elements = tp->queue.start = tp->queue.end = 0;
      tp->end_pose = tp->pose;
      tp->is_done = 1;
      tp->is_aborting = tp->is_pausing = tp->num_queued_active = tp->sg_id =
          tp->sync_dio_current = 0;
      return TP_ERR_OK; // v=0, return possible
    } else {
      sg->v = 0.0;
      sg->a = tp->config.max_acceleration;
      if (nextsg) {
        nextsg->v = 0.0;
        nextsg->a = tp->config.max_acceleration;
      }
    }
  }
 
  // Start of motion (first queue item read or if blending disabled)
  if (!sg->is_active) {
    trajgen_sg_activate(sg);
    tp->sg_id = sg->id;
    tp->num_queued_active = 1;
    tp->sync_dio_current = (tp->sync_dio_current | sg->sync_dio.set) &
        (~sg->sync_dio.clear);
    debug2("# tp: activated segment %lu ...\n", (unsigned long) sg->id);
    if(sg->motion_type != SG_CIRCULAR) debug2("# tp: segment target={%g, %g, %g}\n",
        sg->coords.line.xyz.end.x, sg->coords.line.xyz.end.y, sg->coords.line.xyz.end.z);
  }
 
  // Override initialisation, more override constrains may follow
  if(!sg->override_enabled) tp->_i.override = 1.0;
 
  // Handle next queued segment (only if blending enabled), on syncing
  // blending will be disabled, so this is block executed only once.
  if (nextsg && !nextsg->is_active) {
    #ifndef NO_TRAJECTORY_SYNC
    nextsg->sync_type = TP_SYNC_NONE; // give the compiler a reason to kick all this stuff
    #endif
    // Next segment processing needs to be postponed in case syncing ?
    if((nextsg->sync_type != TP_SYNC_NONE && !isfinite(tp->sync.reference*tp->sync.scaler))
    || (nextsg->sync_type != TP_SYNC_NONE && (tp->sync.scaler == 0))
    || (nextsg->sync_type == TP_SYNC_VELOCITY && tp->sync.reference <= 0)
    ) {
      sg->blending_tolerance = 0;
      nextsg = (trajgen_sg_t*) 0;
    } else {
      // Check if blending is possible, d evaluates to the angle between vectors.
      //
      // stfwi: Find way to optimise acos and cos later. Performance waste.
      //
      pose_vector_t v1, v2; real_t cos_phi, phi;
      if(trajgen_sg_end_direction(tp, sg, &v1) != TP_ERR_OK
      || trajgen_sg_start_direction(tp, nextsg, &v2) != TP_ERR_OK
      ) {
        return trajgen_coord_raise_error(tp, TP_ERR_UNIT_VECTOR_CALC_INVALID_TYPE);
      }
      if((cos_phi = v_dot(v1,v2)) < 0 // double check for erroneous calcs
      || (phi = acos(cos_phi)) > M_PI/180.*MAX_ALLOWED_BLENDING_ANGLE_DEG) {
        debug("# tp: disabled blending for %lu->%lu (angle=%.0fdeg > %.0fdeg)\n",
            sg->id, nextsg->id, phi*(180./M_PI), (real_t)MAX_ALLOWED_BLENDING_ANGLE_DEG);
        sg->blending_tolerance = 0;
        nextsg = (trajgen_sg_t*) 0;
      } else {
        // Blending allowed, precalculate blending activation velocity.
        // Blending rules:
        //  - Only from second half of the current segment to the first half of
        //    the next to prevent 3-segment race conditions.
        //  - Velocity constraints must apply.
        // Triangle speed ramp (without trimmed velocities) --> v^2=s/2 * a.
        //
        // Peak velocity check
        real_t v = sqrtz(nextsg->length/2 * (sg->a<nextsg->a ? sg->a : nextsg->a));
        if(cos_phi > 1) cos_phi = 1; // If unit vectors are a little bit too long
        if(cos_phi > 0 && phi > M_PI/180 * STRAIGHT_BLENDING_ANGLE) {
          real_t vv;
          // Additional tolerance dependent speed constrain
          vv = 2.0 * sqrtz(sg->a * sg->blending_tolerance / cos_phi);
          if(v > vv) v = vv;
          debug2("# tp: activated segment %lu for blending at speed %g...\n",
              (unsigned long) nextsg->id, v);
        } else {
          // Blending in same direction
          // StfWi: Check if more accurate cross product and arcsin is required
          // to validate this.
          sg->blending_tolerance = TG_RESOLUTION; // hint for straight blending
          debug2("# tp: activated segment %lu for 0-angle blending at speed %g...\n",
              (unsigned long) nextsg->id, v);
        }
        if (v > nextsg->v) v = nextsg->v;
        if (v > sg->v) v = sg->v;
        sg->blending_velocity = v;
        // Checkout/activate next segment
        trajgen_sg_activate(nextsg);
        tp->num_queued_active = 2;
        if(sg->motion_type != SG_CIRCULAR) debug2("# tp: segment target={%g, %g, %g}\n",
            nextsg->coords.line.xyz.end.x, nextsg->coords.line.xyz.end.y,
            nextsg->coords.line.xyz.end.z);
      }
    }
  }
 
  // Syncing (manipulates the override depending on an external position or velocity).
  #ifndef NO_TRAJECTORY_SYNC
  tp->sync.i_isset &= (sg->sync_type == TP_SYNC_POSITION);
  if (sg->sync_type == TP_SYNC_VELOCITY && !tp->is_aborting) {
    // Update calculated override depending on the reference speed:
    real_t v = tp->sync.scaler * tp->sync.reference;
    if (tp->is_pausing || !isfinite(v) || v < 0) v = 0;
    if (v < sg->v) tp->_i.override *= (v / sg->v);
  } else if (sg->sync_type == TP_SYNC_POSITION) {
    // Stfwi: CHECK: How's the abort behaviour supposed to be ?
    //  - Thread tapping: critical to stay in sync <-- vs. -->
    //  - Touch probe position tune-in: estop important.
    if (sg->progress >= sg->length) {
      // Segment overrun/finished, initialise sync flag for next one:
      tp->sync.i_isset = isfinite(tp->sync.reference*tp->sync.scaler) && tp->sync.scaler!=0;
      tp->sync.i_offset += tp->sync.i_isset ? sg->length : 0.0;
    } else if (!tp->sync.i_isset && isfinite(tp->sync.reference*tp->sync.scaler)
        && tp->sync.scaler!=0) {
      // Ref position now validated, initialise position offset:
      tp->sync.i_offset = (tp->sync.reference * tp->sync.scaler) - sg->progress;
      tp->sync.i_isset = 1;
    }
    // Determine the new override value for this cycle
    if (!tp->sync.i_isset) {
      tp->_i.override = 0;
    } else {
      // ds/dt = 1/2 a * dt + d^2s   ---(d^2s===dv)--->  dv = ds/dt - a * dt/2
      real_t ds = (tp->sync.reference * tp->sync.scaler) - tp->sync.i_offset
                    - sg->progress - (nextsg ? nextsg->progress : 0.0);
      if(!isfinite(ds) || tp->sync.scaler==0) {
        // Reference position was invalidated during the motion.
        err = trajgen_coord_raise_error(tp, TP_ERR_REF_POSITION_INVALIDATED_DURING_MOTION);
        trajgen_coord_abort(tp);
        tp->_i.override = 0;
      } else if (ds < 0) {
        // Trajectory is ahead of reference
        tp->_i.override = 0;
      } else {
        // Reference is ahead or matches
        real_t T = tp->config.sample_interval;
        real_t dv = ds / T - sg->a * T / 2.0;
        tp->_i.override = (sg->current_velocity + dv) / sg->v;
        tp->_i.override = (tp->_i.override < 0) ? 0 : ((tp->_i.override > 1) ? 1 : tp->_i.override);
      }
    }
  }
  else // else below macro end : End TP_SYNC_POSITION || (TP_SYNC_VELOCITY && !is_aborting)
  #endif
  if (tp->is_pausing) {
    tp->_i.override = 0;
  }
  if (!nextsg) {
    // No next --> no blending --> quickly calculate next pose and return.
    trajgen_sg_tick(tp, sg, 0,0,0,0);
    trajgen_sg_get_pose(tp, sg, sg_of_curr, &tp->pose);
    return err;
  } else if(!sg->is_blending) {
    // Check when to start blending
    real_t v, vn;
    trajgen_sg_tick(tp, sg, 0, &v, 0, &vn);
    trajgen_sg_get_pose(tp, sg, sg_of_curr, &tp->pose);
    if((sg->progress/sg->length > 0.5) && (vn < sg->blending_velocity)) {
      // Blending_velocity changes its meaning if blending=1:
      // sg->blending_velocity = sg->current_velocity;
      nextsg->current_velocity = sg->blending_velocity - sg->current_velocity;
      sg->is_blending = 1;
      debug2("# tp: start blending segment %lu -> %lu...\n", (unsigned long) sg->id,
          (unsigned long) nextsg->id);
    }
  } else {
    // Blending in progress
    real_t v;
    pose_t last_pose, curr_pose, nextsg_last_pose, nextsg_curr_pose;
    trajgen_sg_get_pose(tp, sg, sg_of_curr, &last_pose);
    trajgen_sg_tick(tp, sg, 0, &v, 0, 0);
    trajgen_sg_get_pose(tp, sg, sg_of_curr, &curr_pose);
    if (tp->sg_id != nextsg->id && sg->current_velocity < nextsg->current_velocity) {
      tp->sync_dio_current = (tp->sync_dio_current|nextsg->sync_dio.set)
          & (~nextsg->sync_dio.clear);
      tp->sg_id = nextsg->id;
      debug2("# tp: switched to segment %lu ...\n", (unsigned long) tp->sg_id);
    }
    trajgen_sg_get_pose(tp, nextsg, sg_of_curr, &nextsg_last_pose);
    real_t vv = nextsg->v;
    nextsg->v = sg->blending_velocity - v;
    nextsg->v = nextsg->v < 0.0 ? 0 : nextsg->v;
    trajgen_sg_tick(tp, nextsg, 0,0,0,0);
    nextsg->v = vv;
    trajgen_sg_get_pose(tp, nextsg, sg_of_curr, &nextsg_curr_pose);
    // Add increments of segment and nextsg to the position
    pose_acc(&tp->pose, curr_pose);
    pose_sub(&tp->pose, last_pose);
    pose_acc(&tp->pose, nextsg_curr_pose);
    pose_sub(&tp->pose, nextsg_last_pose);
  }
  return err;
}
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="trajgen_jointtg_*: Free single joint planer">
 
/********************************************************************
 *
 * Simple single axis trajectory planer, used for individual joints.
 *
 ********************************************************************/
 
static trajgen_jointtg_error_t trajgen_jointtg_err(trajgen_jointtg_t *tp, trajgen_jointtg_error_t e)
{ if (!tp) return e; return tp->last_error = e; }
 
trajgen_jointtg_error_t trajgen_jointtg_initialize(trajgen_jointtg_t *tp, trajgen_jointtg_config_t config)
{
  CHK_PTR(!tp) return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_NULLPOINTER);
  tp->config = config;
  tp->velocity = tp->position = 0.0;
  tp->is_enabled = 0;
  tp->last_error = TRAJGEN_FREE_ERROR_OK;
  if (config.max_acceleration < 0.0) {
    return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_INVALID_MAX_ACCEL);
  } else if (config.max_velocity < 0.0) {
    return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_INVALID_MAX_VELOCITY);
  } else if (config.sample_interval <= 0) {
    return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_INVALID_SAMPLE_INTERVAL);
  }
  return TRAJGEN_FREE_ERROR_OK;
}
 
JOINT_TG_STATIC trajgen_jointtg_error_t trajgen_jointtg_reset(trajgen_jointtg_t *tp)
{
  CHK_PTR(!tp) return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_NULLPOINTER);
  tp->velocity = 0.0;
  tp->command_position = tp->position;
  tp->last_error = TRAJGEN_FREE_ERROR_OK;
  tp->is_pause = 0;
  tp->is_done = 1;
  return TRAJGEN_FREE_ERROR_OK;
}
 
JOINT_TG_STATIC trajgen_jointtg_error_t trajgen_jointtg_set_enabled(trajgen_jointtg_t *tp,
    bool_t enabled)
{
  CHK_PTR(!tp) return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_NULLPOINTER);
  if (enabled == tp->is_enabled) {
    return TRAJGEN_FREE_ERROR_OK;
  } else if (!enabled) {
    tp->is_enabled = 0;
  } else {
    if (tp->is_done) trajgen_jointtg_reset(tp);
    tp->is_enabled = 1;
  }
  return TRAJGEN_FREE_ERROR_OK;
}
 
JOINT_TG_STATIC trajgen_jointtg_error_t trajgen_jointtg_set_command_position(trajgen_jointtg_t *tp,
    real_t position)
{ tp->command_position = position; return TRAJGEN_FREE_ERROR_OK; }
 
JOINT_TG_STATIC trajgen_jointtg_error_t trajgen_jointtg_set_command_velocity(trajgen_jointtg_t *tp,
    real_t velocity)
{
  CHK_PTR(!tp) return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_NULLPOINTER);
  if (velocity > tp->config.max_velocity) velocity = tp->config.max_velocity;
  else if (velocity < -tp->config.max_velocity) velocity = -tp->config.max_velocity;
  tp->command_velocity = velocity;
  return TRAJGEN_FREE_ERROR_OK;
}
 
JOINT_TG_STATIC real_t trajgen_jointtg_get_current_position(trajgen_jointtg_t *tp)
{ return tp->position; }
 
JOINT_TG_STATIC trajgen_jointtg_error_t trajgen_jointtg_set_current_position(trajgen_jointtg_t *tp,
    real_t position)
{ tp->position = position; return TRAJGEN_FREE_ERROR_OK; }
 
JOINT_TG_STATIC bool_t trajgen_jointtg_is_done(trajgen_jointtg_t *tp)
{ return tp->is_done && tp->velocity == 0.0; }
 
JOINT_TG_STATIC trajgen_jointtg_error_t trajgen_jointtg_tick(trajgen_jointtg_t *tp)
{
  CHK_PTR(!tp) return trajgen_jointtg_err(tp, TRAJGEN_FREE_ERROR_INIT_NULLPOINTER);
  if(tp->command_velocity < 0) tp->is_enabled = 0; // Force disable
  if(!tp->is_enabled) tp->command_position = tp->position; // Force break
  if (tp->is_enabled || tp->velocity != 0 || tp->command_position != tp->position) {
    real_t T = tp->config.sample_interval;
    real_t dp = tp->command_position - tp->position;
    real_t vd = tp->config.max_acceleration * T;
    real_t dir = dp > 0 ? 1 : -1;
    real_t v;
    // Velocity required using s = 1/2 a t^2 + v0 * t + s0; v^2 = s*a
    dp *= dir; // Work with positive values
    if(dp < TG_RESOLUTION) dp = 0;
    v = (dp <= 0) ? 0 : (sqrtf(2.0 * tp->config.max_acceleration*dp + vd*vd)-vd);
    if(tp->is_pause) v = 0;
    if(v > tp->config.max_velocity && tp->config.max_velocity > 0) v = tp->config.max_velocity;
    if(v > tp->command_velocity && tp->command_velocity > 0) v = tp->command_velocity;
    if((v*=dir) > tp->velocity + vd) {
      tp->velocity += vd;
    } else if (v < tp->velocity - vd) {
      tp->velocity -= vd;
    } else {
      tp->velocity = v;
    }
    tp->position += tp->velocity * T;
    if (v < TG_RESOLUTION && fabs(tp->position - tp->command_position) < TG_RESOLUTION) {
      tp->position = tp->command_position;
      tp->velocity = 0.0;
    }
  }
  tp->is_done = (tp->velocity == 0.0 && tp->position == tp->command_position);
  return TRAJGEN_FREE_ERROR_OK;
}
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="trajgen_man_*: Manual operation planer (tele operation)">
 
/********************************************************************
 *
 *
 * Manual operation planer (tele operation)
 *
 *
 ********************************************************************/
 
static trajgen_man_error_t trajgen_man_err(trajgen_man_t *mtp, trajgen_man_error_t e)
{ if (!mtp) return e; return mtp->last_error = e; }
 
MANUAL_TG_STATIC trajgen_man_error_t trajgen_man_initialize(trajgen_man_t *_tp,
    trajgen_man_config_t config)
{
  CHK_PTR(!_tp) return TG_MAN_ERROR_INIT_NULLPOINTER;
  ZERO_MEMORY(_tp, sizeof (trajgen_man_t)); // Set all data here to 0
  if (!pose_is_all_greater_equal_zero(config.max_acceleration)) {
    return trajgen_man_err(_tp, TG_MAN_ERROR_INIT_INVALID_MAX_ACCEL);
  } else if (!pose_is_all_greater_equal_zero(config.max_velocity)) {
    return trajgen_man_err(_tp, TG_MAN_ERROR_INIT_INVALID_MAX_VELOCITY);
  } else if (config.sample_interval <= 0) {
    return trajgen_man_err(_tp, TG_MAN_ERROR_INIT_INVALID_SAMPLE_INTERVAL);
  }
  _tp->config = config;
  return TG_MAN_ERROR_OK;
}
 
MANUAL_TG_STATIC trajgen_man_error_t trajgen_man_reset(trajgen_man_t *mtp)
{
  pose_set_zero(&mtp->velocity);
  pose_set_zero(&mtp->current_velocity);
  mtp->is_enabled = 0;
  mtp->last_error = TG_MAN_ERROR_OK;
  return TG_MAN_ERROR_OK;
}
 
MANUAL_TG_STATIC trajgen_man_error_t trajgen_man_abort(trajgen_man_t *mtp)
{ mtp->is_enabled = 0; return TG_MAN_ERROR_OK; }
 
MANUAL_TG_STATIC bool_t trajgen_man_is_done(trajgen_man_t *mtp)
{ return pose_is_zero(mtp->velocity) && pose_is_zero(mtp->current_velocity); }
 
MANUAL_TG_STATIC pose_t trajgen_man_get_position(trajgen_man_t *mtp)
{ return mtp->pose; }
 
MANUAL_TG_STATIC trajgen_man_error_t trajgen_man_set_position(trajgen_man_t *mtp,
    pose_t position)
{ mtp->pose = position; return TG_MAN_ERROR_OK; }
 
MANUAL_TG_STATIC trajgen_man_error_t trajgen_man_set_velocity(trajgen_man_t *mtp,
    pose_t speed)
{
  pose_trim_all_upper(&speed, mtp->config.max_velocity);
  pose_neg(&speed);
  pose_trim_all_upper(&speed, mtp->config.max_velocity);
  pose_neg(&speed);
  mtp->velocity = speed;
  return TG_MAN_ERROR_OK;
}
 
MANUAL_TG_STATIC trajgen_man_error_t trajgen_man_tick(trajgen_man_t *mtp)
{
  pose_t dv; real_t T=mtp->config.sample_interval;
  if (!mtp->is_enabled) pose_set_zero(&mtp->velocity);
  // Determine maximum required acceleration to obtain the speed in one cycle,
  // including the polar axes
  pose_diff(mtp->velocity, mtp->current_velocity, &dv);
  if (!pose_is_zero(dv)) {
    real_t d, scl = 1.;
    // Something changed
    pose_scale(&dv, 1./T);
    // Accelerations for.x/y/z can be pre-checked with the trajectory acceleration
    if (dv.x != 0 && (d = fabs(mtp->config.max_acceleration.x / dv.x)) < scl) scl = d;
    if (dv.y != 0 && (d = fabs(mtp->config.max_acceleration.y / dv.y)) < scl) scl = d;
    if (dv.z != 0 && (d = fabs(mtp->config.max_acceleration.z / dv.z)) < scl) scl = d;
    #ifndef NO_ABC
    if (dv.a != 0 && (d = fabs(mtp->config.max_acceleration.a / dv.a)) < scl) scl = d;
    if (dv.b != 0 && (d = fabs(mtp->config.max_acceleration.b / dv.b)) < scl) scl = d;
    if (dv.c != 0 && (d = fabs(mtp->config.max_acceleration.c / dv.c)) < scl) scl = d;
    #else
    dv.a = dv.b = dv.c = 0;
    #endif
    #ifndef NO_UVW
    if (dv.u != 0 && (d = fabs(mtp->config.max_acceleration.u / dv.u)) < scl) scl = d;
    if (dv.v != 0 && (d = fabs(mtp->config.max_acceleration.v / dv.v)) < scl) scl = d;
    if (dv.w != 0 && (d = fabs(mtp->config.max_acceleration.w / dv.w)) < scl) scl = d;
    #else
    dv.u = dv.v = dv.w = 0;
    #endif
    if (scl < 1.0) {
      d = scl * T;       // Scale the speed down to the maximum acceleration allowed
      pose_scale(&dv, d); // Integrate the new velocity
      pose_acc(&mtp->current_velocity, dv);
    } else {
      // We reached the requested velocity in the actual sample period. accel
      // is now near zero. So we round to the requested velocity.
      mtp->current_velocity = mtp->velocity;
    }
  }
  dv = mtp->current_velocity;
  pose_scale(&dv, T); // Integrate position
  pose_acc(&mtp->pose, dv);
  return TG_MAN_ERROR_OK;
}
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="trajgen_*: Main trajectory generation coordinator">
 
/*******************************************************************************
 *
 * Main trajectory generation coordinator. Manages Free planers for all joints,
 * manual and coordinated planer, including errors and switching, interpolating,
 * and trajgen position updating.
 *
 * It is in the very end a finite state machine based organisation of the
 * available functionalities.
 *
 ******************************************************************************/
 
static trajgen_t *tg;
 
#define MTG (&tg->man_planer)
#define CTG (&tg->coord_planer)
 
/**
 * Generates the last_error variable and forwards the code passed as argument.
 * @param source
 * @param code
 * @param joint
 * @return
 */
static trajgen_error_t tg_err(unsigned code, unsigned joint)
{
  trajgen_error_details_t e;
  if (!tg) return (trajgen_error_t) code;
  e.errno = 0;
  e.sel.joint = joint;
  e.sel.code = code;
  tg->last_error = e;
  #ifdef TG_DEBUG_LEVEL
  if(code) {
    char s[128]; s[0]=s[127]='\0'; trajgen_errstr(tg->last_error.errno, s, 127);
    debug("# tg: trajgen_err(): ! Error: %s\n", s);
  }
  #endif
  return (trajgen_error_t) code;
}
 
/* not static but not exported in header, linker shall drop it if unused */
const char* trajgen_state_name(trajgen_state_t s)
{
  if(s<0 || s>TRAJ_STATE_TG_MAN_LEAVE) return "INVALID";
  static const char *names[] = {
    "disabled", "jointtg", "coordinated", "manual", "INVALID",
    "(ok-to-switch)", "(disabling)", "(jointtg enter)", "(jointtg leave)",
    "(coordinated enter)", "(coordinated leave)", "(manual enter)",
    "(manual leave)"
  };
  return names[s];
}
 
#define CHKERR(X) tg_err((X), 0)
 
/* Coord wrappers */
trajgen_error_t trajgen_pause()
{
  for(int i=0; i<tg->config.number_of_used_joints; i++) {
    tg->joints[i].tg.is_pause = 1;
  }
  return CHKERR(trajgen_coord_pause(CTG));
}
 
trajgen_error_t trajgen_resume()
{
  for(int i=0; i<tg->config.number_of_used_joints; i++) {
    tg->joints[i].tg.is_pause = 0;
  }
  return CHKERR(trajgen_coord_resume(CTG));
}
 
bool_t trajgen_queue_full()
{ return trajgen_coord_is_queue_full(CTG); }
 
uint32_t trajgen_current_id()
{ return trajgen_coord_get_current_id(CTG); }
 
uint16_t trajgen_num_queued()
{ return trajgen_coord_get_num_queued(CTG); }
 
uint16_t trajgen_queue_size()
{ return trajgen_coord_get_queue_size(CTG); }
 
trajgen_error_t trajgen_add_line(pose_t end, real_t velocity, real_t acceleration)
{ return CHKERR(trajgen_coord_add_line(CTG, end, velocity, acceleration)); }
 
trajgen_error_t trajgen_add_arc(pose_t end, pose_position_t center, pose_vector_t normal,
    unsigned n_turns, real_t velocity, real_t acceleration)
{ return CHKERR(trajgen_coord_add_arc(CTG, end, center, normal, n_turns, velocity,
    acceleration)); }
 
trajgen_coord_error_t trajgen_add_curve(pose_t end, pose_vector_t start_ctrl_point,
    pose_vector_t end_ctrl_point, real_t velocity, real_t acceleration)
{ return CHKERR(trajgen_coord_add_curve(CTG, end, start_ctrl_point, end_ctrl_point,
    velocity, acceleration)); }
 
/* Free wrappers */
trajgen_error_t trajgen_joint_command_velocity(unsigned joint, real_t v)
{
  if(joint > tg->config.number_of_used_joints) return CHKERR(TRAJ_ERROR_INVALID_JOINT_NO);
  return CHKERR(trajgen_jointtg_set_command_velocity(&tg->joints[joint].tg, v));
}
 
#undef CHKERR
 
/**
 * Returns the current tick counter of the TG
 * @return uint32_t
 */
uint32_t trajgen_get_tick()
{ return tg->_internals.tick; }
 
/**
 * Abort / stop (all planers)
 * @return trajgen_error_t
 */
trajgen_error_t trajgen_abort()
{
  debug("# tg: trajgen_abort()\n");
  unsigned i;
  trajgen_coord_abort(CTG);
  trajgen_man_abort(MTG);
  for (i=0; i < TG_MAX_JOINTS; i++) tg->joints[i].tg.is_enabled = 0;
  tg->is_done = 0;
  tg->requested_state = TRAJ_STATE_DISABLED;
  return TRAJ_ERROR_OK;
}
 
/**
 * Sets the new requested trajgen state.
 * @param trajgen_state_t state
 * @return trajgen_error_t
 */
trajgen_error_t trajgen_switch_state(trajgen_state_t state)
{
  if (state < TRAJ_STATE_DISABLED || state >= TRAJ_______INTERNAL_STATES) {
    return tg_err(TRAJ_ERROR_INVALID_SWITCHING_STATE, 0);
  }
  tg->is_done = 0;
  tg->requested_state = state;
  debug("# tg: trajgen_switch_state(%s)\n", trajgen_state_name(state));
  return TRAJ_ERROR_OK;
}
 
/**
 * Initializes the trajectory generator and all sub systems. This function
 * MUST BE CALLED BEFORE ANY OTHER OPERATIONS with the trajgen, as internal
 * pointer require initialization.
 * @param trajgen_t *trajectory_generator_data_location
 * @param trajgen_config_t config
 * @return trajgen_error_t
 */
trajgen_error_t trajgen_initialize(trajgen_t *trajgen_data_location, trajgen_config_t config)
{
  int i, err = 0;
  debug("# tg: trajgen_initialize(...)\n");
  // Assign basics
  tg = trajgen_data_location;
  if (!tg) return tg_err(TRAJ_ERROR_NULL_POINTER, 0);
  ZERO_MEMORY(tg, sizeof (trajgen_t));
 
  // Fixed configuration checks (in case the compiler does not raise errors before)
  if (FLOAT_EPS <= 0
  ||  TG_RESOLUTION <= FLOAT_EPS // stfwi: intentionally at least 2*FLOAT_EPS
  ||  TG_D_RES <= FLOAT_EPS
  ||  TG_A_RES <= FLOAT_EPS
  ||  TG_QUEUE_SIZE < 4
  ||  TG_MAX_JOINTS > JOINT_MAX_JOINTS
  ||  sizeof(real_t) < sizeof(float)
  ){
    return tg_err(TRAJ_ERROR_CONFIG_COMPILE_SETTING_INVALID, 0);
  }
  // Configuration checks
  if (config.number_of_used_joints > TG_MAX_JOINTS || !config.number_of_used_joints) {
    return tg_err(TRAJ_ERROR_CONFIG_LAST_USED_JOINT_INVALID, 0);
  }
  if(!config.interpolation_rate) {
    return tg_err(TRAJ_ERROR_CONFIG_INTERPOLATION_RATE_INVALID, 0);
  }
  // Coordinated planer initialisation
  {
    trajgen_coord_config_t trajgen_coord_config;
    trajgen_coord_config.sample_interval = config.sample_interval * config.interpolation_rate;
    trajgen_coord_config.max_velocity = config.max_coord_velocity;
    trajgen_coord_config.max_acceleration = config.max_coord_acceleration;
    if ((err = trajgen_coord_init(&(tg->coord_planer), trajgen_coord_config)) != 0) {
      return tg_err(err, 0);
    }
  }
  // Manual op planer initialisation
  {
    trajgen_man_config_t trajgen_man_config;
    trajgen_man_config.max_acceleration = config.max_manual_accelerations;
    trajgen_man_config.max_velocity = config.max_manual_velocities;
    trajgen_man_config.sample_interval = config.sample_interval * config.interpolation_rate;
    if ((err = trajgen_man_initialize(MTG, trajgen_man_config)) != 0) {
      return tg_err(err, 0);
    }
  }
  // Cubic interpolators and joints
  for (i = 0; i < TG_MAX_JOINTS; i++) {
    config.joints[i].sample_interval = config.sample_interval * config.interpolation_rate;
    tg->_internals.joint_positions[i] = &(tg->joints[i].position);
    tg->_internals.planer_positions[i] = &(tg->joints[i].planer_position);
    if (i >= config.number_of_used_joints) {
      config.joints[i].max_acceleration = config.joints[i].max_velocity = 0;
    }
    // Interpolators are all standard initialised
    if ((err = interpolator_init(&(tg->joints[i].interpolator), config.sample_interval
        *config.interpolation_rate, config.interpolation_rate)) != 0) {
      return tg_err(err, i);
    }
    // Free joint trajgens
    if ((err = trajgen_jointtg_initialize(&tg->joints[i].tg, config.joints[i])) != 0) {
      return tg_err(err, i);
    }
  }
  // Kinematics
  if ((err = kinematics_initialize(&(tg->kinematics), config.kinematics_functions)) != 0) {
    return tg_err(err, 0);
  }
  tg->config = config;
  // Reset state
  trajgen_reset();
  return TRAJ_ERROR_OK;
}
 
/**
 * Resets the whole trajgen
 * @return trajgen_error_t
 */
trajgen_error_t trajgen_reset()
{
  debug("# tg: trajgen_reset()\n");
 
  // Reset axes trajgens and interpolators
  unsigned i;
  for (i = 0; i < tg->config.number_of_used_joints; i++) {
    tg->_internals.planer_positions[i] = tg->_internals.joint_positions[i];
    tg->joints[i].tg.command_position = tg->joints[i].position;
    trajgen_jointtg_reset(&tg->joints[i].tg);
    interpolator_reset(&tg->joints[i].interpolator, tg->joints[i].planer_position);
  }
 
  // Update planer pose from planer position
  tg->kinematics.forward(tg->_internals.joint_positions, &tg->planer_pose);
  tg->_internals.tick = 0;
 
  // Reset coordinated trajgens
  trajgen_coord_reset(CTG);
  trajgen_man_reset(MTG);
 
  // Reset state and status
  tg->is_done = 1;
  tg->state = TRAJ_STATE_DISABLED;
  return TRAJ_ERROR_OK;
}
 
/**
 * Sets the planer poses and positions corresponding to the current joint
 * positions.
 * @return trajgen_error_t
 */
static TG_INLINE trajgen_error_t trajgen_reset_positions()
{
  int err, i;
  for (i=0; i < tg->config.number_of_used_joints; i++) {
    tg->joints[i].planer_position = tg->joints[i].tg.command_position =
        tg->joints[i].position;
    tg->joints[i].acceleration = tg->joints[i].velocity = tg->joints[i].jerk = 0;
    interpolator_reset(&tg->joints[i].interpolator, tg->joints[i].position);
  }
  if ((err = tg->kinematics.forward(tg->_internals.joint_positions, &tg->planer_pose)) != 0) {
    return tg_err(err, 0);
  }
  tg->kinematics.forward(tg->_internals.joint_positions, &tg->planer_pose);
  trajgen_coord_set_position(CTG, tg->planer_pose);
  trajgen_man_set_position(MTG, tg->planer_pose);
  return TRAJ_ERROR_OK;
}
 
/**
 * Called frequently with the sample period tg.config.sample_interval.
 * @return trajgen_error_t
 */
trajgen_error_t trajgen_tick()
{
  int err, i;
  #if TG_DEBUG_LEVEL > 1
  static trajgen_state_t _dbg_state = TRAJ_STATE_DISABLED;
  if(_dbg_state != tg->state) {
    debug2("# tg: trajgen_tick(): state change: %s --> %s\n", trajgen_state_name(_dbg_state),
        trajgen_state_name(tg->state));
    _dbg_state = tg->state;
  }
  #endif
 
  tg->_internals.tick++;
 
  // Commanded state switching
  if (tg->state != tg->requested_state) {
    switch (tg->state) {
      case TRAJ_STATE_DISABLED:
      case TRAJ_STATE_OK_TO_SWITCH:
        switch (tg->requested_state) {
          case TRAJ_STATE_COORDINATED:
            tg->state = TRAJ_STATE_COORDINATED_ENTER;
            tg->is_done = 0;
            break;
          case TRAJ_STATE_JOINT:
            tg->state = TRAJ_STATE_JOINT_ENTER;
            tg->is_done = 0;
            break;
          case TRAJ_STATE_MAN:
            tg->state = TRAJ_STATE_TG_MAN_ENTER;
            tg->is_done = 0;
            break;
          case TRAJ_STATE_DISABLED:
            // Prevent error below
            break;
          default:
            tg->requested_state = TRAJ_STATE_DISABLED;
            return tg_err(TRAJ_ERROR_INVALID_SWITCHING_STATE, 0);
        }
        break;
      case TRAJ_STATE_COORDINATED:
        tg->state = TRAJ_STATE_COORDINATED_LEAVE;
        break;
      case TRAJ_STATE_JOINT:
        tg->state = TRAJ_STATE_JOINT_LEAVE;
        break;
      case TRAJ_STATE_MAN:
        tg->state = TRAJ_STATE_TG_MAN_LEAVE;
        break;
      default:
        ; // This will be caught below
    }
  }
 
  // State execution and implicit switching
  switch (tg->state) {
    case TRAJ_STATE_COORDINATED_ENTER:
    {
      // Reset the coord planer position to the main generator position.
      // (if jogging/manual/axis independent motions were done before).
      // Update planer pose from joints
      trajgen_reset_positions();
      tg->state = TRAJ_STATE_COORDINATED;
      tg->is_done = 1;
      break;
    }
    case TRAJ_STATE_COORDINATED_LEAVE:
    {
      // Stop the motion, trajgen_coord_abort() can be called every cycle until
      // the motion is done.
      trajgen_coord_abort(CTG);
      // After finishing, switch to disabled preparation state
      // stfwi: CHECK: TIMEOUT TO PREVENT INTERNAL ERROR CASES ?
      if (tg->is_done) {
        tg->state = TRAJ_STATE_OK_TO_SWITCH;
      }
      // NO BREAK, the planer has to run until the motion is finished
    }
    case TRAJ_STATE_COORDINATED:
    {
      trajgen_joint_t *joint;
      // The trajectory is defined using the functions in trajgen_coordinated.h,
      // mainly trajgen_coord_add_line() and trajgen_coord_add_circle().
      // Generate new coarse joint positions for the interpolator
      if (interpolator_need_update(&(tg->joints[0].interpolator))) {
        if ((err = trajgen_coord_tick(CTG)) != 0) {
          return tg_err(err, 0);
        }
        tg->planer_pose = trajgen_coord_get_position(CTG);
        if ((err = tg->kinematics.inverse(&(tg->planer_pose), tg->_internals.planer_positions)) != 0) {
          return tg_err(err, 0);
        }
        for (i = 0; i < tg->config.number_of_used_joints; i++) {
          if ((err = interpolator_push(&(tg->joints[i].interpolator),
              *tg->_internals.planer_positions[i])) != 0) {
            return tg_err(err, i);
          }
        }
      }
      bool_t v0 = 1;
      if (!interpolator_need_update(&(tg->joints[0].interpolator))) {
        // Interpolate (every sample interval)
        for (i = 0; i < tg->config.number_of_used_joints; i++) {
          joint = &tg->joints[i];
          if ((err = interpolator_interpolate(&(joint->interpolator), &(joint->position),
              &(joint->velocity), &(joint->acceleration), &(joint->jerk))) != 0) {
            return tg_err(err, i);
          }
          v0 &= (joint->velocity < TG_RESOLUTION);
          if(!isfinite(joint->position)) {
            return tg_err(TRAJ_ERROR_NUMERIC, i);
          }
        }
      }
      // Transfer state variables
      tg->is_done = trajgen_coord_is_done(CTG) && v0;
      break;
    }
 
    ////////////////////////////////////////////////////////////////////////
 
    case TRAJ_STATE_JOINT_ENTER:
    {
      int i;
      trajgen_reset_positions();
      for (i = 0; i < tg->config.number_of_used_joints; i++) {
        if((err = trajgen_jointtg_reset(&tg->joints[i].tg))) {
          tg_err(err, i);
        } else {
          if(tg->config.joints[i].max_acceleration > 0
          && tg->config.joints[i].max_velocity > 0) {
            tg->joints[i].tg.is_enabled = 1;
            tg->joints[i].tg.is_done = 0;
          } else {
            tg->joints[i].tg.is_enabled = 0;
          }
        }
      }
      tg->state = TRAJ_STATE_JOINT;
      break;
    }
    case TRAJ_STATE_JOINT_LEAVE:
    {
      int i;
      for (i = 0; i < tg->config.number_of_used_joints; i++) {
        tg->joints[i].tg.is_enabled = 0;
      }
      if (tg->is_done) {
        tg->state = TRAJ_STATE_OK_TO_SWITCH;
      }
    }
    case TRAJ_STATE_JOINT:
    {
      int i, n_done = 0;
      trajgen_joint_t *joint;
      for (i = 0; i < tg->config.number_of_used_joints; i++) {
        joint = &tg->joints[i];
        if (interpolator_need_update(&(joint->interpolator))) {
          if ((err = trajgen_jointtg_tick(&joint->tg)) != 0) {
            return tg_err(err, i);
          }
          joint->planer_position = joint->tg.position;
          if (joint->tg.is_done) {
            n_done++;
          }
          if ((err = interpolator_push(&(joint->interpolator), joint->planer_position)) != 0) {
            return tg_err(err, i);
          }
        }
        if (interpolator_interpolate(&(joint->interpolator), &(joint->position),
            &(joint->velocity), &(joint->acceleration), &(joint->jerk)) != 0) {
          return tg_err(err, i);
        }
        if(!isfinite(joint->position)) {
          return tg_err(TRAJ_ERROR_NUMERIC, i);
        }
      }
      tg->is_done = n_done >= tg->config.number_of_used_joints;
      break;
    }
 
    ////////////////////////////////////////////////////////////////////////
 
    case TRAJ_STATE_TG_MAN_ENTER:
    {
      // Reset velocities/accelerations
      trajgen_man_reset(MTG);
      // Update planer pose
      trajgen_reset_positions();
      // Enable
      MTG->is_enabled = 1;
      // Switch state
      tg->state = TRAJ_STATE_MAN;
      break;
    }
    case TRAJ_STATE_TG_MAN_LEAVE:
    {
      // Force manual op command velocity to zero
      trajgen_man_abort(MTG);
      // If the axes are not moving anymore, switch state
      if (tg->is_done) {
        tg->state = TRAJ_STATE_OK_TO_SWITCH;
      }
      // NO BREAK HERE
    }
    case TRAJ_STATE_MAN:
    {
      int i;
      trajgen_joint_t *joint;
      // The pose_t variable tg->trajgen_man_planer,_planer->velocity shall
      // contain the new requested velocities. This variable shall be set every
      // before calling trajgen_tick(), e.g. transferring scaled joystick
      // inputs, velocity command from the computer keyboard etc.
      while (interpolator_need_update(&(tg->joints[0].interpolator))) {
        if ((err = trajgen_man_tick(MTG)) != 0) {
          return tg_err(err, 0);
        }
        tg->planer_pose = trajgen_man_get_position(MTG);
        if ((err = tg->kinematics.inverse(&(tg->planer_pose),
            tg->_internals.planer_positions)) != 0) {
          return tg_err(err, 0);
        }
        for (i = 0; i < tg->config.number_of_used_joints; i++) {
          if ((err = interpolator_push(&(tg->joints[i].interpolator),
              *tg->_internals.planer_positions[i])) != 0) {
            return tg_err(err, i);
          }
        }
      }
 
      // Interpolate (every sample interval)
      for (i = 0; i < tg->config.number_of_used_joints; i++) {
        joint = &tg->joints[i];
        interpolator_interpolate(&(joint->interpolator), &(joint->position),
            &(joint->velocity), &(joint->acceleration), &(joint->jerk));
        if(!isfinite(joint->position)) {
          return tg_err(TRAJ_ERROR_NUMERIC, i);
        }
      }
      // Transfer state variables
      tg->is_done = trajgen_man_is_done(MTG);
      break;
    }
 
    ////////////////////////////////////////////////////////////////////////
 
    case TRAJ_STATE_DISABLING:
    {
      // Reset the trajgen, which does not include the config or actual
      // joint values.
      trajgen_reset();
      // Proceed directly in DISABLED state
      tg->state = TRAJ_STATE_DISABLED;
      // NO BREAK
    }
    case TRAJ_STATE_DISABLED:
    {
      tg->is_done = 1;
      // State block reserved for updating states
    }
    case TRAJ_STATE_OK_TO_SWITCH:
    {
      // Planer position needs to be updated externally
      break;
    }
 
    ////////////////////////////////////////////////////////////////////////
 
    default:
    {
      tg->state = TRAJ_STATE_DISABLING;
      return tg_err(TRAJ_ERROR_INVALID_STATE, 0);
    }
  }
  return TRAJ_ERROR_OK;
}
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="trajgen_errstr/info: Error to string conversion">
 
/********************************************************************
 *
 * Trajectory generator error to string conversion (no standard libs required)
 * Trajectory generator info string
 *
 ********************************************************************/
 
#define sprinterr(_T) do {\
  if(_T) for(tp=_T; (*tp) && (errstr_p != errstr_e); errstr_p++, tp++) *errstr_p=*tp; \
} while(0);
 
#define add_joint() do {\
  if(errstr_p != errstr_e) { *errstr_p='['; errstr_p++; } \
  if(errstr_p != errstr_e) { *errstr_p='0'+(err.sel.joint%10); errstr_p++; } \
  if(errstr_p != errstr_e) { *errstr_p=']'; errstr_p++; } \
} while(0);
 
/**
 * Writes build information into the given string and returns latter as
 * const pointer.
 * @param char* s
 * @param unsigned length
 * @return const char*
 */
const char* trajgen_info(char* s, unsigned length) {
  if(!s || !length) return (const char*) s;
  char *errstr_p = s, *errstr_e = s+length;
  const char* tp;
  do { unsigned i; for(i=0; i<length; i++) s[i] = '\0'; } while(0);
  sprinterr("trajgen.c/h, version "); sprinterr(TG_VERSION);
  #ifdef ARG_POINTER_CHECKS
  sprinterr(", with-0pointer-checks");
  #else
  sprinterr(", without-0pointer-checks");
  #endif
  #ifdef TG_DEBUG_LEVEL
  #if TG_DEBUG_LEVEL > 1
  sprinterr(", debug2");
  #else
  sprinterr(", debug2");
  #endif
  #else
  sprinterr(", no-debug");
  #endif
  #ifdef NO_INTERPOLATION
  sprinterr(", without-interpolation");
  #else
  sprinterr(", with-interpolation");
  #endif
  #if NO_MOTION_BLENDING > 0
  sprinterr(", without-blending");
  #else
  sprinterr(", with-blending");
  #endif
  #ifdef WITH_CURVES
  sprinterr(", with-curves");
  #else
  sprinterr(", without-curves");
  #endif
  #ifdef NO_TRAJECTORY_SYNC
  sprinterr(", without-syncing");
  #else
  sprinterr(", with-syncing");
  #endif
  #ifdef NO_ABC
  sprinterr(", without-abc");
  #else
  sprinterr(", with-abc");
  #endif
  #ifdef NO_UVW
  sprinterr(", without-uvw");
  #else
  sprinterr(", with-uvw");
  #endif
  #ifdef EXPORT_INTERPOLATORS
  sprinterr(", exported-interpolators");
  #endif
  #ifdef EXPORT_JOINT_TG
  sprinterr(", exported-jointtg");
  #endif
  #ifdef EXPORT_MANUAL_TG
  sprinterr(", exported-manualtg");
  #endif
  #ifdef EXPORT_COORD_TG
  sprinterr(", exported-coordtg");
  #endif
  #define TG_STR(s) TG_STRINNER(s)
  #define TG_STRINNER(s) #s
  sprinterr((", float-type="  TG_STR(real_t)));
  sprinterr(", max-joints=" TG_STR(TG_MAX_JOINTS));
  sprinterr(", queue-size=" TG_STR(TG_QUEUE_SIZE));
  sprinterr(", machine-resolution=" TG_STR(TG_RESOLUTION));
  sprinterr(", dimensional-resolution=" TG_STR(TG_D_RES));
  sprinterr(", angle-resolution=" TG_STR(TG_A_RES));
  sprinterr(", blending-max-angle=" TG_STR(MAX_ALLOWED_BLENDING_ANGLE_DEG) "deg");
  sprinterr(", straight-blending-max-angle=" TG_STR(STRAIGHT_BLENDING_ANGLE) "deg");
  sprinterr(", stfwi. ");
  #undef TG_STR
  #undef TG_STRINNER
  return (const char*) s;
}
 
/**
 * Writes a text version of a trajgen error into *errstr
 * @param errnum
 * @param errstr
 * @param length
 */
const char* trajgen_errstr(uint16_t errnum, char* errstr, unsigned length)
{
  if(!errstr || !length) return (const char*) errstr;
  char *errstr_p = errstr, *errstr_e = errstr+length;
  const char* tp;
  trajgen_error_details_t err;
  err.errno = errnum;
  do { unsigned i; for(i=0; i<length; i++) errstr[i] = '\0'; } while(0);
  switch (err.sel.code) {
    case TRAJ_ERROR_OK:
      sprinterr("(ok)");
      break;
    case TRAJ_ERROR_ERROR:
      sprinterr("[main trajgen] General error");
      break;
    case TRAJ_ERROR_NULL_POINTER:
      sprinterr("[main trajgen] Null pointer");
      break;
    case TRAJ_ERROR_NUMERIC:
      sprinterr("[main trajgen] Null pointer");
      break;
    case TRAJ_ERROR_CONFIG_LAST_USED_JOINT_INVALID:
      sprinterr("[main trajgen] Invalid last used joint value");
      break;
    case TRAJ_ERROR_CONFIG_INTERPOLATION_RATE_INVALID:
      sprinterr("[main trajgen] Invalid interpolation rate setting");
      break;
    case TRAJ_ERROR_CONFIG_COMPILE_SETTING_INVALID:
      sprinterr("[main trajgen] Invalid compilation setting (preprocessor defined)");
      break;
    case TRAJ_ERROR_CONFIG_OVERRIDE_INVALID:
      sprinterr("[main trajgen] Invalid override");
      break;
    case TRAJ_ERROR_INVALID_STATE:
      sprinterr("[main trajgen] Invalid state");
      break;
    case TRAJ_ERROR_INVALID_SWITCHING_STATE:
      sprinterr("[main trajgen] Invalid switching state");
      break;
    case TRAJ_ERROR_INVALID_JOINT_NO:
      sprinterr("[main trajgen] Invalid joint index");
      break;
    case KINEMATICS_ERR_ERROR:
      sprinterr("[kinematics] General error");
      break;
    case KINEMATICS_ERR_NULL_POINTER:
      sprinterr("[kinematics] Null pointer");
      break;
    case KINEMATICS_ERR_INIT_FUNCTION_NULL:
      sprinterr("[kinematics] Invalid kinematics definition (contains function null-pointer)");
      break;
    case KINEMATICS_ERR_FORWARD_FAILED:
      sprinterr("[kinematics] Forward kinematics failed");
      break;
    case KINEMATICS_ERR_INVERSE_FAILED:
      sprinterr("[kinematics] Inverse kinematics failed");
      break;
    case KINEMATICS_ERR_RESET_FAILED:
      sprinterr("[kinematics] Reset setting failed");
      break;
    case INTERPOLATOR_ERROR:
      sprinterr("[interpolator] General error"); add_joint();
      break;
    case INTERPOLATOR_ERROR_INIT_ARG_INVALID:
      sprinterr("[interpolator] Invalid initialisation argument"); add_joint();
      break;
    case INTERPOLATOR_ERROR_QUEUE_FULL:
      sprinterr("[interpolator] Queue full"); add_joint();
      break;
    case INTERPOLATOR_ERROR_OFFSET_IP_NULLPOINTER:
      sprinterr("[interpolator] Offset CI null pointer"); add_joint();
      break;
    case INTERPOLATOR_ERROR_INTERPOLATE_ARG_NULLPOINTER:
      sprinterr("[interpolator] Interpolate: Null pointer"); add_joint();
      break;
    case INTERPOLATOR_ERROR_NOT_RESET:
      sprinterr("[interpolator] Not initialised"); add_joint();
      break;
    case TRAJGEN_FREE_ERROR_ERROR:
      sprinterr("[joint tg] General error"); add_joint();
      break;
    case TRAJGEN_FREE_ERROR_INIT_NULLPOINTER:
      sprinterr("[joint tg] Initialisation null pointer"); add_joint();
      break;
    case TRAJGEN_FREE_ERROR_INIT_INVALID_MAX_ACCEL:
      sprinterr("[joint tg] Invalid maximum acceleration"); add_joint();
      break;
    case TRAJGEN_FREE_ERROR_INIT_INVALID_MAX_VELOCITY:
      sprinterr("[joint tg] Invalid maximum velocity"); add_joint();
      break;
    case TRAJGEN_FREE_ERROR_INIT_INVALID_SAMPLE_INTERVAL:
      sprinterr("[joint tg] Invalid sample interval"); add_joint();
      break;
    case TRAJGEN_FREE_ERROR_INVALID_ACCELERATION:
      sprinterr("[joint tg] Invalid acceleration"); add_joint();
      break;
    case TP_ERR_ERROR:
      sprinterr("[coord tg] General error");
      break;
    case TP_ERR_TP_NULL_POINTER:
      sprinterr("[coord tg] Null pointer");
      break;
    case TP_ERR_ABORTING:
      sprinterr("[coord tg] Aborting");
      break;
    case TP_ERR_QUEUE_PUT_FAILED:
      sprinterr("[coord tg] Queue push failed");
      break;
    case TP_ERR_INVALID_PARAM:
      sprinterr("[coord tg] Invalid argument");
      break;
    case TP_ERR_QUEUE_FULL:
      sprinterr("[coord tg] Queue full");
      break;
    case TP_ERR_QUEUE_TO_MANY_ELEMENTS_TO_REMOVE:
      sprinterr("[coord tg] Too many items to remove");
      break;
    case TP_ERR_INVALID_MOTION_TYPE:
      sprinterr("[coord tg] Invalid motion type");
      break;
    case TP_ERR_INVALID_SPEED:
      sprinterr("[coord tg] Invalid velocity");
      break;
    case TP_ERR_INVALID_ACCEL:
      sprinterr("[coord tg] Invalid acceleration");
      break;
    case TP_ERR_INVALID_POSE:
      sprinterr("[coord tg] Invalid argument pose");
      break;
    case TP_ERR_INVALID_SAMPLE_INTERVAL:
      sprinterr("[coord tg] Invalid sample interval");
      break;
    case TP_ERR_ALREADY_MOVING:
      sprinterr("[coord tg] Already moving");
      break;
    case TP_ERR_SEGMENT_LENGTH_ZERO:
      sprinterr("[coord tg] Segment to move has a length of zero");
      break;
    case TP_ERR_UNIT_VECTOR_CALC_INVALID_TYPE:
      sprinterr("[coord tg] Unit vector calculation failed");
      break;
    case TP_ERR_REF_POSITION_INVALIDATED_DURING_MOTION:
      sprinterr("[coord tg] Ref-position invalidated during motion");
      break;
    case TG_MAN_ERROR_ERROR:
      sprinterr("[manual tg] General error");
      break;
    case TG_MAN_ERROR_INIT_NULLPOINTER:
      sprinterr("[manual tg] Initialisation: null pointer");
      break;
    case TG_MAN_ERROR_INIT_INVALID_MAX_ACCEL:
      sprinterr("[manual tg] Invalid maximum acceleration");
      break;
    case TG_MAN_ERROR_INIT_INVALID_MAX_VELOCITY:
      sprinterr("[manual tg] Invalid maximum velocity");
      break;
    case TG_MAN_ERROR_INIT_INVALID_SAMPLE_INTERVAL:
      sprinterr("[manual tg] Invalid sample interval");
      break;
    case TG_MAN_ERROR_INVALID_ACCELERATION:
      sprinterr("[manual tg] Invalid acceleration");
      break;
    default:
      sprinterr("Unknown error code");
  }
  return (const char*) errstr;
}
 
#undef sprinterr
#undef add_joint
 
// </editor-fold>

