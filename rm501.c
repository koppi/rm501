/*
 * rm501.c - Mitsubishi RM-501 Movemaster II Robot Simulator
 *
 * Copyright (C) 2013-2021 Jakob Flierl <jakob.flierl@gmail.com>
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation:
 *  version 2.1 of the License.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *  MA  02110-1301  USA
 */

#define PROGRAM_VERSION "0.0.1"

// sudo apt-get -y install libsdl2-dev libsdl2-ttf-dev
// for joystick access: sudo usermod -aG input $USER

//#define HAVE_SDL           // working
#ifdef HAVE_SDL
  #define HAVE_JOYSTICK    // working, requires HAVE_SDL
  #define HAVE_PNG         // working, requires HAVE_SDL
  #define ENABLE_FPS_LIMIT // working, requires HAVE_SDL
  #ifdef ENABLE_FPS_LIMIT
    #define DEFAULT_FPS 50
  #endif
#endif

#define HAVE_SPACENAV        // working
//#define HAVE_HAL             // partially working
//#define HAVE_NCURSES         // unfinished
//#define HAVE_SERIAL          // unfinished
//#define HAVE_SOCKET          // unfinished
//#define HAVE_MOSQUITTO       // unfinished
//#define HAVE_ZMQ             // unfinished
//#define HAVE_TRAJGEN         // unfinished

// see http://www2.ece.ohio-state.edu/~zheng/ece5463/proj2/5463-Project-2-FA2015.pdf
#define PROJ2

// see http://www2.ece.ohio-state.edu/~zheng/ece5463/proj3/5463-Project-3-FA2015.pdf
#define PROJ3

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h> // EXIT_SUCCESS
#include <string.h> // memcpy()
#include <signal.h> // sigaction(), sigsuspend(), sig*()
#include <math.h>

#ifdef HAVE_ZMQ
#include <zmq.h>
#endif

#ifdef HAVE_MOSQUITTO
#include <mosquitto.h>
#endif

#ifdef HAVE_SDL
#include <GL/glu.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_opengl.h>
#endif

#ifdef HAVE_NCURSES
#include <curses.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#ifdef HAVE_SPACENAV
#include <linux/input.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#ifdef HAVE_SERIAL
#include <libserialport.h>
#endif

#ifdef HAVE_SDL
#ifdef HAVE_PNG
#include "savepng.h"
#include "savepng.c"
#endif
#endif

#ifdef HAVE_TRAJGEN
#include "trajgen.h"
#endif

volatile int done    = 0; // SIGINT or user exit requested

#ifdef HAVE_HAL
int do_hal = 0;
#include "hal.h"
#define MODULE_NAME "rm501"
char *modname = MODULE_NAME;

typedef struct {
    hal_float_t *axis[5];
} hal_t;

int    hal_comp_id;
hal_t *hal_pos_data;
#endif

#ifdef HAVE_SDL
int view_mode = 0;
const int width = 1280, height = 700; // 720
TTF_Font *sdl_font;

SDL_Window   *sdl_window;
SDL_Renderer *sdl_renderer;
#endif

#ifdef HAVE_NCURSES
WINDOW *menubar,*messagebar;
#endif

#ifdef HAVE_SDL
#ifdef HAVE_PNG
SDL_Surface *png_shot;
#endif
#endif

#ifdef HAVE_JOYSTICK
SDL_Joystick* joy = NULL;
const int JOYSTICK_DEAD_ZONE = 2500;
#endif

#ifdef HAVE_SOCKET
int do_net = 0;
#endif

#ifdef HAVE_ZMQ
int do_zmq = 0;
#endif

#ifdef HAVE_MOSQUITTO
int do_mosquitto = 0;
struct mosquitto *mosq;
#endif

#ifdef HAVE_TRAJGEN
int do_trajgen_test = 0;
real_t tg_speed;
real_t tg_accel;
real_t tg_blending;
unsigned tg_n_joints;
trajgen_t tg_tg;
trajgen_config_t tg_cfg;
pose_t tg_last_pose;
char tg_err_str[1024];

double rnd() {
  return ((double)rand())/RAND_MAX;
}

double roundd(double v, unsigned digits) {
  for(unsigned i=0; i<digits; i++) v*=10.;
  v = round(v);
  for(unsigned i=0; i<digits; i++) v/=10.;
  return v;
}

double randpos(double max) {
  return roundd(max*((rnd()*2)-1), 2);
}

void tg_echk (unsigned errno) {
  if (errno) {
    fprintf(stderr, "tg error: %s\n", trajgen_errstr(tg_tg.last_error.errno, tg_err_str, 1024));
    exit(1); //XXX
  }
}

void tg_init (real_t max_speed, real_t max_accel, real_t sample_frequency,
	unsigned interpolation_rate, unsigned max_joints)
  {
    memset(tg_err_str, 0, sizeof(tg_err_str));
    memset(&tg_last_pose, 0, sizeof(pose_t));
    memset(&tg_tg, 0, sizeof(trajgen_t));
    memset(&tg_cfg, 0, sizeof(trajgen_config_t));
    
    tg_n_joints = max_joints > TG_MAX_JOINTS ? TG_MAX_JOINTS : max_joints;
    memset(&tg_cfg, 0, sizeof(trajgen_config_t));
    tg_cfg.number_of_used_joints = tg_n_joints;
    tg_cfg.max_coord_velocity = max_speed;
    tg_cfg.max_coord_acceleration = max_accel;
    tg_cfg.sample_interval = 1.0/sample_frequency;
    tg_cfg.interpolation_rate = interpolation_rate;
    pose_set_all(&tg_cfg.max_manual_velocities, max_speed);
    pose_set_all(&tg_cfg.max_manual_accelerations, max_accel);
    for (unsigned i = 0; i < tg_n_joints; i++) {
      tg_cfg.joints[i].max_velocity = max_speed;
      tg_cfg.joints[i].max_acceleration = max_speed;
    }
    tg_echk(trajgen_initialize(&tg_tg, tg_cfg));
  }

void tg_line(double x, double y, double z) {
    pose_t p;
    p.x = isnan(x) ? tg_last_pose.x : x; p.y = isnan(y) ? tg_last_pose.y : y;
    p.z = isnan(z) ? tg_last_pose.z : z;
    tg_last_pose = p;

    fprintf(stderr, "# line {x:%.6g, y:%.6g, z:%.6g, v:%.6g, a:%.6g, tol:%.6g }\n", p.x, p.y, p.z, tg_speed, tg_accel, tg_blending);
    tg_echk(trajgen_add_line(p, tg_speed, tg_accel));
  }

#endif

typedef struct {
    double d1, d5, a2, a3; // dh parameters

    struct joint_s {
        double pos;
        double vel;
        double min;
        double max;
#ifdef PROJ2
        double tar;
#endif
    } j[5]; // joints

    double t[16];  // tool matrix
    int  err;      // error a to joint number if inverse kinematics fails
    char msg[2048]; // opt. message from inverse kinematics

#ifdef PROJ3
    int proj3counter;
#endif

} bot_t;

#define rad2deg(rad) ((rad)*(180.0/M_PI))
#define deg2rad(deg) ((deg)*(M_PI/180.0))
#define sq(x) ((x)*(x))

void rotate_m_axyz(double *mat, double angle, double x, double y, double z ) {
    int i;

    double m[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    double s = sin( deg2rad(angle) );
    double c = cos( deg2rad(angle) );

    double mag = sqrt(sq(x) + sq(y) + sq(z));
    if (mag <= 1.0e-4) { return; } // no rotation, leave mat as-is
    x /= mag; y /= mag; z /= mag;

    double one_c = 1.0 - c;
#define M(row,col)  m[col*4+row]
    M(0,0) = (one_c * sq(x)) + c;
    M(0,1) = (one_c * x*y) - z*s;
    M(0,2) = (one_c * z*x) + y*s;

    M(1,0) = (one_c * x*y) + z*s;
    M(1,1) = (one_c * sq(y)) + c;
    M(1,2) = (one_c * y*z) - x*s;

    M(2,0) = (one_c * z*x) - y*s;
    M(2,1) = (one_c * y*z) + x*s;
    M(2,2) = (one_c * sq(z)) + c;
#undef M

    for (i = 0; i < 4; i++) {
#define A(row,col)  mat[(col<<2)+row]
#define B(row,col)  m[(col<<2)+row]
        double ai0=A(i,0), ai1=A(i,1), ai2=A(i,2), ai3=A(i,3);
        A(i,0) = ai0 * B(0,0) + ai1 * B(1,0) + ai2 * B(2,0) + ai3 * B(3,0);
        A(i,1) = ai0 * B(0,1) + ai1 * B(1,1) + ai2 * B(2,1) + ai3 * B(3,1);
        A(i,2) = ai0 * B(0,2) + ai1 * B(1,2) + ai2 * B(2,2) + ai3 * B(3,2);
        A(i,3) = ai0 * B(0,3) + ai1 * B(1,3) + ai2 * B(2,3) + ai3 * B(3,3);
#undef A
#undef B
    }

}

#define RPY_P_FUZZ (0.000001)

int pmMatRpyConvert(double m[], double *r, double *p, double *y) {
    *p = atan2(-m[2], sqrt(sq(m[0]) + sq(m[1])));

    if (fabs(*p - (2 * M_PI)) < RPY_P_FUZZ) {
        *r = atan2(m[4], m[5]);
        *p = (2 * M_PI);
        *y = 0.0;
    } else if (fabs(*p + (2 * M_PI)) < RPY_P_FUZZ) {
        *r = -atan2(m[6], m[5]);
        *p = -(2 * M_PI);
        *y = 0.0;
    } else {
        *r = atan2(m[6], m[10]);
        *y = atan2(m[1], m[0]);
    }

    return 0;
}

void kins_fwd(bot_t *bot) {
    double tr1 = deg2rad(bot->j[0].pos);
    double tr2 = deg2rad(bot->j[1].pos);
    double tr3 = deg2rad(bot->j[2].pos);
    double tr4 = deg2rad(bot->j[3].pos);
    double tr5 = deg2rad(bot->j[4].pos);

    double C234 = cos(tr2+tr3+tr4);
    double S234 = sin(tr2+tr3+tr4);
    double C23  = cos(tr2+tr3);
    double S23  = sin(tr2+tr3);
    double S1   = sin(tr1);
    double C1   = cos(tr1);
    double S2   = sin(tr2);
    double C2   = cos(tr2);
    double S5   = sin(tr5);
    double C5   = cos(tr5);

    double px = C1*(bot->d5*S234 + bot->a3*C23 + bot->a2*C2);
    double py = S1*(bot->d5*S234 + bot->a3*C23 + bot->a2*C2);
    double pz = bot->d1 - bot->d5*C234 + bot->a3*S23 + bot->a2*S2;

    bot->t[0] =   C1*C5*C234 - S1*S5;
    bot->t[1] =   C5*S234;
    bot->t[2] = - S1*C5*C234 - C1*S5;
    bot->t[3] = 0;

    bot->t[4] = -C1*S234;
    bot->t[5] =  C234;
    bot->t[6] =  S1*S234;
    bot->t[7] = 0;

    bot->t[8]  = S1*C5 + C1*C234*S5;
    bot->t[9]  = S5*S234;
    bot->t[10] = C1*C5 - S1*C234*S5;
    bot->t[11] = 0;

    bot->t[12] = px;
    bot->t[13] = pz;
    bot->t[14] = py;

    bot->t[15] = 1;
}

void kins_inv(bot_t* bot) {

    double nx = -bot->t[0], ny =  bot->t[2];
    double ox =  bot->t[8], oy = -bot->t[10];
    double ax = -bot->t[4], ay =  bot->t[6],  az = -bot->t[5];

    double px = bot->t[12];
    double pz = bot->t[13];
    double py = bot->t[14];

    double th1;

    if (py == 0 && px == 0) {     // point on the Z0 axis
        if (ay == 0 && ax == 0) { // wrist pointing straight up/down
            th1 = 0;
        } else {
            th1 = atan2(ay, ax);
        }
    } else {
        th1 = atan2(py, px);
    }

    double c1 = cos(th1);
    double s1 = sin(th1);

    double t234 = atan2(c1*ax + s1*ay, -az);
    double c234 = cos(t234);
    double s234 = sin(t234);

    // joint 3 - elbow
    double tp1 = c1*px + s1*py - bot->d5*s234;
    double tp2 = pz - bot->d1 + bot->d5*c234;
    double c3  = (sq(tp1) + sq(tp2) - sq(bot->a3) - sq(bot->a2)) / (2*bot->a2*bot->a3);
    double s3  = -sqrt(1 - sq(c3));
    double th3 = atan2(s3, c3);

    // joint 2 - shoulder
    double num = tp2*(bot->a3*c3 + bot->a2) - bot->a3*s3*tp1;
    double den = tp1*(bot->a3*c3 + bot->a2) + bot->a3*s3*tp2;
    double th2 = atan2(num, den);

    // joint 4 - pitch
    double th4 = (t234 - th3 - th2);

    // joint 5 - roll
    double th5 = atan2(s1*nx - c1*ny, s1*ox - c1*oy);

    char msg[5][256];
    double th[] = {th1, th2, th3, th4, th5};
    int i;

    bot->err = 0;

    for (i = 0; i < 5; i++) {
#ifdef KINS_INV_IGNORE_LIMITS
        if (!isnan(th[i])) {
#else
            if (!isnan(th[i]) && th[i] >= deg2rad(bot->j[i].min) && th[i] <= deg2rad(bot->j[i].max)) {
#endif
                bot->j[i].pos = rad2deg(th[i]);
            } else {
                bot->err |= (1 << i);
            }

            // pretty print results

            if (isnan(th[i])) {
                snprintf(msg[i], sizeof(msg[i]), "%7s ", "nan");
            } else if (th[i] < deg2rad(bot->j[i].min)) {
                snprintf(msg[i], sizeof(msg[i]), "%7.2fv", rad2deg(th[i]));
            } else if (th[i] > deg2rad(bot->j[i].max)) {
                snprintf(msg[i], sizeof(msg[i]), "%7.2f^", rad2deg(th[i]));
            } else {
                snprintf(msg[i], sizeof(msg[i]), "%7.2f ", rad2deg(th[i]));
            }
        }

        snprintf(bot->msg, sizeof(bot->msg), "kin_inv(%d): %s %s %s %s %s", bot->err, msg[0], msg[1], msg[2], msg[3], msg[4]);
    }

#ifdef HAVE_SDL
void cross(float th, float l) {
    glLineWidth(th);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(l, 0, 0);
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, l, 0);
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, l);
    glEnd();
    glLineWidth(1);
}
#endif

#ifdef HAVE_SDL
    void text(int x, int y, TTF_Font *font, const char * format, ...) {
        char buffer[256];
        va_list args;

        if (!font) { return; }

        va_start (args, format);
        vsnprintf (buffer, 255, format, args);

        SDL_Color col = {255, 255, 255};
        SDL_Surface *msg = TTF_RenderText_Blended(font, buffer, col);
        unsigned tex = 0;

        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, msg->w, msg->h, 0, GL_BGRA,
                     GL_UNSIGNED_BYTE, msg->pixels);

        int z = 1;

        glBegin(GL_QUADS);
        glTexCoord2d(0, 0); glVertex3d(x, y, z);
        glTexCoord2d(1, 0); glVertex3d(x+msg->w, y, z);
        glTexCoord2d(1, 1); glVertex3d(x+msg->w, y+msg->h, z);
        glTexCoord2d(0, 1); glVertex3d(x, y+msg->h, z);
        glEnd();

        glDeleteTextures(1, &tex);
        SDL_FreeSurface(msg);

        va_end (args);
    }

    void text_matrix(int x, int y, double m[]) {
        int i, j;
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                text(x + 75 * i, y + TTF_FontHeight(sdl_font) * j, sdl_font, "%8.2f", m[4*i+j]);
            }
        }
    }

    void cube( GLfloat dSize, int wire ) {
        float size = dSize * 0.5;

#   define V(a,b,c) glVertex3d( a size, b size, c size );
#   define N(a,b,c) glNormal3d( a, b, c );

        if (wire) {
            glBegin( GL_LINE_LOOP ); N( 1.0, 0.0, 0.0); V(+,-,+); V(+,-,-); V(+,+,-); V(+,+,+); glEnd();
            glBegin( GL_LINE_LOOP ); N( 0.0, 1.0, 0.0); V(+,+,+); V(+,+,-); V(-,+,-); V(-,+,+); glEnd();
            glBegin( GL_LINE_LOOP ); N( 0.0, 0.0, 1.0); V(+,+,+); V(-,+,+); V(-,-,+); V(+,-,+); glEnd();
            glBegin( GL_LINE_LOOP ); N(-1.0, 0.0, 0.0); V(-,-,+); V(-,+,+); V(-,+,-); V(-,-,-); glEnd();
            glBegin( GL_LINE_LOOP ); N( 0.0,-1.0, 0.0); V(-,-,+); V(-,-,-); V(+,-,-); V(+,-,+); glEnd();
            glBegin( GL_LINE_LOOP ); N( 0.0, 0.0,-1.0); V(-,-,-); V(-,+,-); V(+,+,-); V(+,-,-); glEnd();
        } else {
            glBegin( GL_QUADS );
            N( 1.0, 0.0, 0.0); V(+,-,+); V(+,-,-); V(+,+,-); V(+,+,+);
            N( 0.0, 1.0, 0.0); V(+,+,+); V(+,+,-); V(-,+,-); V(-,+,+);
            N( 0.0, 0.0, 1.0); V(+,+,+); V(-,+,+); V(-,-,+); V(+,-,+);
            N(-1.0, 0.0, 0.0); V(-,-,+); V(-,+,+); V(-,+,-); V(-,-,-);
            N( 0.0,-1.0, 0.0); V(-,-,+); V(-,-,-); V(+,-,-); V(+,-,+);
            N( 0.0, 0.0,-1.0); V(-,-,-); V(-,+,-); V(+,+,-); V(+,-,-);
            glEnd();
        }
#   undef V
#   undef N
    }

    GLUquadricObj *create_quadric(int wire) {
        GLUquadricObj *qobj = gluNewQuadric();
        if (!wire) {
            gluQuadricDrawStyle(qobj, GLU_FILL);
        } else {
            gluQuadricDrawStyle(qobj, GLU_LINE);
        }
        return qobj;
    }

    void cylinder(int wire, double BASE, double TOP, double HEIGHT, double SLICES, double STACKS) {
        GLUquadricObj *QUAD = create_quadric(wire);

        gluCylinder (QUAD, BASE, TOP, HEIGHT, SLICES, STACKS);

        if (!wire) {
            glRotatef(180, 1, 0, 0);
            gluDisk(QUAD, 0.0, BASE, SLICES, 1);
            glRotatef(180, 1, 0, 0);
            glTranslatef(0.0, 0.0, HEIGHT);
            gluDisk(QUAD, 0.0, TOP, SLICES, 1);
        }

        gluDeleteQuadric(QUAD);
    }

    void draw_bot(int wire, bot_t *bot) {
        glPushMatrix();

        glTranslatef (0.0, bot->d1, 0.0);

        cross(4, 1);

        if (!wire) {
            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        } else {
            glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        }

        // link1 - base

        glColor3f(0.25, 0.25, 0.25);

        glPushMatrix();
        glTranslatef (-0.5, -1.875, 0);
        glScalef (2, 0.75, 2);
        cube (1.0, wire);
        glPopMatrix();

        glPushMatrix();
        glTranslatef (0, -1.75, 0);
        glScalef (1-0.05, 1.0-0.05, 1.25-0.05);
        cube (1.0, wire);
        glPopMatrix();

        glPushMatrix();
        glTranslatef (0.0, -1.175 - 0.65 / 2, 0);
        glRotatef (-90, 1, 0, 0);
        glScalef (1.0, 1.0, 1.0);
        glColor3f(0.5, 0.5, 0.5);

        cylinder(wire, 0.35, 0.35, 0.65, 16, 1);

        glPopMatrix();

        // link2 - body

        glColor3f(1.0, 0.44, 0.176);

        glPushMatrix();
        glTranslatef (0, 0.0, 0.0);
        glRotatef (bot->j[0].pos, 0.0, 1.0, 0.0);
        glTranslatef (0, 0.0, 0.0);

        glPushMatrix();
        glTranslatef (-0.75, -0.45, 0.0);
        glScalef (2.5, 0.9, 1.2);
        cube (1.001, wire);
        glPopMatrix();

        glPushMatrix();
        glRotatef (14, 0.0, 0.0, 1.0);
        glTranslatef (-0.975, 0.05, 0.0);
        glScalef (1.9, 0.9, 1.2);
        cube (0.999, wire);
        glPopMatrix();

        glPushMatrix();
        glTranslatef (0.0, 0, -1.2/2);
        glRotatef (-90, 0.0, 0.0, 1.0);
        glScalef (1.0, 1.0, 1.0);
        cylinder(wire, 0.5, 0.5, 1.2, 16, 1);
        glPopMatrix();

        if (!wire) {
            glPushMatrix();
            glTranslatef (0.0, 0, -1.4/2);
            glRotatef (-90, 0.0, 0.0, 1.0);
            glScalef (1.0, 1.0, 1.0);
            cylinder(wire, 0.15, 0.15, 1.4, 16, 1);
            glPopMatrix();
        }

        glColor3f(1.0, 0.44, 0.176);

        glPushMatrix();
        glTranslatef (0.0, 0, -bot->d5/2);
        glRotatef (-90, 0.0, 0.0, 1.0);
        glScalef (1.0, 1.0, 1.0);
        cylinder(wire, 0.35, 0.35, bot->d5, 16, 1);
        glPopMatrix();

        // link3 - upperarm

        glColor3f(1.0, 0.44, 0.176);

        glRotatef (bot->j[1].pos, 0.0, 0.0, 1.0);

        glPushMatrix();
        glTranslatef (1.1, 0.0, 0.0);
        glScalef (bot->a2, 1.0, 1.0);
        cube (1.0, wire);
        glPopMatrix();

        if (!wire) {
            glPushMatrix();
            glTranslatef (0, 0, -0.5);
            glRotatef (-90, 0.0, 0.0, 1.0);
            glScalef (1.0, 1.0, 1.0);
            cylinder(wire, 0.5, 0.5, 1, 16, 1);
            glPopMatrix();

            glPushMatrix();
            glTranslatef (bot->a2, 0, -0.5);
            glRotatef (-90, 0.0, 0.0, 1.0);
            glScalef (1.0, 1.0, 1.0);
            cylinder(wire, 0.5, 0.5, 1, 16, 1);
            glPopMatrix();

            glPushMatrix();
            glTranslatef (bot->a2, 0, -1.1/2);
            glRotatef (-90, 0.0, 0.0, 1.0);
            glScalef (1.0, 1.0, 1.0);
            cylinder(wire, 0.25, 0.25, 1.1, 16, 1);
            glPopMatrix();

            glPushMatrix();
            glTranslatef (bot->a2, 0, -1.2/2);
            glRotatef (-90, 0.0, 0.0, 1.0);
            glScalef (1.0, 1.0, 1.0);
            cylinder(wire, 0.15, 0.15, 1.2, 16, 1);
            glPopMatrix();
        }

        // link4 - forearm

        glTranslatef(bot->a2,0,0);
        glRotatef (bot->j[2].pos, 0.0, 0.0, 1.0);

        glPushMatrix();
        glTranslatef (0.6, 0.0, 0.0);
        glScalef (1.2, 0.8, 0.9);
        cube (1.0, wire);
        glPopMatrix();

        glPushMatrix();
        glTranslatef (0, 0, -0.45);
        glRotatef (-90, 0.0, 0.0, 1.0);
        glScalef (1.0, 1.0, 1.0);
        cylinder(wire, 0.4, 0.4, 0.9, 16, 1);
        glPopMatrix();

        glPushMatrix();
        glTranslatef (1.2, 0, -0.45);
        glRotatef (-90, 0.0, 0.0, 1.0);
        glScalef (1.0, 1.0, 1.0);
        cylinder(wire, 0.4, 0.4, 0.9, 16, 1);
        glPopMatrix();

        // link5 - wrist pitch

        glColor3f(0.25, 0.25, 0.25);

        glTranslatef(bot->a3,0,0);
        glRotatef (bot->j[3].pos, 0.0, 0.0, 1.0);

        glPushMatrix();
        glTranslatef (0, 0, -0.6);
        glRotatef (-90, 0.0, 0.0, 1.0);
        cylinder(wire, 0.3, 0.3, 1.2, 22, 1);
        glPopMatrix();

        glColor3f(0.5, 0.5, 0.5);

        // link6 - hand

        glColor3f(0.25, 0.25, 0.25);

        glRotatef (bot->j[4].pos, 0.0, 1.0, 0.0);

        glPushMatrix();
        glRotatef (90, 1.0, 0.0, 0.0);
        cylinder(wire, 0.13, 0.13, 0.5, 22, 1);
        glPopMatrix();

        glColor3f(0.15, 0.15, 0.15);

        glPushMatrix();
        glTranslatef (0.0, -0.7, 0.0);
        glScalef (0.5, 0.4, 0.4);
        cube (1.0, wire);
        glPopMatrix();

        glPushMatrix();
        glTranslatef (0.0, -1.02, 0.0);
        glScalef (0.8, 0.25, 0.4);
        cube (1.0, wire);
        glPopMatrix();

        glDisable (GL_LIGHTING);
        glDisable (GL_COLOR_MATERIAL);

        glTranslatef (0, -bot->d5, 0);

        cross(4, 0.5);

        glPopMatrix();

        glPopMatrix();
    }

    void scene(bot_t *bot_fwd, bot_t *bot_inv) {
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);

        // table

        glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

        glColor3f(0.001, 0.001, 0.001);

        int x;
        for (x = -50; x <= 50; x+=5) {
            glPushMatrix();
            glTranslatef (-0.5 + x * .1, 0, 0);
            glScalef (.05, .01, 10.0);
            cube (1.0, 0);
            glPopMatrix();
            glPushMatrix();
            glTranslatef (-0.5, 0, x * .1);
            glScalef (10.0, .01, .05);
            cube (1.0, 0);
            glPopMatrix();
        }

        draw_bot(0, bot_fwd);
        draw_bot(1, bot_inv);
    }

    void draw_hud(bot_t* bot) {
        glMatrixMode(GL_MODELVIEW);

        glViewport(0, 0, width, height);

        glMatrixMode(GL_PROJECTION);

        glPushMatrix();
        glLoadIdentity();
        glOrtho(0.0, width, height, 0.0, -1.0, 10.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glClear(GL_DEPTH_BUFFER_BIT);
        glColor3f(1,0.2f,0.2f);

        glEnable( GL_BLEND );
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

        glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

        glColor3ub( 25, 200, 25 );

        text(15, 10, sdl_font, "AXIS:  POS:     VEL:");

        int i;
        for (i = 0; i < 5; i++) {
            text(15, 10+(i+1)*TTF_FontHeight(sdl_font), sdl_font,
                 "%d: %8.2f %8.3f", i+1, bot->j[i].pos, bot->j[i].vel);
        }

        text(15, 10+7*TTF_FontHeight(sdl_font), sdl_font, "x: %8.2f", bot->t[12]);
        text(15, 10+8*TTF_FontHeight(sdl_font), sdl_font, "y: %8.2f", bot->t[13]);
        text(15, 10+9*TTF_FontHeight(sdl_font), sdl_font, "z: %8.2f", bot->t[14]);

        double r, p, y;

        pmMatRpyConvert(bot->t, &r, &p, &y);

        text(15, 10+11*TTF_FontHeight(sdl_font), sdl_font, "a: %8.2f", rad2deg(r));
        text(15, 10+12*TTF_FontHeight(sdl_font), sdl_font, "b: %8.2f", rad2deg(p));
        text(15, 10+13*TTF_FontHeight(sdl_font), sdl_font, "c: %8.2f", rad2deg(y));

        text(width - 370,10, sdl_font, "TOOL"); text_matrix(width - 320, 10, bot->t);

        if (strlen(bot->msg)) {
            text(15, height - TTF_FontHeight(sdl_font) * 1.5, sdl_font, bot->msg);
        }

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glMatrixMode(GL_MODELVIEW);
    }

    void display(bot_t *bot_fwd, bot_t *bot_inv) {
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glClearDepth(1.0);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        if (view_mode == 0) {
            glViewport(0, 0, width, height);
        } else {
            glViewport(0, 0, width / 2, height / 2);
        }

        glPushMatrix();
        glTranslatef(0, 0, -10);
        glRotatef(15, 1, 0, 0); // glRotatef(rotate_x, 0, 1, 0);
        glTranslatef(0, -2, 0);

        scene(bot_fwd, bot_inv);

        glPopMatrix();

        glMatrixMode(GL_PROJECTION);

        if (view_mode != 0) {
            glPushMatrix();

            glLoadIdentity();
#define DIM 4.5
            if (height > width) {
                glOrtho(-DIM, DIM, -DIM * (height / (width * 1.0)), DIM * (height / (width * 1.0)), -DIM*2, DIM*2);
            } else {
                glOrtho(-DIM * (width / (height * 1.0)), DIM * (width / (height * 1.0)), -DIM, DIM, -DIM*2, DIM*2);
            }
#undef DIM
            glMatrixMode(GL_MODELVIEW);

            glViewport(0, height / 2 + 1, width / 2 + 1, height / 2);
            glPushMatrix();
            glTranslatef(0,-3,0);
            glRotatef(90, 0, -1, 0);

            scene(bot_fwd, bot_inv);

            glPopMatrix();

            glViewport(width / 2 + 1, height / 2 + 1, width / 2, height / 2);
            glPushMatrix();
            glTranslatef(0,-3,0);
            scene(bot_fwd, bot_inv);
            glPopMatrix();

            glViewport(width / 2 + 1, 0, width / 2, height / 2);
            glPushMatrix();
            //glTranslatef( -2,0,0);
            glRotatef(90, 1, 0, 0);
            scene(bot_fwd, bot_inv);
            glPopMatrix();

            glMatrixMode(GL_PROJECTION);
        }

        glPopMatrix();

        draw_hud(bot_inv);

        glFlush();

        SDL_GL_SwapWindow(sdl_window);
    }
#endif

    int do_kins_fwd = 1;
    int do_kins_inv = 0;

    void jog_joint(bot_t* bot, int i, double amount) {
        double tmp = bot->j[i].pos + amount;

        if (amount > 0) {
            bot->j[i].pos = fmin(tmp, bot->j[i].max);
            do_kins_fwd = 1;
        } else {
            bot->j[i].pos = fmax(tmp, bot->j[i].min);
            do_kins_fwd = 1;
        }
    }

    void move_tool(bot_t* bot, int i, double amount) {
        bot->t[12+i] += amount;
        do_kins_inv = 1;
    }

    void rotate_tool(bot_t* bot, double a, double x, double y, double z) {
        rotate_m_axyz(bot->t, a, x, y, z);
        do_kins_inv = 1;
    }

#ifdef HAVE_SPACENAV

    typedef struct {
        int fd;
        int pos[6];
        int key[2];
    } spacenav_t;

#define test_bit(bit, array) (array[bit/8] & (1<<(bit%8)))

    int spacenav_open(void) {
        char fname[20];
        struct input_id id;

        int i = 0;
        while (i < 64) {
            snprintf(fname, sizeof(fname), "/dev/input/event%d", i++);
            int fd = open(fname, O_RDWR | O_NONBLOCK);
            if (fd > 0) {
                ioctl(fd, EVIOCGID, &id);

                if (id.vendor == 0x046d && (id.product == 0xc626 || id.product == 0xc623 || id.product == 0xc603)) {
                    // fprintf(stderr, "Using device: %s\n", fname);
                    return fd;
                }

                close(fd);
            }
        }

        return 0;
    }

    void spacenav_close(spacenav_t *s) {
        if (s->fd) {
            close(s->fd);
        }
    }

    void spacenav_read(spacenav_t *s) {
        struct input_event ev;

        int n;
        while ((n = read(s->fd, &ev, sizeof(struct input_event))) > 0) {
            // fprintf(stderr, "spacenav_read %d bytes.\n", n);

            if (n >= sizeof(struct input_event)) {
                switch (ev.type) {
                case EV_KEY:
                    s->key[ev.code] = ev.value; // fprintf(stderr, "Key %d pressed %d.\n", ev.code, ev.value);
                    break;
                case EV_REL:
                    s->pos[ev.code] = ev.value; // fprintf(stderr, "REL %d %d\n", ev.code, ev.value);
                    break;
                case EV_ABS:
                    s->pos[ev.code] = ev.value; // fprintf(stderr, "ABS %d %d\n", ev.code, ev.value);
                    break;
                default:
                    break;
                }
            }
        }
    }

#endif

    void update_model(bot_t *bot_fwd, bot_t* bot_inv, int do_kins_fwd, int do_kins_inv) {
        if (do_kins_inv) {
            kins_inv(bot_inv);
            if (bot_inv->err == 0) {
                kins_fwd(bot_inv);
                memcpy(bot_fwd, bot_inv, sizeof(bot_t));
            } else {
                memcpy(bot_inv, bot_fwd, sizeof(bot_t));
            }
        } else if (do_kins_fwd) {
            kins_fwd(bot_fwd);
            memcpy(bot_inv, bot_fwd, sizeof(bot_t));
        }
    }

#ifdef HAVE_MOSQUITTO
void on_mosquitto_publish(struct mosquitto *mosq, void *userdata, int mid) {
    // fprintf(stderr, "on_mosquitto_publish()");
}
#endif

#ifdef HAVE_SOCKET

    int sock_printf(int sock, const char * format, ...) {
        char buffer[256];
        va_list args;

        if (!sock) {
            return -1;
        }
	
        va_start (args, format);
        vsnprintf (buffer, 255, format, args);

        int err = write(sock, buffer, strlen(buffer));

        va_end(args);

        return err;
    }

    void sock_close(int sock) {
      if (sock) {
	close(sock);
      }
    }

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>

    typedef struct {
      int opt;
      int master_socket, addrlen, new_socket, client_socket[30], max_clients, activity, i, valread, sd;
      int max_sd;
      struct sockaddr_in address;

      char buffer[1025];
      fd_set readfds;

      char *message;
    } net_t;

    int net_init(net_t *net, int port) {
      int i;
      
      net->opt = 1; // allow multiple connections
      net->max_clients = 30;
      net->message = "ECHO Daemon v1.0 \r\n";
      
      for (i = 0; i < net->max_clients; i++) {
	net->client_socket[i] = 0;
      }
      
      if ((net->master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0) {
	perror("socket failed");
	return (EXIT_FAILURE);
        }
      
      if (setsockopt(net->master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&net->opt, sizeof(net->opt)) < 0) {
	perror("setsockopt");
	return (EXIT_FAILURE);
      }
      
      net->address.sin_family = AF_INET;
      net->address.sin_addr.s_addr = INADDR_ANY;
      net->address.sin_port = htons(port);
      
      if (bind(net->master_socket, (struct sockaddr *)&net->address, sizeof(net->address))<0) {
	perror("bind failed");
	return (EXIT_FAILURE);
      }
      printf("Listening on port %d \n", port);
      
      if (listen(net->master_socket, 3) < 0) {
	perror("listen");
	return (EXIT_FAILURE);
      }
      
      net->addrlen = sizeof(net->address);
      
      return EXIT_SUCCESS;
    }
    
#endif

#ifdef HAVE_SDL
#ifdef HAVE_PNG
    void screenshot(int x, int y, const char * filename) {
        unsigned char pixels[width * height * 6];
        SDL_Surface *infoSurface = SDL_GetWindowSurface(sdl_window);
        SDL_Surface *sdl_screenshot = SDL_CreateRGBSurfaceFrom(pixels, infoSurface->w, infoSurface->h, infoSurface->format->BitsPerPixel, infoSurface->w * infoSurface->format->BytesPerPixel, infoSurface->format->Rmask, infoSurface->format->Gmask, infoSurface->format->Bmask, infoSurface->format->Amask);
	
        SDL_RenderReadPixels(sdl_renderer, &infoSurface->clip_rect, infoSurface->format->format, pixels, infoSurface->w * infoSurface->format->BytesPerPixel);
        SDL_SavePNG(sdl_screenshot, filename);
        SDL_FreeSurface(sdl_screenshot);
        SDL_FreeSurface(infoSurface);
        infoSurface = NULL;
    }
#endif
#endif

void handle_signal(int signal) {
  sigset_t pending;

  // Find out which signal we're handling
  switch (signal) {
  case SIGHUP:
    break;
  case SIGUSR1:
    break;
  case SIGINT:
    done = 1;
  default:
    return;
  }

  sigpending(&pending);
  if (sigismember(&pending, SIGHUP)) {
  }
  if (sigismember(&pending, SIGUSR1)) {
  }
}

int main(int argc, char** argv) {
  struct sigaction sa;
  sa.sa_handler = &handle_signal;
  sa.sa_flags = SA_RESTART;
  sigfillset(&sa.sa_mask);

  if (sigaction(SIGHUP, &sa, NULL) == -1) {
    perror("Error: cannot handle SIGHUP");
  }

  if (sigaction(SIGUSR1, &sa, NULL) == -1) {
    perror("Error: cannot handle SIGUSR1");
  }

  if (sigaction(SIGINT, &sa, NULL) == -1) {
    perror("Error: cannot handle SIGINT");
  }

#ifdef HAVE_SOCKET
#define PORT 8888
        net_t net;
#endif

#ifdef ENABLE_FPS_LIMIT
        unsigned int ft = 0, frames;
#endif

        bot_t bot_fwd, bot_inv;

#ifdef HAVE_SDL
        int do_sdl = 0;

        SDL_Event ev;
        const Uint8 *keys = SDL_GetKeyboardState(0);

        int sdlk_tab_pressed = 0;

        int sdl_flags = SDL_WINDOW_SHOWN;
        SDL_DisplayMode sdl_displaymode;
#endif
	
#ifdef HAVE_NCURSES
        int do_curses = 0;
#endif

        int do_help = 0;
        int do_version = 0;
        int verbose = 0;

        int i = 0;
        while (++i < argc) {
#define OPTION_SET(longopt,shortopt) (strcmp(argv[i], longopt)==0 || strcmp(argv[i], shortopt)==0)
#define OPTION_VALUE ((i+1 < argc)?(argv[i+1]):(NULL))
#define OPTION_VALUE_PROCESSED (i++)
	  if (OPTION_SET("--help", "-h")) {
            do_help = 1;
	  } else if (OPTION_SET("--version", "-v")) {
            do_version = 1;
#ifdef HAVE_SDL
	  } else if (OPTION_SET("--fullscreen", "-f")) {
	    sdl_flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
	  } else if (OPTION_SET("--sdl", "-s")) {
                do_sdl = 1;
#endif
#ifdef HAVE_HAL
	  } else if (OPTION_SET("--hal", "-l")) {
	    do_hal = 1;
#endif
#ifdef HAVE_TRAJGEN
	  } else if (OPTION_SET("--trajgen-test", "-t")) {
	    do_trajgen_test = 1;
#endif
#ifdef HAVE_NCURSES
	  } else if (OPTION_SET("--curses", "-c")) {
	    do_curses = 1;
#endif
#ifdef HAVE_SOCKET
	  } else if (OPTION_SET("--net", "-n")) {
	    do_net = 1;
#endif
#ifdef HAVE_ZMQ
	  } else if (OPTION_SET("--zmq", "-z")) {
	    do_zmq = 1;
#endif
#ifdef HAVE_MOSQUITTO
          } else if (OPTION_SET("--mqtt", "-m")) {
            do_mosquitto = 1;
#endif
          } else if (OPTION_SET("--verbose", "-x")) {
            verbose++;
	  } else {
	    fprintf(stderr, "Unknown option: %s\n", argv[i]);
	    do_help = 1;
	  }
        }

        if (do_version) {
	  fprintf(stdout, "%s %s\n", argv[0], PROGRAM_VERSION);
            return EXIT_SUCCESS;
	}
	
        if (do_help) {
            fprintf(stdout, "Usage: %s [OPTIONS]\n\n"
                    " Where [OPTIONS] are zero or more of the following:\n\n"
#ifdef HAVE_SDL
                    "    [-s|--sdl]               SDL window mode\n"
                    "    [-f|--fullscreen]        Fullscreen mode\n"
#endif
#ifdef HAVE_NCURSES
                    "    [-c|--curses]            Curses text mode\n"
#endif
#ifdef HAVE_HAL
                    "    [-l|--hal]               HAL mode\n"
#endif
#ifdef HAVE_TRAJGEN
                    "    [-t|--trajgen-test]      Do trajgen test\n"
#endif
#ifdef HAVE_SOCKET
                    "    [-n|--net]               Network server mode\n"
#endif
#ifdef HAVE_ZMQ
                    "    [-z|--zmq]               ZMQ server mode\n"
#endif
#ifdef HAVE_MOSQUITTO
                    "    [-m|--mqtt]              MQTT client mode\n"
#endif
                    "    [-x|--verbose]           Show verbose information\n\n"
                    "    [-h|--help]              Show help information\n\n"
                    "    [-v|--version]           Show version number\n\n"
                    , argv[0]);
            return EXIT_SUCCESS;
        }

#ifdef HAVE_HAL
	if (do_hal) {
	  // initialize component
	  hal_comp_id = hal_init(modname);
	  if (hal_comp_id < 1) {
            fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
            goto fail0;
	  }

	  // allocate hal memory
	  hal_pos_data = hal_malloc(sizeof(hal_t));
	  if (hal_pos_data == NULL) {
            fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
            goto fail1;
	  }
	  
	  // register pins
	  for (i = 0; i < 5; i++) {
	    if (hal_pin_float_newf(HAL_OUT, &(hal_pos_data->axis[i]), hal_comp_id, "%s.%d.pos-cmd", MODULE_NAME, i) != 0) {
	      fprintf(stderr, "%s: ERROR: unable to register hal pin %s.%d.pos-cmd\n", modname, MODULE_NAME, i);
	      goto fail1;
            }
	  }
	}
#endif

#ifdef HAVE_ZMQ
	if (do_zmq && verbose >= 1) {
	  int major, minor, patch;
	  zmq_version (&major, &minor, &patch);
	  fprintf(stderr, "Using 0MQ version %d.%d.%d\n", major, minor, patch);
	}
#endif

#ifdef HAVE_MOSQUITTO
        if (do_mosquitto) {

          mosquitto_lib_init();

          if (verbose >= 1) {
            int major, minor, revision;
            mosquitto_lib_version(&major, &minor, &revision);
            fprintf(stderr, "Using Mosquitto version %d.%d.%d\n", major, minor, revision);
          }

          mosq = mosquitto_new("rm501", true, NULL);
          mosquitto_publish_callback_set(mosq, on_mosquitto_publish);
          int keepalive = 60;
          mosquitto_connect(mosq, "localhost", 1883, keepalive);
        }
#endif

#ifdef HAVE_SOCKET
	if (do_net) {
	  net_init(&net, 8888);
	}
#endif

#ifdef HAVE_NCURSES
	if (do_curses) {
	  initscr();
	  nonl();
	  cbreak();
	  noecho();
	  keypad(stdscr,1);
	  timeout(0); // getch non-blocking
	  if (has_colors()) {
	    start_color();
	    init_pair(1, COLOR_WHITE,COLOR_BLUE);
	    init_pair(2, COLOR_BLUE,COLOR_WHITE);
	    init_pair(3, COLOR_RED,COLOR_WHITE);
	  }
	  curs_set(0);
	  
	  struct winsize w;
	  ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
	  
	  menubar    = subwin(stdscr, 1, w.ws_col, 0, 0);
	  messagebar = subwin(stdscr, 1, w.ws_col-1, w.ws_row-1, 1);
	  wbkgd(menubar, COLOR_PAIR(2));
	  wbkgd(messagebar, COLOR_PAIR(2));
	  
	  werase(menubar);
	  wrefresh(menubar);
	  
	  werase(messagebar);
	  wrefresh(messagebar);
	  
	  wprintw(menubar, "Mitsubishi RM-501 Movemaster II Robot Simulator");
	  wprintw(messagebar, "Status: DISCONNECTED, OFFLINE");
	}
#endif
    
#ifdef HAVE_SOCKET
    if (do_net) {
      FD_ZERO(&net.readfds);
      FD_SET(net.master_socket, &net.readfds);
      net.max_sd = net.master_socket;
      
      for ( i = 0 ; i < net.max_clients ; i++) {
        net.sd = net.client_socket[i];
      
	if (net.sd > 0) {
          FD_SET( net.sd , &net.readfds);
        }
      
      if (net.sd > net.max_sd) {
        net.max_sd = net.sd;
      }
    }
      
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
      
    net.activity = select( net.max_sd + 1 , &net.readfds , NULL , NULL , &tv);

    if ((net.activity < 0) && (errno!=EINTR)) {
      printf("select error");
    }
      
    if (FD_ISSET(net.master_socket, &net.readfds)) {
      if ((net.new_socket = accept(net.master_socket,
	  (struct sockaddr *)&net.address, (socklen_t*)&net.addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
      }
      
      printf("New connection , socket fd is %d , ip is : %s , port : %d \n",
        net.new_socket , inet_ntoa(net.address.sin_addr) , ntohs(net.address.sin_port));
      
      // sock_printf(net.new_socket, "P %ld %ld %ld %ld %ld\n", net_bot.step[0], net_bot.step[1], net_bot.step[2], net_bot.step[3], net_bot.step[4]);
      
      for (i = 0; i < net.max_clients; i++) {
        if (net.client_socket[i] == 0) {
          net.client_socket[i] = net.new_socket;
          printf("Adding to list of sockets as %d\n" , i);
      
          break;
        }
      }
    }
      
    for (i = 0; i < net.max_clients; i++) {
      net.sd = net.client_socket[i];
      
      if (FD_ISSET( net.sd , &net.readfds)) {
        if ((net.valread = read( net.sd , net.buffer, 1024)) == 0) { // check disconnect
          getpeername(net.sd , (struct sockaddr*)&net.address , (socklen_t*)&net.addrlen);
          printf("Host disconnected , ip %s , port %d \n",
            inet_ntoa(net.address.sin_addr) , ntohs(net.address.sin_port));
      
          close(net.sd);
          net.client_socket[i] = 0;
        } else {
          net.buffer[net.valread] = '\0';
          send(net.sd , net.buffer , strlen(net.buffer) , 0 );
        }
      }
    }
      
    for (i = 0; i < net.max_clients; i++) {
      net.sd = net.client_socket[i];
      
      // sock_printf(net.sd, "P %ld %ld %ld %ld %ld\n",
      // net_bot.step[0], net_bot.step[1], net_bot.step[2], net_bot.step[3], net_bot.step[4]);
    }
  }
#endif

#ifdef HAVE_SPACENAV
    spacenav_t sn = {0};
    
    sn.fd = spacenav_open();
#endif

#ifdef HAVE_SDL
    if (do_sdl) {

      if (verbose >= 1) {
        SDL_version linked;
        SDL_GetVersion(&linked);
        SDL_version compiled;
        SDL_VERSION(&compiled);
        fprintf(stderr, "Compiled  with SDL version %d.%d.%d.\n", compiled.major, compiled.minor, compiled.patch);
        fprintf(stderr, "Linked against SDL version %d.%d.%d.\n", linked.major, linked.minor, linked.patch);
      }

#ifdef HAVE_JOYSTICK
      SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
#endif
      if (SDL_Init(SDL_INIT_EVERYTHING) < 0)   {
	fprintf(stderr, "Unable to initialise SDL: %s\n", SDL_GetError());
	exit(EXIT_FAILURE);
      }

#ifdef HAVE_JOYSTICK
      if (SDL_NumJoysticks() < 1) {
	printf( "Warning: No joysticks connected!\n" );
      } else {
	joy = SDL_JoystickOpen( 0 );
	if (joy == NULL) {
	  printf( "Warning: Unable to open game controller! SDL Error: %s\n", SDL_GetError() );
	}
      }
#endif

      if (TTF_Init() < 0) {
	fprintf(stderr, "Unable to initialise SDL_ttf.\n");
	exit(EXIT_FAILURE);
      }
      
      char* sdl_font_file = "/usr/share/fonts/truetype/ttf-dejavu/DejaVuSansMono.ttf";
      
      sdl_font = TTF_OpenFont(sdl_font_file, 15);
      
      if (!sdl_font) {
	fprintf(stderr, "%s: %s\n", SDL_GetError(), sdl_font_file);
	exit(EXIT_FAILURE);
      }
      
      if (SDL_GetCurrentDisplayMode(0, &sdl_displaymode) != 0) {
	fprintf(stderr, "Could not get display mode for video display #%d: %s", 0, SDL_GetError());
	exit(EXIT_FAILURE);
      }
      
      /*
        if (sdl_flags & SDL_WINDOW_FULLSCREEN) {
  	  width  = sdl_displaymode.w;
	  height = sdl_displaymode.h;
	}*/
      
      sdl_window = SDL_CreateWindow("Mitsubishi RM-501 Movemaster II Robot Simulator",
				    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
				    width, height, sdl_flags);
      
      sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_ACCELERATED);
      
      SDL_RenderClear(sdl_renderer);
      
      SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
      // SDL_GL_SetSwapInterval(1);
      
      {
	GLfloat pos[4]   = {3., 5., 2., 1.};
	GLfloat white[4] = {0.75, 0.75, 0.75, 1.};
	GLfloat black[4] = {0., 0., 0., 0.};
	
	glDisable (GL_LIGHTING);
	glEnable (GL_LIGHT1);
	glLightfv (GL_LIGHT1, GL_POSITION, pos);
	glLightfv (GL_LIGHT1, GL_DIFFUSE, white);
	glLightfv (GL_LIGHT1, GL_SPECULAR, black);
	
	glEnable (GL_COLOR_MATERIAL);
	glColorMaterial (GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv (GL_FRONT, GL_SPECULAR, black);
      }
      
      glEnable(GL_TEXTURE_2D);

      glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

      glEnable(GL_LINE_SMOOTH);
      glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

      glEnable( GL_POLYGON_SMOOTH );
      glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

      glDepthFunc(GL_LEQUAL);
      glEnable(GL_DEPTH_TEST);
      glShadeModel(GL_SMOOTH);
      
      glMatrixMode(GL_PROJECTION);
      
      glLoadIdentity();
      gluPerspective(45, width / (height * 1.0), 0.1, 500);
      
      glMatrixMode(GL_MODELVIEW);
#endif
    }

    bot_fwd.d1 = 2.3;
    bot_fwd.d5 = 1.3;
    bot_fwd.a2 = 2.2;
    bot_fwd.a3 = 1.6;
    
    bot_fwd.j[0].min = -120;
    bot_fwd.j[0].max = 180;
    
    bot_fwd.j[1].min = -30;
    bot_fwd.j[1].max = 100;
    
    bot_fwd.j[2].min = -100;
    bot_fwd.j[2].max = 0;
    
    bot_fwd.j[3].min = -15;
    bot_fwd.j[3].max = 195;
    
    bot_fwd.j[4].min = -360;
    bot_fwd.j[4].max = 360;
    
    bot_fwd.j[0].pos = 0;
    bot_fwd.j[1].pos = 90;
    bot_fwd.j[2].pos = -90;
    bot_fwd.j[3].pos = 0;
    bot_fwd.j[4].pos = 0;
    
    memcpy(&bot_inv, &bot_fwd, sizeof(bot_t));
    kins_inv(&bot_inv);
    kins_fwd(&bot_inv);
    kins_fwd(&bot_fwd);
    
#ifdef HAVE_SDL
    if (do_sdl) {
#ifdef ENABLE_FPS_LIMIT
      frames = 0;
      ft = SDL_GetTicks();
#endif
    }
#endif

#ifdef HAVE_HAL
    if (do_hal) {
      hal_ready(hal_comp_id);
    }
#endif

#ifdef HAVE_TRAJGEN
    tg_init(1000, 25000, 1000.0, 1, 3);
    tg_echk(trajgen_switch_state(TRAJ_STATE_COORDINATED));
    while (!tg_tg.is_done) tg_echk(trajgen_tick());
    tg_speed = 100;
    tg_accel = 5000;
    tg_blending = 15;
    tg_line(1.6, 3.2, 0.0);
    while (!tg_tg.is_done) tg_echk(trajgen_tick());
#endif
    
    while (!done) {

        static double old_pos[5];
        
        for (int i = 0; i < 5; i++) {
            old_pos[i] = bot_fwd.j[i].pos;
        }

#ifdef HAVE_SDL
  if (do_sdl) {
    SDL_PollEvent(&ev);
	  
    if (ev.type == SDL_QUIT ||
      (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE))
      done = 1;
    
    if (ev.type == SDL_KEYUP && ev.key.keysym.sym == SDLK_TAB) {
      sdlk_tab_pressed = 0;
    }

    double d   = 0.05;
    double cnt = 1.00;
	  
    if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL]) {
      d   *= 2;
      cnt *= 2;
    }
    
    if (keys[SDL_SCANCODE_Q]) { jog_joint(&bot_fwd, 0,  cnt); }
    if (keys[SDL_SCANCODE_W]) { jog_joint(&bot_fwd, 1,  cnt); }
    if (keys[SDL_SCANCODE_E]) { jog_joint(&bot_fwd, 2,  cnt); }
    if (keys[SDL_SCANCODE_R]) { jog_joint(&bot_fwd, 3,  cnt); }
    if (keys[SDL_SCANCODE_T]) { jog_joint(&bot_fwd, 4,  cnt); }
    
    if (keys[SDL_SCANCODE_A]) { jog_joint(&bot_fwd, 0, -cnt); }
    if (keys[SDL_SCANCODE_S]) { jog_joint(&bot_fwd, 1, -cnt); }
    if (keys[SDL_SCANCODE_D]) { jog_joint(&bot_fwd, 2, -cnt); }
    if (keys[SDL_SCANCODE_F]) { jog_joint(&bot_fwd, 3, -cnt); }
    if (keys[SDL_SCANCODE_G]) { jog_joint(&bot_fwd, 4, -cnt); }

    if (!keys[SDL_SCANCODE_LSHIFT] && !keys[SDL_SCANCODE_RSHIFT]) {
      if (keys[SDL_SCANCODE_LEFT])     { move_tool(&bot_inv, 0, -d); }
      if (keys[SDL_SCANCODE_RIGHT])    { move_tool(&bot_inv, 0,  d); }
      if (keys[SDL_SCANCODE_I])        { move_tool(&bot_inv, 1,  d); }
      if (keys[SDL_SCANCODE_K])        { move_tool(&bot_inv, 1, -d); }
      if (keys[SDL_SCANCODE_UP])       { move_tool(&bot_inv, 2,  d); }
      if (keys[SDL_SCANCODE_DOWN])     { move_tool(&bot_inv, 2, -d); }
    } else {
      if (keys[SDL_SCANCODE_UP])  {
        rotate_tool(&bot_inv, -cnt, sin(deg2rad(bot_inv.j[0].pos)), 0, cos(deg2rad(bot_inv.j[0].pos)));
      }
      if (keys[SDL_SCANCODE_DOWN]) {
        rotate_tool(&bot_inv,  cnt, sin(deg2rad(bot_inv.j[0].pos)), 0, cos(deg2rad(bot_inv.j[0].pos)));
      }
      if (keys[SDL_SCANCODE_LEFT])  {
        rotate_tool(&bot_inv, -cnt, 0, 1, 0);
      }
      if (keys[SDL_SCANCODE_RIGHT]) {
        rotate_tool(&bot_inv,  cnt, 0, 1, 0);
      }
    }
	  
#ifdef PROJ2
///////////////////////////////////////////////////////////////////////////////
    if (keys[SDL_SCANCODE_H]) {
      bot_fwd.j[0].tar= 30;
      bot_fwd.j[1].tar= 0;
      bot_fwd.j[2].tar= 0;
      bot_fwd.j[3].tar= 90;
      bot_fwd.j[4].tar= 0;
    }
	  
    if (keys[SDL_SCANCODE_N]) {
      bot_fwd.j[0].tar= -120;
      bot_fwd.j[1].tar= 100;
      bot_fwd.j[2].tar= -90;
      bot_fwd.j[3].tar= 0;
      bot_fwd.j[4].tar= 0;
    }
#ifdef PROJ3
    // C: move in a circle
    if (keys[SDL_SCANCODE_C]) {
      bot_inv.proj3counter %= 360;
      bot_inv.proj3counter++;
      double theta = deg2rad(bot_inv.proj3counter);
      bot_inv.t[12] = 2.8 + cos(theta) * 0.6;
      bot_inv.t[13] = 2.4;
      bot_inv.t[14] = sin(theta) * 0.6;
	    
      do_kins_inv = 1;
    }
	  
    // V: reset circle counter
    if (keys[SDL_SCANCODE_V]) {
      bot_inv.proj3counter = 0;
    }
	  
#endif
    // H/N: move to home positions (try with the shift key to see the difference)
    if (keys[SDL_SCANCODE_H] || keys[SDL_SCANCODE_N]) {
      i = 1;
      while (1) {
        if (bot_fwd.j[i].tar > bot_fwd.j[i].pos) {
    	  if (bot_fwd.j[i].tar - bot_fwd.j[i].pos < cnt) {
	    bot_fwd.j[i].pos = bot_fwd.j[i].tar;
          } else {
            jog_joint(&bot_fwd, i, cnt); 
            if (!keys[SDL_SCANCODE_LSHIFT] && !keys[SDL_SCANCODE_RSHIFT]) {
              break;
	    }
	  }
        }
        if (bot_fwd.j[i].tar < bot_fwd.j[i].pos) {
          if (bot_fwd.j[i].tar - bot_fwd.j[i].pos > -cnt) {
	    bot_fwd.j[i].pos = bot_fwd.j[i].tar;
	  } else {
	    jog_joint(&bot_fwd, i, -cnt);
	    if (!keys[SDL_SCANCODE_LSHIFT] && !keys[SDL_SCANCODE_RSHIFT]) {
	      break;
	  }
	}
      }
      if (i >= 1 && i <= 3)
        i++;
      else if (i==4)
        i=0;
      else if (i==0)
        break;
      }
    }
///////////////////////////////////////////////////////////////////////////////
#endif

    if (ev.type == SDL_KEYDOWN) {
	    
      switch( ev.key.keysym.sym ) {
      case SDLK_TAB:
	if (!sdlk_tab_pressed) {
	  if (view_mode == 0) {
	    view_mode = 1;
	  } else {
	    view_mode = 0;
	  }
	  sdlk_tab_pressed = 1;
	}
	break;
      case SDLK_F1:
	bot_fwd.j[0].pos = 0;
	bot_fwd.j[1].pos = 45;
	bot_fwd.j[2].pos = -45;
	bot_fwd.j[3].pos = 0;
	bot_fwd.j[4].pos = 0;
	do_kins_fwd = 1;
	break;
      default:
	break;
      }
    }
#ifdef HAVE_JOYSTICK
    if (ev.type == SDL_JOYAXISMOTION) {
      #define JOY_SCALE (1.0 / (32768.0 * 2.0))
      if (ev.jaxis.which == 0) {
	// X axis motion
	if (ev.jaxis.axis == 0) {
	  //Left of dead zone
	  if (ev.jaxis.value < -JOYSTICK_DEAD_ZONE) {
	    move_tool(&bot_inv, 0, (double)(d * ev.jaxis.value * JOY_SCALE));
	  }
	  //Right of dead zone
	  else if (ev.jaxis.value > JOYSTICK_DEAD_ZONE) {
	    move_tool(&bot_inv, 0, (double)(d * ev.jaxis.value * JOY_SCALE));
	  }
	} // Y axis motion
	else if (ev.jaxis.axis == 1) {
	  //Left of dead zone
	  if (ev.jaxis.value < -JOYSTICK_DEAD_ZONE || ev.jaxis.value > JOYSTICK_DEAD_ZONE) {
	    move_tool(&bot_inv, 2, (double)(-d * ev.jaxis.value * JOY_SCALE));
	  }
	} // Z axis motion
	else if (ev.jaxis.axis == 3) {
	  //Left of dead zone
	  if (ev.jaxis.value < -JOYSTICK_DEAD_ZONE || ev.jaxis.value > JOYSTICK_DEAD_ZONE) {
	    move_tool(&bot_inv, 1, (double)(-d * ev.jaxis.value * JOY_SCALE));
	  }
	}
      }
    }
#endif
  }
#endif
	  
#ifdef HAVE_SPACENAV
  if (sn.fd) {
    spacenav_read(&sn);
	    
#define JOG_MIN 15
#define JOG_SPEED 0.0001
#define JOG_SPEED_ROT 0.008
    if (abs(sn.pos[0]) > JOG_MIN) {
      move_tool(&bot_inv, 0, sn.pos[0] * JOG_SPEED);
    }
    
    if (abs(sn.pos[1]) > JOG_MIN) {
      move_tool(&bot_inv, 2, -sn.pos[1] * JOG_SPEED);
    }
	    
    if (abs(sn.pos[2]) > JOG_MIN) {
      move_tool(&bot_inv, 1, -sn.pos[2] * JOG_SPEED);
    }
    
    if (abs(sn.pos[5]) > JOG_MIN) {
      rotate_tool(&bot_inv, -sn.pos[5] * JOG_SPEED_ROT, 0, 1, 0);
    }
    
#ifdef SPACENAV_JOG_A
    if (abs(sn.pos[4]) > JOG_MIN) {
      rotate_tool(&bot_inv, -sn.pos[4] * JOG_SPEED_ROT,
		  sin(deg2rad(bot_inv.j[0].pos)), 0, cos(deg2rad(bot_inv.j[0].pos)));
    }
#endif
    if (sn.key[0]) {
      done = 1;
    }
  }
#endif

#ifdef HAVE_TRAJGEN
  if (do_trajgen_test) {
      trajgen_tick();
      
      double oldx = bot_inv.t[12];
      double oldy = bot_inv.t[13];
      double oldz = bot_inv.t[14];
      
      double dx = tg_tg.joints[0].position - oldx;
      double dy = tg_tg.joints[1].position - oldy;
      double dz = tg_tg.joints[2].position - oldz;
      
      move_tool(&bot_inv, 0, dx);
      move_tool(&bot_inv, 1, dy);
      move_tool(&bot_inv, 2, dz);
      
      if (trajgen_num_queued() < 2) {
          tg_line(2.8 + randpos(0.5), 1.0 + randpos(1), randpos(2));
      }
  }
#endif
    
  update_model(&bot_fwd, &bot_inv, do_kins_fwd, do_kins_inv);

  if (do_kins_inv || do_kins_fwd) {
    memcpy(&bot_inv, &bot_fwd, sizeof(bot_t));
    kins_inv(&bot_inv);
    kins_fwd(&bot_inv);
    kins_fwd(&bot_fwd);
  }

  do_kins_fwd = 0;
  do_kins_inv = 0;

  // update joint velocity values
  for (int i = 0; i < 5; i++) {
      bot_fwd.j[i].vel = bot_fwd.j[i].pos - old_pos[i];
      bot_inv.j[i].vel = bot_inv.j[i].pos - old_pos[i];
  }

#ifdef HAVE_SDL
  if (do_sdl) {
    display(&bot_fwd, &bot_inv);
    SDL_RenderPresent(sdl_renderer);
    // screenshot(0, 0, "screenshot.png");
  }
#endif

#ifdef HAVE_NCURSES
  if (do_curses) {
    int key = getch();
    
    if (key=='q') {
      done = 1;
    }
    
    move(8,1);  printw("x: %7.2f", bot_inv.t[12]);
    move(9,1);  printw("y: %7.2f", bot_inv.t[13]);
    move(10,1); printw("z: %7.2f", bot_inv.t[14]);
    
    double r, p, y;
    
    pmMatRpyConvert(bot_inv.t, &r, &p, &y);
	      
    move(12,1); printw("a: %7.2f", rad2deg(r));
    move(13,1); printw("b: %7.2f", rad2deg(p));
    move(14,1); printw("c: %7.2f", rad2deg(y));

    int i;
    for (i = 0; i < 5; i++) {
      move(18,1+i*11); printw("%7.2f", bot_inv.j[i].pos);
    }
	      
    move(20,1);
    if (strlen(bot_inv.msg)) {
      printw(bot_inv.msg);
    } else {
      printw("                             ");
    }
	    
    touchwin(stdscr);
    refresh();
  }
#endif
	    
#ifdef HAVE_HAL
  if (do_hal) {
    for (i = 0; i < 5; i++) {
      *(hal_pos_data->axis[i]) = bot_fwd.j[i].pos;
    }
  }
#endif

#ifdef HAVE_MOSQUITTO
  char topic[32];
  char payload[64];

  if (do_mosquitto) {
    for (i = 0; i < 5; i++) {
      snprintf(topic,   sizeof topic,   "%s/%d/pos", "rm501", i);
      snprintf(payload, sizeof payload, "%f", bot_fwd.j[i].pos);
      mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 0, false);
    }
  }
#endif
  
#ifdef HAVE_SDL
  if (do_sdl) {
#ifdef ENABLE_FPS_LIMIT
    while (frames*1000.0/((float)(SDL_GetTicks()-ft+1))>(float)(DEFAULT_FPS)) {
      SDL_Delay(10);
    }
    frames++;
#endif
  }
#endif
  } // while ! done
    
#ifdef HAVE_SDL
  if (do_sdl) {
    SDL_RenderClear(sdl_renderer);

    if (sdl_font) {
      TTF_CloseFont(sdl_font);
    }

#ifdef HAVE_JOYSTICK
    if (joy) {
      SDL_JoystickClose(joy);
      joy = NULL;
    }
#endif

    TTF_Quit();
	
    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(sdl_window);
    
    SDL_Quit();
  }
#endif
	
#ifdef HAVE_SPACENAV
  spacenav_close(&sn);
#endif

#ifdef HAVE_NCURSES
  delwin(menubar);
  delwin(messagebar);
  endwin();
#endif

#ifdef HAVE_HAL
fail1:
  if (do_hal) {
    hal_exit(hal_comp_id);
  }
fail0:
#endif

#ifdef HAVE_MOSQUITTO
  if (do_mosquitto) {
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
  }
#endif

  return EXIT_SUCCESS;
}
