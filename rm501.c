/*
 * rm501.c - Mitsubishi RM-501 Movemaster II Robot Simulator
 *
 * Copyright (C) 2013-2022 Jakob Flierl <jakob.flierl@gmail.com>
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
#define HAVE_AUDIO    // working, requires HAVE_SDL
#define HAVE_JOYSTICK // working, requires HAVE_SDL
//#define HAVE_PNG         // working, requires HAVE_SDL
#define ENABLE_FPS_LIMIT // working, requires HAVE_SDL
#ifdef ENABLE_FPS_LIMIT
#define DEFAULT_FPS 30
#endif
#endif

//#define HAVE_SPACENAV        // working
//#define HAVE_HAL             // partially working
//#define HAVE_NCURSES         // unfinished
//#define HAVE_SERIAL          // unfinished
//#define HAVE_MQTT            // unfinished
//#define HAVE_ZMQ             // unfinished
//#define HAVE_TRAJGEN         // unfinished

// see http://www2.ece.ohio-state.edu/~zheng/ece5463/proj2/5463-Project-2-FA2015.pdf
#define PROJ2

// see http://www2.ece.ohio-state.edu/~zheng/ece5463/proj3/5463-Project-3-FA2015.pdf
#define PROJ3

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>  // EXIT_SUCCESS
#include <stdbool.h> // for bool, true, false
#include <stdint.h>
#include <inttypes.h>
#include <string.h> // memcpy()
#include <signal.h> // sigaction(), sigsuspend(), sig*()
#ifdef __MINW32__
#define _USE_MATH_DEFINES 1
#endif
#include <math.h>

#ifdef HAVE_ZMQ
#include <zmq.h>
#endif

#ifdef HAVE_MQTT
#include "mqtt.h"
#endif

#ifdef HAVE_SDL
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/glu.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_opengl.h>
#include <SDL2/SDL_image.h>
#ifndef GL_BGR
#define GL_BGR 0x80E0
#endif
#ifndef GL_BGRA
#define GL_BGRA 0x80E1
#endif
#endif

#ifdef HAVE_NCURSES
#include <ncursesw/curses.h>
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
#include "png.h"
#include "png.c"
#endif
#endif

#ifdef HAVE_TRAJGEN
#include "trajgen.h"
#endif

volatile int done = 0; // SIGINT or user exit requested

#ifdef HAVE_HAL
int do_hal = 0;
#include "hal.h"
#define MODULE_NAME "rm501"
char *modname = MODULE_NAME;

typedef struct
{
  hal_float_t *axis[5];
} hal_t;

int hal_comp_id;
hal_t *hal_pos_data;
#endif

#ifdef HAVE_SDL
int view_mode = 0;
int width = 640, height = 480;
TTF_Font *sdl_font;

SDL_Window *sdl_window;
SDL_GLContext sdl_glcontext;
SDL_Renderer *sdl_renderer;

#ifdef HAVE_AUDIO
int do_audio = 0;
double MIDDLE_C[5];
int middle_c_dist[5];
#endif
#endif

#ifdef HAVE_NCURSES
WINDOW *menubar, *messagebar;
#endif

#ifdef HAVE_SDL
#ifdef HAVE_PNG
SDL_Surface *png_shot;
#endif
#endif

#ifdef HAVE_JOYSTICK
SDL_Joystick *joy = NULL;
const int JOYSTICK_DEAD_ZONE = 2500;
#endif

#ifdef HAVE_ZMQ
int do_zmq = 0;
#endif

#ifdef HAVE_MQTT
int do_mqtt = 0;
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

double rnd()
{
  return ((double)rand()) / RAND_MAX; // NOLINT(cert-msc30-c, cert-msc50-cpp)
}

double roundd(double v, unsigned digits)
{
  for (unsigned i = 0; i < digits; i++)
    v *= 10.;
  v = round(v);
  for (unsigned i = 0; i < digits; i++)
    v /= 10.;
  return v;
}

double randpos(double max)
{
  return roundd(max * ((rnd() * 2) - 1), 2);
}

void tg_echk(unsigned errnom)
{
  if (errnom)
  {
    fprintf(stderr, "tg error: %s\n", trajgen_errstr(tg_tg.last_error.errnom, tg_err_str, 1024));
    exit(1); // XXX
  }
}

void tg_init(real_t max_speed, real_t max_accel, real_t sample_frequency,
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
  tg_cfg.sample_interval = 1.0 / sample_frequency;
  tg_cfg.interpolation_rate = interpolation_rate;
  pose_set_all(&tg_cfg.max_manual_velocities, max_speed);
  pose_set_all(&tg_cfg.max_manual_accelerations, max_accel);
  for (unsigned i = 0; i < tg_n_joints; i++)
  {
    tg_cfg.joints[i].max_velocity = max_speed;
    tg_cfg.joints[i].max_acceleration = max_speed;
  }
  tg_echk(trajgen_initialize(&tg_tg, tg_cfg));
}

void tg_line(double x, double y, double z)
{
  pose_t p;
  p.x = isnan(x) ? tg_last_pose.x : x;
  p.y = isnan(y) ? tg_last_pose.y : y;
  p.z = isnan(z) ? tg_last_pose.z : z;
  p.a = tg_last_pose.a;
  p.b = tg_last_pose.b;
  p.c = tg_last_pose.c;
  p.u = tg_last_pose.u;
  p.v = tg_last_pose.v;
  p.w = tg_last_pose.w;

  tg_last_pose = p;

  fprintf(stderr, "# line {x:%.6g, y:%.6g, z:%.6g, v:%.6g, a:%.6g, tol:%.6g }\n", p.x, p.y, p.z, tg_speed, tg_accel, tg_blending);
  tg_echk(trajgen_add_line(p, tg_speed, tg_accel));
}

#endif // HAVE_TRAJGEN

typedef struct
{
  double d1, d5, a2, a3; // dh parameters

  struct joint_s
  {
    double pos;
    double vel;
    double min;
    double max;
#ifdef PROJ2
    double tar;
#endif
  } j[5]; // joints

  struct cart_s
  {
    double vel;
  } cart[6]; // cartesian coordinates

  double t[16];   // tool matrix
  int err;        // error a to joint number if inverse kinematics fails
  char msg[2048]; // opt. message from inverse kinematics

#ifdef PROJ3
  int proj3counter;
#endif

  bool grip;

} bot_t;

#ifdef HAVE_AUDIO
short middle_c_gen()
{
  short out = 0;

#define AXES 5
  for (int i = 0; i < AXES; i++)
  {
    double volume = 0.05;
    middle_c_dist[i]++; /* Take one sample */
    int MIDDLE_C_SAMPLES = (int) (44100 / MIDDLE_C[i]);
    if (middle_c_dist[i] >= MIDDLE_C_SAMPLES)
      middle_c_dist[i] = 0;

    /* Low? */
    if (middle_c_dist[i] < MIDDLE_C_SAMPLES / 2)
      out += (short)(-32768 * volume / AXES);
    else
      /* High */
      out += (short)(32767 * volume / AXES);
  }
#undef AXES
  return out;
}

void fill_audio(void *data, Uint8 *stream, int len)
{
  /* Cast */
  short* buff = (short*)stream;
  len /= 2; /* Because we're now using shorts */
  /* Square */
  for (int i = 0; i < len; i += 2)
  {
    buff[i] = middle_c_gen(); /* Left */
    buff[i + 1] = buff[i];    /* Right, same as left */
  }
}

void open_audio()
{
  SDL_AudioSpec as;
  /* Fill out what we want */
  as.freq = 44100;
  as.format = AUDIO_S16SYS;
  as.channels = 2;
  as.samples = 1024;
  as.callback = fill_audio;
  /* Get it */
  SDL_OpenAudio(&as, NULL);
  /* Go! */
  SDL_PauseAudio(0);
}

void close_audio()
{
  SDL_CloseAudio();
}
#endif // HAVE_AUDIO

#define rad2deg(rad) ((rad) * (180.0 / M_PI))
#define deg2rad(deg) ((deg) * (M_PI / 180.0))
#define sq(x) ((x) * (x))

void rotate_m_axyz(double *mat, double angle, double x, double y, double z)
{
  double m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double s = sin(deg2rad(angle));
  double c = cos(deg2rad(angle));

  double mag = sqrt(sq(x) + sq(y) + sq(z));
  if (mag <= 1.0e-4)
  {
    return;
  } // no rotation, leave mat as-is
  x /= mag;
  y /= mag;
  z /= mag;

  double one_c = 1.0 - c;
#define M(row, col) m[col * 4 + row]
  M(0, 0) = (one_c * sq(x)) + c;
  M(0, 1) = (one_c * x * y) - z * s;
  M(0, 2) = (one_c * z * x) + y * s;

  M(1, 0) = (one_c * x * y) + z * s;
  M(1, 1) = (one_c * sq(y)) + c;
  M(1, 2) = (one_c * y * z) - x * s;

  M(2, 0) = (one_c * z * x) - y * s;
  M(2, 1) = (one_c * y * z) + x * s;
  M(2, 2) = (one_c * sq(z)) + c;
#undef M

  for (int i = 0; i < 4; i++)
  {
#define A(row, col) mat[(col << 2) + row]
#define B(row, col) m[(col << 2) + row]
    double ai0 = A(i, 0), ai1 = A(i, 1), ai2 = A(i, 2), ai3 = A(i, 3);
    A(i, 0) = ai0 * B(0, 0) + ai1 * B(1, 0) + ai2 * B(2, 0) + ai3 * B(3, 0);
    A(i, 1) = ai0 * B(0, 1) + ai1 * B(1, 1) + ai2 * B(2, 1) + ai3 * B(3, 1);
    A(i, 2) = ai0 * B(0, 2) + ai1 * B(1, 2) + ai2 * B(2, 2) + ai3 * B(3, 2);
    A(i, 3) = ai0 * B(0, 3) + ai1 * B(1, 3) + ai2 * B(2, 3) + ai3 * B(3, 3);
#undef A
#undef B
  }
}

#define RPY_P_FUZZ (0.000001)

int pmMatRpyConvert(double m[], double *r, double *p, double *y)
{
  *p = atan2(-m[2], sqrt(sq(m[0]) + sq(m[1])));

  if (fabs(*p - (2 * M_PI)) < RPY_P_FUZZ)
  {
    *r = atan2(m[4], m[5]);
    *p = (2 * M_PI);
    *y = 0.0;
  }
  else if (fabs(*p + (2 * M_PI)) < RPY_P_FUZZ)
  {
    *r = -atan2(m[6], m[5]);
    *p = -(2 * M_PI);
    *y = 0.0;
  }
  else
  {
    *r = atan2(m[6], m[10]);
    *y = atan2(m[1], m[0]);
  }

  return 0;
}

int kins_fwd(bot_t *bot)
{
  double tr1 = deg2rad(bot->j[0].pos);
  double tr2 = deg2rad(bot->j[1].pos);
  double tr3 = deg2rad(bot->j[2].pos);
  double tr4 = deg2rad(bot->j[3].pos);
  double tr5 = deg2rad(bot->j[4].pos);

  double C234 = cos(tr2 + tr3 + tr4);
  double S234 = sin(tr2 + tr3 + tr4);
  double C23 = cos(tr2 + tr3);
  double S23 = sin(tr2 + tr3);
  double S1 = sin(tr1);
  double C1 = cos(tr1);
  double S2 = sin(tr2);
  double C2 = cos(tr2);
  double S5 = sin(tr5);
  double C5 = cos(tr5);

  double px = C1 * (bot->d5 * S234 + bot->a3 * C23 + bot->a2 * C2);
  double py = S1 * (bot->d5 * S234 + bot->a3 * C23 + bot->a2 * C2);
  double pz = bot->d1 - bot->d5 * C234 + bot->a3 * S23 + bot->a2 * S2;

  bot->t[0] = C1 * C5 * C234 - S1 * S5;
  bot->t[1] = C5 * S234;
  bot->t[2] = -S1 * C5 * C234 - C1 * S5;
  bot->t[3] = 0;

  bot->t[4] = -C1 * S234;
  bot->t[5] = C234;
  bot->t[6] = S1 * S234;
  bot->t[7] = 0;

  bot->t[8] = S1 * C5 + C1 * C234 * S5;
  bot->t[9] = S5 * S234;
  bot->t[10] = C1 * C5 - S1 * C234 * S5;
  bot->t[11] = 0;

  bot->t[12] = px;
  bot->t[13] = pz;
  bot->t[14] = py;

  bot->t[15] = 1;

  return 0;
}

int kins_inv(bot_t *bot)
{

  double nx = -bot->t[0], ny = bot->t[2];
  double ox = bot->t[8], oy = -bot->t[10];
  double ax = -bot->t[4], ay = bot->t[6], az = -bot->t[5];

  double px = bot->t[12];
  double pz = bot->t[13];
  double py = bot->t[14];

  double th1;

  if (py == 0 && px == 0)
  { // point on the Z0 axis
    if (ay == 0 && ax == 0)
    { // wrist pointing straight up/down
      th1 = 0;
    }
    else
    {
      th1 = atan2(ay, ax);
    }
  }
  else
  {
    th1 = atan2(py, px);
  }

  double c1 = cos(th1);
  double s1 = sin(th1);

  double t234 = atan2(c1 * ax + s1 * ay, -az);
  double c234 = cos(t234);
  double s234 = sin(t234);

  // joint 3 - elbow
  double tp1 = c1 * px + s1 * py - bot->d5 * s234;
  double tp2 = pz - bot->d1 + bot->d5 * c234;
  double c3 = (sq(tp1) + sq(tp2) - sq(bot->a3) - sq(bot->a2)) / (2 * bot->a2 * bot->a3);
  double s3 = -sqrt(1 - sq(c3));
  double th3 = atan2(s3, c3);

  // joint 2 - shoulder
  double num = tp2 * (bot->a3 * c3 + bot->a2) - bot->a3 * s3 * tp1;
  double den = tp1 * (bot->a3 * c3 + bot->a2) + bot->a3 * s3 * tp2;
  double th2 = atan2(num, den);

  // joint 4 - pitch
  double th4 = (t234 - th3 - th2);

  // joint 5 - roll
  double th5 = atan2(s1 * nx - c1 * ny, s1 * ox - c1 * oy);

  char msg[5][256];
  double th[] = {th1, th2, th3, th4, th5};

  bot->err = 0;

  for (int i = 0; i < 5; i++)
  {
#ifdef KINS_INV_IGNORE_LIMITS
    if (!isnan(th[i]))
    {
#else
    if (!isnan(th[i]) && th[i] >= deg2rad(bot->j[i].min) && th[i] <= deg2rad(bot->j[i].max))
    {
#endif
      bot->j[i].pos = rad2deg(th[i]);
    }
    else
    {
      bot->err |= (1 << i);
    }

    // pretty print results

    if (isnan(th[i]))
    {
      snprintf(msg[i], sizeof(msg[i]), "%7s ", "nan");
    }
    else if (th[i] < deg2rad(bot->j[i].min))
    {
      snprintf(msg[i], sizeof(msg[i]), "%7.2fv", rad2deg(th[i]));
    }
    else if (th[i] > deg2rad(bot->j[i].max))
    {
      snprintf(msg[i], sizeof(msg[i]), "%7.2f^", rad2deg(th[i]));
    }
    else
    {
      snprintf(msg[i], sizeof(msg[i]), "%7.2f ", rad2deg(th[i]));
    }
  }

  snprintf(bot->msg, sizeof(bot->msg), "kin_inv(%d): %s %s %s %s %s", bot->err, msg[0], msg[1], msg[2], msg[3], msg[4]);

  return bot->err;
}

#ifdef HAVE_SDL
void cross(float th, float l)
{
  glLineWidth(th);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(l, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, l, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, l);
  glEnd();
  glLineWidth(1);
}
#endif

#ifdef HAVE_SDL

unsigned int power_two_floor(unsigned int val)
{
  unsigned int power = 2, nextVal = power * 2;
  while ((nextVal *= 2) <= val)
    power *= 2;
  return power * 2;
}

void text(int x, int y, TTF_Font *font, const char *format, ...)
{
  char buffer[256];
  va_list args;

  if (!font)
  {
    return;
  }

  va_start(args, format);
  vsnprintf(buffer, 255, format, args);

  SDL_Color col = {255, 255, 255};
  SDL_Surface *msg = TTF_RenderText_Blended(font, buffer, col);
  GLuint tex;
  int texture_format;

  glEnable(GL_TEXTURE_2D);
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);

  // Avoid mipmap filtering
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  unsigned int w = power_two_floor(msg->w) * 2;
  unsigned int h = power_two_floor(msg->h) * 2;

  // Create a surface to the correct size in RGB format, and copy the old image
  SDL_Surface *s = SDL_CreateRGBSurface(0, w, h, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000);
  SDL_BlitSurface(msg, NULL, s, NULL);

  int colors = s->format->BytesPerPixel;
  if (colors == 4)
  { // alpha
    if (s->format->Rmask == 0x000000ff)
      texture_format = GL_RGBA;
    else
      texture_format = GL_BGRA;
  }
  else
  { // no alpha
    if (s->format->Rmask == 0x000000ff)
      texture_format = GL_RGB;
    else
      texture_format = GL_BGR;
  }

  glTexImage2D(GL_TEXTURE_2D, 0, colors, w, h, 0, texture_format,
               GL_UNSIGNED_BYTE, s->pixels);

  glBegin(GL_QUADS);
  {
    int z = 1;
    glTexCoord2d(0, 0);
    glVertex3f(x, y, z);
    glTexCoord2d(1, 0);
    glVertex3f(x + s->w, y, z);
    glTexCoord2d(1, 1);
    glVertex3f(x + s->w, y + s->h, z);
    glTexCoord2d(0, 1);
    glVertex3f(x, y + s->h, z);
  }
  glEnd();

  glDisable(GL_TEXTURE_2D);

  glDeleteTextures(1, &tex);
  SDL_FreeSurface(s);
  SDL_FreeSurface(msg);

  va_end(args);
}

void text_matrix(int x, int y, double m[])
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      text(x + 75 * i, y + TTF_FontHeight(sdl_font) * j, sdl_font, "%8.2f", m[4 * i + j]);
    }
  }
}

void cube(GLfloat dSize, int wire)
{
  float size = dSize * 0.5;

#define V(a, b, c) glVertex3d(a size, b size, c size);
#define N(a, b, c) glNormal3d(a, b, c);

  if (wire)
  {
    glBegin(GL_LINE_LOOP);
    N(1.0, 0.0, 0.0);
    V(+, -, +);
    V(+, -, -);
    V(+, +, -);
    V(+, +, +);
    glEnd();
    glBegin(GL_LINE_LOOP);
    N(0.0, 1.0, 0.0);
    V(+, +, +);
    V(+, +, -);
    V(-, +, -);
    V(-, +, +);
    glEnd();
    glBegin(GL_LINE_LOOP);
    N(0.0, 0.0, 1.0);
    V(+, +, +);
    V(-, +, +);
    V(-, -, +);
    V(+, -, +);
    glEnd();
    glBegin(GL_LINE_LOOP);
    N(-1.0, 0.0, 0.0);
    V(-, -, +);
    V(-, +, +);
    V(-, +, -);
    V(-, -, -);
    glEnd();
    glBegin(GL_LINE_LOOP);
    N(0.0, -1.0, 0.0);
    V(-, -, +);
    V(-, -, -);
    V(+, -, -);
    V(+, -, +);
    glEnd();
    glBegin(GL_LINE_LOOP);
    N(0.0, 0.0, -1.0);
    V(-, -, -);
    V(-, +, -);
    V(+, +, -);
    V(+, -, -);
    glEnd();
  }
  else
  {
    glBegin(GL_QUADS);
    N(1.0, 0.0, 0.0);
    V(+, -, +);
    V(+, -, -);
    V(+, +, -);
    V(+, +, +);
    N(0.0, 1.0, 0.0);
    V(+, +, +);
    V(+, +, -);
    V(-, +, -);
    V(-, +, +);
    N(0.0, 0.0, 1.0);
    V(+, +, +);
    V(-, +, +);
    V(-, -, +);
    V(+, -, +);
    N(-1.0, 0.0, 0.0);
    V(-, -, +);
    V(-, +, +);
    V(-, +, -);
    V(-, -, -);
    N(0.0, -1.0, 0.0);
    V(-, -, +);
    V(-, -, -);
    V(+, -, -);
    V(+, -, +);
    N(0.0, 0.0, -1.0);
    V(-, -, -);
    V(-, +, -);
    V(+, +, -);
    V(+, -, -);
    glEnd();
  }
#undef V
#undef N
}

GLUquadricObj *create_quadric(int wire)
{
  GLUquadricObj *qobj = gluNewQuadric();
  if (!wire)
  {
    gluQuadricDrawStyle(qobj, GLU_FILL);
  }
  else
  {
    gluQuadricDrawStyle(qobj, GLU_LINE);
  }
  return qobj;
}

void cylinder(int wire, double BASE, double TOP, double HEIGHT, double SLICES, double STACKS)
{
  GLUquadricObj *QUAD = create_quadric(wire);

  gluCylinder(QUAD, BASE, TOP, HEIGHT, SLICES, STACKS);

  if (!wire)
  {
    glRotatef(180, 1, 0, 0);
    gluDisk(QUAD, 0.0, BASE, SLICES, 1);
    glRotatef(180, 1, 0, 0);
    glTranslatef(0.0, 0.0, HEIGHT);
    gluDisk(QUAD, 0.0, TOP, SLICES, 1);
  }

  gluDeleteQuadric(QUAD);
}

void draw_bot(int wire, const bot_t *bot)
{
  glPushMatrix(); // 1

  glTranslatef(0.0, bot->d1, 0.0);

  cross(4, 1);

  if (!wire)
  {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  else
  {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  }

  // link1 - base

  glColor3f(0.25, 0.25, 0.25);

  glPushMatrix(); // 2
  glTranslatef(-0.5, -1.875, 0);
  glScalef(2, 0.75, 2);
  cube(1.0, wire);
  glPopMatrix(); // 1

  glPushMatrix(); // 2
  glTranslatef(0, -1.75, 0);
  glScalef(1 - 0.05, 1.0 - 0.05, 1.25 - 0.05);
  cube(1.0, wire);
  glPopMatrix(); // 1

  glPushMatrix(); // 2
  glTranslatef(0.0, -1.175 - 0.65 / 2, 0);
  glRotatef(-90, 1, 0, 0);
  glScalef(1.0, 1.0, 1.0);
  glColor3f(0.5, 0.5, 0.5);

  cylinder(wire, 0.35, 0.35, 0.65, 16, 1);

  glPopMatrix(); // 1

  // link2 - body

  glColor3f(1.0, 0.44, 0.176);

  glPushMatrix(); // 2
  glTranslatef(0, 0.0, 0.0);
  glRotatef(bot->j[0].pos, 0.0, 1.0, 0.0);
  glTranslatef(0, 0.0, 0.0);

  glPushMatrix(); // 3
  glTranslatef(-0.75, -0.45, 0.0);
  glScalef(2.5, 0.9, 1.2);
  cube(1.001, wire);
  glPopMatrix(); // 2

  glPushMatrix(); // 3
  glRotatef(14, 0.0, 0.0, 1.0);
  glTranslatef(-0.975, 0.05, 0.0);
  glScalef(1.9, 0.9, 1.2);
  cube(0.999, wire);
  glPopMatrix(); // 2

  glPushMatrix(); // 3
  glTranslatef(0.0, 0, -1.2 / 2);
  glRotatef(-90, 0.0, 0.0, 1.0);
  glScalef(1.0, 1.0, 1.0);
  cylinder(wire, 0.5, 0.5, 1.2, 16, 1);
  glPopMatrix(); // 2

  if (!wire)
  {
    glPushMatrix(); // 3
    glTranslatef(0.0, 0, -1.4 / 2);
    glRotatef(-90, 0.0, 0.0, 1.0);
    glScalef(1.0, 1.0, 1.0);
    cylinder(wire, 0.15, 0.15, 1.4, 16, 1);
    glPopMatrix(); // 2
  }

  glColor3f(1.0, 0.44, 0.176);

  glPushMatrix(); // 3
  glTranslatef(0.0, 0, -bot->d5 / 2);
  glRotatef(-90, 0.0, 0.0, 1.0);
  glScalef(1.0, 1.0, 1.0);
  cylinder(wire, 0.35, 0.35, bot->d5, 16, 1);
  glPopMatrix(); // 2

  // link3 - upperarm

  glColor3f(1.0, 0.44, 0.176);

  glRotatef(bot->j[1].pos, 0.0, 0.0, 1.0);

  glPushMatrix(); // 3
  glTranslatef(1.1, 0.0, 0.0);
  glScalef(bot->a2, 1.0, 1.0);
  cube(1.0, wire);
  glPopMatrix(); // 2

  if (!wire)
  {
    glPushMatrix(); // 3
    glTranslatef(0, 0, -0.5);
    glRotatef(-90, 0.0, 0.0, 1.0);
    glScalef(1.0, 1.0, 1.0);
    cylinder(wire, 0.5, 0.5, 1, 16, 1);
    glPopMatrix(); // 2

    glPushMatrix(); // 3
    glTranslatef(bot->a2, 0, -0.5);
    glRotatef(-90, 0.0, 0.0, 1.0);
    glScalef(1.0, 1.0, 1.0);
    cylinder(wire, 0.5, 0.5, 1, 16, 1);
    glPopMatrix(); // 2

    glPushMatrix(); // 3
    glTranslatef(bot->a2, 0, -1.1 / 2);
    glRotatef(-90, 0.0, 0.0, 1.0);
    glScalef(1.0, 1.0, 1.0);
    cylinder(wire, 0.25, 0.25, 1.1, 16, 1);
    glPopMatrix(); // 2

    glPushMatrix(); // 3
    glTranslatef(bot->a2, 0, -1.2 / 2);
    glRotatef(-90, 0.0, 0.0, 1.0);
    glScalef(1.0, 1.0, 1.0);
    cylinder(wire, 0.15, 0.15, 1.2, 16, 1);
    glPopMatrix(); // 2
  }

  // link4 - forearm

  glTranslatef(bot->a2, 0, 0);
  glRotatef(bot->j[2].pos, 0.0, 0.0, 1.0);

  glPushMatrix(); // 3
  glTranslatef(0.6, 0.0, 0.0);
  glScalef(1.2, 0.8, 0.9);
  cube(1.0, wire);
  glPopMatrix(); // 2

  glPushMatrix(); // 3
  glTranslatef(0, 0, -0.45);
  glRotatef(-90, 0.0, 0.0, 1.0);
  glScalef(1.0, 1.0, 1.0);
  cylinder(wire, 0.4, 0.4, 0.9, 16, 1);
  glPopMatrix(); // 2

  glPushMatrix(); // 3
  glTranslatef(1.2, 0, -0.45);
  glRotatef(-90, 0.0, 0.0, 1.0);
  glScalef(1.0, 1.0, 1.0);
  cylinder(wire, 0.4, 0.4, 0.9, 16, 1);
  glPopMatrix(); // 2

  // link5 - wrist pitch

  glColor3f(0.25, 0.25, 0.25);

  glTranslatef(bot->a3, 0, 0);
  glRotatef(bot->j[3].pos, 0.0, 0.0, 1.0);

  glPushMatrix(); // 3
  glTranslatef(0, 0, -0.6);
  glRotatef(-90, 0.0, 0.0, 1.0);
  cylinder(wire, 0.3, 0.3, 1.2, 22, 1);
  glPopMatrix(); // 2

  glColor3f(0.5, 0.5, 0.5);

  // link6 - hand

  glColor3f(0.25, 0.25, 0.25);

  glRotatef(bot->j[4].pos, 0.0, 1.0, 0.0);

  glPushMatrix(); // 3
  glRotatef(90, 1.0, 0.0, 0.0);
  cylinder(wire, 0.13, 0.13, 0.5, 22, 1);
  glPopMatrix(); // 2

  glColor3f(0.15, 0.15, 0.15);

  glPushMatrix(); // 3
  glTranslatef(0.0, -0.7, 0.0);
  glScalef(0.5, 0.4, 0.4);
  cube(1.0, wire);
  glPopMatrix(); // 2

  glPushMatrix(); // 3
  glTranslatef(0.0, -1.02, 0.0);
  glScalef(0.8, 0.25, 0.4);
  cube(1.0, wire);
  glPopMatrix(); // 2

  glTranslatef(0, -bot->d5, 0);

  double grip = bot->grip ? 0.25 : 0.05;

  glPushMatrix(); // 3
  glTranslatef(grip, 0.0, 0.0);
  glScalef(0.1, 0.4, 0.25);
  cube(1.0, wire);
  glPopMatrix(); // 2

  glPushMatrix(); // 3
  glTranslatef(-grip, 0.0, 0.0);
  glScalef(0.1, 0.4, 0.25);
  cube(1.0, wire);
  glPopMatrix(); // 2

  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  cross(4, 0.5);

  glPopMatrix(); // 1

  glPopMatrix(); // 0
}

void scene(const bot_t *bot_fwd, const bot_t *bot_inv)
{
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);

  // table

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glColor3f(0.001, 0.001, 0.001);

  for (int x = -50; x <= 50; x += 5)
  {
    glPushMatrix();
    glTranslatef(-0.5 + x * .1, 0, 0);
    glScalef(.05, .01, 10.0);
    cube(1.0, 0);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.5, 0, x * .1);
    glScalef(10.0, .01, .05);
    cube(1.0, 0);
    glPopMatrix();
  }

  draw_bot(0, bot_fwd);
  draw_bot(1, bot_inv);
}

void draw_hud(bot_t *bot)
{
  glMatrixMode(GL_MODELVIEW);

  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);

  glPushMatrix();
  glLoadIdentity();
  glOrtho(0.0, width, height, 0.0, -1.0, 10.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glClear(GL_DEPTH_BUFFER_BIT);
  glColor3f(1, 0.2f, 0.2f);

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

  glColor3ub(25, 200, 25);

  text(15, 10, sdl_font, "AXIS   POS     VEL");

  text(15, 10 + 1 * TTF_FontHeight(sdl_font), sdl_font, "X: %8.2f %8.3f", bot->t[12], bot->cart[0].vel);
  text(15, 10 + 2 * TTF_FontHeight(sdl_font), sdl_font, "Y: %8.2f %8.3f", bot->t[13], bot->cart[1].vel);
  text(15, 10 + 3 * TTF_FontHeight(sdl_font), sdl_font, "Z: %8.2f %8.3f", bot->t[14], bot->cart[2].vel);

  double r, p, y;
  pmMatRpyConvert(bot->t, &r, &p, &y);

  text(15, 10 + 5 * TTF_FontHeight(sdl_font), sdl_font, "R: %8.2f %8.3f", rad2deg(r), bot->cart[5].vel);
  text(15, 10 + 6 * TTF_FontHeight(sdl_font), sdl_font, "P: %8.2f %8.3f", rad2deg(p), bot->cart[3].vel);
  text(15, 10 + 7 * TTF_FontHeight(sdl_font), sdl_font, "Y: %8.2f %8.3f", rad2deg(y), bot->cart[4].vel);

  text(15, 10 + 9 * TTF_FontHeight(sdl_font), sdl_font, "GRIP: %s", bot->grip ? "Open" : "Closed");

  text(15, 10 + 11 * TTF_FontHeight(sdl_font), sdl_font, "JOINT ANGLE    VEL");
  /*
  int i;
  for (i = 0; i < 5; i++) {
      text(15, 10+(i+1)*TTF_FontHeight(sdl_font), sdl_font,
           "%d: %8.2f %8.3f", i+1, bot->j[i].pos, bot->j[i].vel);
  }*/

  text(15, 10 + 12 * TTF_FontHeight(sdl_font), sdl_font, "%d: %8.2f %8.3f", 1, bot->j[0].pos, bot->j[0].vel);
  text(15, 10 + 13 * TTF_FontHeight(sdl_font), sdl_font, "%d: %8.2f %8.3f", 2, bot->j[1].pos, bot->j[1].vel);
  text(15, 10 + 14 * TTF_FontHeight(sdl_font), sdl_font, "%d: %8.2f %8.3f", 3, bot->j[2].pos, bot->j[2].vel);
  text(15, 10 + 15 * TTF_FontHeight(sdl_font), sdl_font, "%d: %8.2f %8.3f", 4, bot->j[3].pos - 90.0, bot->j[3].vel);
  text(15, 10 + 16 * TTF_FontHeight(sdl_font), sdl_font, "%d: %8.2f %8.3f", 5, bot->j[4].pos, bot->j[4].vel);

  text(width - 370, 10, sdl_font, "TOOL");
  text_matrix(width - 320, 10, bot->t);

  if (strlen(bot->msg))
  {
    text(15, height - TTF_FontHeight(sdl_font) * 1.5, sdl_font, bot->msg);
  }

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  glMatrixMode(GL_MODELVIEW);
}

void display(const bot_t *bot_fwd, bot_t *bot_inv)
{
  SDL_GetWindowSize(sdl_window, &width, &height);

  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();

  if (view_mode == 0)
  {
    glViewport(0, 0, width, height);
  }
  else
  {
    glViewport(0, 0, width / 2, height / 2);
  }

  glPushMatrix();
  glTranslatef(0, 0, -10);
  glRotatef(15, 1, 0, 0); // glRotatef(rotate_x, 0, 1, 0);
  glTranslatef(0, -2, 0);

  scene(bot_fwd, bot_inv);

  glPopMatrix();

  glMatrixMode(GL_PROJECTION);

  if (view_mode != 0)
  {
    glPushMatrix();

    glLoadIdentity();
#define DIM 4.5
    if (height > width)
    {
      glOrtho(-DIM, DIM, -DIM * (height / (width * 1.0)), DIM * (height / (width * 1.0)), -DIM * 2, DIM * 2);
    }
    else
    {
      glOrtho(-DIM * (width / (height * 1.0)), DIM * (width / (height * 1.0)), -DIM, DIM, -DIM * 2, DIM * 2);
    }
#undef DIM
    glMatrixMode(GL_MODELVIEW);

    glViewport(0, height / 2 + 1, width / 2 + 1, height / 2);
    glPushMatrix();
    glTranslatef(0, -3, 0);
    glRotatef(90, 0, -1, 0);

    scene(bot_fwd, bot_inv);

    glPopMatrix();

    glViewport(width / 2 + 1, height / 2 + 1, width / 2, height / 2);
    glPushMatrix();
    glTranslatef(0, -3, 0);
    scene(bot_fwd, bot_inv);
    glPopMatrix();

    glViewport(width / 2 + 1, 0, width / 2, height / 2);
    glPushMatrix();
    // glTranslatef( -2,0,0);
    glRotatef(90, 1, 0, 0);
    scene(bot_fwd, bot_inv);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
  }

  glPopMatrix();

  draw_hud(bot_inv);

  glEnd();

  SDL_GL_SwapWindow(sdl_window);
}
#endif

int do_kins_fwd = 1;
int do_kins_inv = 0;

void jog_joint(bot_t *bot, int i, double amount)
{
  double tmp = bot->j[i].pos + amount;

  if (amount > 0)
  {
    bot->j[i].pos = fmin(tmp, bot->j[i].max);
    do_kins_fwd = 1;
  }
  else
  {
    bot->j[i].pos = fmax(tmp, bot->j[i].min);
    do_kins_fwd = 1;
  }
}

void move_tool(bot_t *bot, int i, double amount)
{
  bot->t[12 + i] += amount;
  do_kins_inv = 1;
}

void rotate_tool(bot_t *bot, double a, double x, double y, double z)
{
  rotate_m_axyz(bot->t, a, x, y, z);
  do_kins_inv = 1;
}

#ifdef HAVE_SPACENAV

typedef struct
{
  int fd;
  int pos[6];
  int key[2];
} spacenav_t;

#define test_bit(bit, array) (array[bit / 8] & (1 << (bit % 8)))

int spacenav_open(void)
{
  char fname[20];
  struct input_id id;

  int i = 0;
  while (i < 64)
  {
    snprintf(fname, sizeof(fname), "/dev/input/event%d", i++);
    int fd = open(fname, O_RDWR | O_NONBLOCK);
    if (fd > 0)
    {
      ioctl(fd, EVIOCGID, &id);

      if (id.vendor == 0x046d && (id.product == 0xc626 || id.product == 0xc623 || id.product == 0xc603))
      {
        // fprintf(stderr, "Using device: %s\n", fname);
        return fd;
      }

      close(fd);
    }
  }

  return 0;
}

void spacenav_close(spacenav_t *s)
{
  if (s->fd)
  {
    close(s->fd);
  }
}

void spacenav_read(spacenav_t *s)
{
  struct input_event ev;

  int n;
  while ((n = read(s->fd, &ev, sizeof(struct input_event))) > 0)
  {
    // fprintf(stderr, "spacenav_read %d bytes.\n", n);

    if (n >= sizeof(struct input_event))
    {
      switch (ev.type)
      {
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

int update_model(bot_t *bot_fwd, bot_t *bot_inv, int do_kins_fwd, int do_kins_inv)
{
  int err = 0;
  if (do_kins_inv)
  {
    err = kins_inv(bot_inv);
    if (bot_inv->err == 0)
    {
      kins_fwd(bot_inv);
      memcpy(bot_fwd, bot_inv, sizeof(bot_t));
    }
    else
    {
      memcpy(bot_inv, bot_fwd, sizeof(bot_t));
    }
  }
  else if (do_kins_fwd)
  {
    kins_fwd(bot_fwd);
    memcpy(bot_inv, bot_fwd, sizeof(bot_t));
  }

  return err;
}

#ifdef HAVE_SDL
#include "icons/rm501.cdata"
void set_window_icon(SDL_Window *sdl_window)
{
  SDL_RWops *z = SDL_RWFromMem(rm501_png, sizeof(rm501_png));
  SDL_Surface *sdl_window_icon_surface = IMG_Load_RW(z, 1);
  if (sdl_window_icon_surface == NULL)
  {
    fprintf(stderr, "Error: unable to load icons/rm501.png: %s.\n", SDL_GetError());
    exit(EXIT_FAILURE);
  }
  SDL_SetWindowIcon(sdl_window, sdl_window_icon_surface);
  SDL_FreeSurface(sdl_window_icon_surface);
}

#ifdef HAVE_PNG
void screenshot(int x, int y, const char *filename)
{
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

SDL_Rect toggle_fake_fullscreen(SDL_Rect old_bounds)
{
  if (SDL_GetWindowFlags(sdl_window) & SDL_WINDOW_BORDERLESS)
  {
    SDL_SetWindowBordered(sdl_window, SDL_TRUE);
    SDL_SetWindowSize(sdl_window, old_bounds.w, old_bounds.h);
    SDL_SetWindowPosition(sdl_window, old_bounds.x, old_bounds.y);
    return old_bounds;
  }
  else
  {
    SDL_Rect cur_bounds;
    SDL_GetWindowPosition(sdl_window, &cur_bounds.x, &cur_bounds.y);
    SDL_GetWindowSize(sdl_window, &cur_bounds.w, &cur_bounds.h);

    int idx = SDL_GetWindowDisplayIndex(sdl_window);
    SDL_Rect bounds;
    SDL_GetDisplayBounds(idx, &bounds);
    SDL_SetWindowBordered(sdl_window, SDL_FALSE);
    // SDL_SetWindowPosition(sdl_window, bounds.x, bounds.y);
    SDL_SetWindowSize(sdl_window, bounds.w, bounds.h);

    return cur_bounds;
  }
}
#endif

void handle_signal(int signal)
{
#ifndef __MINGW32__
  sigset_t pending;
  // Find out which signal we're handling
  switch (signal)
  {
  case SIGHUP:
  case SIGUSR1:
    break;
  case SIGINT:
    done = 1;
  default:
    return;
  }

  sigpending(&pending);
  if (sigismember(&pending, SIGHUP))
  {
  }
  if (sigismember(&pending, SIGUSR1))
  {
  }
#endif
}

#ifdef HAVE_MQTT

#define RM501_SCALAR 150.0

coord_t bot2coord(bot_t *bot)
{
  // convert bot to coord
  double r = 0, p = 0, y = 0;
  pmMatRpyConvert(bot->t, &y, &r, &p);
  coord_t coord = {
      bot->t[14] * RM501_SCALAR,
      bot->t[12] * RM501_SCALAR,
      bot->t[13] * RM501_SCALAR,
      rad2deg(p) + 90.0,
      rad2deg(r), // bot_inv.j[4].pos
      bot->grip};
  return coord;
}

void coord2bot(bot_t *bot, coord_t coord)
{
  // convert coord to bot
  bot_t bot_aux = *bot;
  bot_aux.t[14] = coord.x / RM501_SCALAR;
  bot_aux.t[12] = coord.y / RM501_SCALAR;
  bot_aux.t[13] = coord.z / RM501_SCALAR;

  for (int i = 0; i < 10; i++)
  {
    double r = 0, p = 0, y = 0;
    pmMatRpyConvert(bot_aux.t, &y, &r, &p);
    p = rad2deg(p) - 90.0;
    r = rad2deg(r);
    // pitch
    rotate_m_axyz(bot_aux.t, coord.pitch - p, sin(deg2rad(bot_aux.j[0].pos)), 0, cos(deg2rad(bot_aux.j[0].pos)));
    // roll
    rotate_m_axyz(bot_aux.t, coord.roll - r, 0, 1, 0);

    kins_inv(&bot_aux);
    // bot_aux.j[4].pos=r;
    // kins_fwd(&bot_aux);
  }

  bot_aux.grip = coord.grip;
  // return results
  *bot = bot_aux;
}
#endif

int main(int argc, char **argv)
{

#ifndef __MINGW32__
  struct sigaction sa;
  sa.sa_handler = &handle_signal;
  sa.sa_flags = SA_RESTART;
  sigfillset(&sa.sa_mask);

  if (sigaction(SIGHUP, &sa, NULL) == -1)
  {
    perror("Error: cannot handle SIGHUP");
  }

  if (sigaction(SIGUSR1, &sa, NULL) == -1)
  {
    perror("Error: cannot handle SIGUSR1");
  }

  if (sigaction(SIGINT, &sa, NULL) == -1)
  {
    perror("Error: cannot handle SIGINT");
  }
#endif

#ifdef ENABLE_FPS_LIMIT
  unsigned int ft = 0, frames;
#endif

  bot_t bot_fwd, bot_inv;

#ifdef HAVE_SDL
  int do_sdl = 0;
  bool do_fullscreen = false;

  SDL_Event ev;
  const Uint8 *keys = SDL_GetKeyboardState(0);

  int sdlk_tab_pressed = 0;

  int sdl_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL;
  SDL_DisplayMode sdl_displaymode;

  SDL_Rect cur_bounds;
#endif

#ifdef HAVE_NCURSES
  int do_curses = 0;
#endif

  int do_help = 0;
  int do_version = 0;
  int verbose = 0;
  int rw_mode = 0;

  int i = 0;
  while (++i < argc)
  {
#define OPTION_SET(longopt, shortopt) (strcmp(argv[i], longopt) == 0 || strcmp(argv[i], shortopt) == 0)
#define OPTION_VALUE ((i + 1 < argc) ? (argv[i + 1]) : (NULL))
#define OPTION_VALUE_PROCESSED (i++)
    if (OPTION_SET("--help", "-h"))
    {
      do_help = 1;
    }
    else if (OPTION_SET("--version", "-v"))
    {
      do_version = 1;
#ifdef HAVE_SDL
    }
    else if (OPTION_SET("--fullscreen", "-f"))
    {
      do_fullscreen = true;
    }
    else if (OPTION_SET("--sdl", "-s"))
    {
      do_sdl = 1;
#endif
#ifdef HAVE_AUDIO
    }
    else if (OPTION_SET("--audio", "-a"))
    {
      do_audio = 1;
#endif
#ifdef HAVE_HAL
    }
    else if (OPTION_SET("--hal", "-l"))
    {
      do_hal = 1;
#endif
#ifdef HAVE_TRAJGEN
    }
    else if (OPTION_SET("--trajgen-test", "-t"))
    {
      do_trajgen_test = 1;
#endif
#ifdef HAVE_NCURSES
    }
    else if (OPTION_SET("--curses", "-c"))
    {
      do_curses = 1;
#endif
#ifdef HAVE_ZMQ
    }
    else if (OPTION_SET("--zmq", "-z"))
    {
      do_zmq = 1;
#endif
#ifdef HAVE_MQTT
    }
    else if (OPTION_SET("--mqtt", "-m"))
    {
      do_mqtt = 1;
    }
    else if (OPTION_SET("--rw", "-w"))
    {
      do_mqtt = 1;
      rw_mode = 1;
#endif
    }
    else if (OPTION_SET("--verbose", "-x"))
    {
      verbose++;
    }
    else
    {
      fprintf(stderr, "Unknown option: %s\n", argv[i]);
      do_help = 1;
    }
  }

  if (do_version)
  {
    fprintf(stdout, "%s %s\n", argv[0], PROGRAM_VERSION);
    return EXIT_SUCCESS;
  }

  if (do_help)
  {
    fprintf(stdout, "Usage: %s [OPTIONS]\n\n"
                    " Where [OPTIONS] are zero or more of the following:\n\n"
#ifdef HAVE_SDL
                    "    [-s|--sdl]               SDL window mode\n"
                    "    [-f|--fullscreen]        SDL window fullscreen mode\n"
#endif
#ifdef HAVE_AUDIO
                    "    [-a|--audio]             Motor motion audio\n"
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
#ifdef HAVE_ZMQ
                    "    [-z|--zmq]               ZMQ server mode\n"
#endif
#ifdef HAVE_MQTT
                    "    [-m|--mqtt]              MQTT client mode\n"
                    "    [-w|--rw]                MQTT write mode\n"
#endif
                    "    [-x|--verbose]           Show verbose information\n\n"
                    "    [-h|--help]              Show help information\n\n"
                    "    [-v|--version]           Show version number\n\n"
                    "Report bugs to Jakob Flierl <jakob.flierl@gmail.com>\n"
                    "Website and manual: https://github.com/koppi/rm501\n"
                    "\n",
            argv[0]);
    return EXIT_SUCCESS;
  }

#ifdef HAVE_HAL
  if (do_hal)
  {
    // initialize component
    hal_comp_id = hal_init(modname);
    if (hal_comp_id < 1)
    {
      fprintf(stderr, "%s: ERROR: hal_init failed\n", modname);
      goto fail0;
    }

    // allocate hal memory
    hal_pos_data = hal_malloc(sizeof(hal_t));
    if (hal_pos_data == NULL)
    {
      fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", modname);
      goto fail1;
    }

    // register pins
    for (i = 0; i < 5; i++)
    {
      if (hal_pin_float_newf(HAL_OUT, &(hal_pos_data->axis[i]), hal_comp_id, "%s.%d.pos-cmd", MODULE_NAME, i) != 0)
      {
        fprintf(stderr, "%s: ERROR: unable to register hal pin %s.%d.pos-cmd\n", modname, MODULE_NAME, i);
        goto fail1;
      }
    }
  }
#endif

#ifdef HAVE_ZMQ
  if (do_zmq && verbose >= 1)
  {
    int major, minor, patch;
    zmq_version(&major, &minor, &patch);
    fprintf(stderr, "Using 0MQ version %d.%d.%d\n", major, minor, patch);
  }
#endif

#ifdef HAVE_MQTT
  if (do_mqtt)
  {
    mqtt_handler_init();
  }
#endif

#ifdef HAVE_NCURSES
  if (do_curses)
  {
    initscr();
    nonl();
    cbreak();
    noecho();
    keypad(stdscr, 1);
    timeout(0); // getch non-blocking
    if (has_colors())
    {
      start_color();
      init_pair(1, COLOR_WHITE, COLOR_BLUE);
      init_pair(2, COLOR_BLUE, COLOR_WHITE);
      init_pair(3, COLOR_RED, COLOR_WHITE);
    }
    curs_set(0);

    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);

    menubar = subwin(stdscr, 1, w.ws_col, 0, 0);
    messagebar = subwin(stdscr, 1, w.ws_col - 1, w.ws_row - 1, 1);
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

#ifdef HAVE_SPACENAV
  spacenav_t sn = {0};

  sn.fd = spacenav_open();
#endif

#ifdef HAVE_SDL
  if (do_sdl)
  {

    if (verbose >= 1)
    {
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
    if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    {
      fprintf(stderr, "Unable to initialise SDL: %s\n", SDL_GetError());
      exit(EXIT_FAILURE);
    }

#ifdef HAVE_AUDIO
    if (do_audio)
    {
      open_audio();
    }
#endif

#ifdef HAVE_JOYSTICK
    if (SDL_NumJoysticks() < 1)
    {
      // printf( "Warning: No joysticks connected!\n" );
    }
    else
    {
      joy = SDL_JoystickOpen(0);
      if (joy == NULL)
      {
        printf("Warning: Unable to open game controller! SDL Error: %s\n", SDL_GetError());
      }
    }
#endif

    if (TTF_Init() < 0)
    {
      fprintf(stderr, "Unable to initialise SDL_ttf.\n");
      exit(EXIT_FAILURE);
    }

    char *sdl_font_file = NULL;
#if defined(__HAIKU__)
    sdl_font_file = "/Haiku/system/data/fonts/ttfonts/DejaVuSansMono.ttf";
#elif defined(__FreeBSD__)
    sdl_font_file = "/usr/local/share/fonts/dejavu/DejaVuSansMono.ttf";
//#elif defined(linux) // Alpine Linux
//    sdl_font_file = "/usr/share/fonts/ttf-dejavu/DejaVuSansMono.ttf";
#elif defined(__linux__)
    sdl_font_file = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf";
#elif defined(__MINGW32__)
    sdl_font_file = "doc/DejaVuSansMono.ttf";
#else
#warning "Please set your font path in the source code."
#endif

    sdl_font = TTF_OpenFont(sdl_font_file, 15);

    if (!sdl_font)
    {
      fprintf(stderr, "%s: %s\n", SDL_GetError(), sdl_font_file);
      exit(EXIT_FAILURE);
    }

    if (SDL_GetCurrentDisplayMode(0, &sdl_displaymode) != 0)
    {
      fprintf(stderr, "Could not get display mode for video display #%d: %s", 0, SDL_GetError());
      exit(EXIT_FAILURE);
    }

    /*
      if (sdl_flags & SDL_WINDOW_FULLSCREEN) {
    width  = sdl_displaymode.w;
  height = sdl_displaymode.h;
}*/

    SDL_SetHint(SDL_HINT_RENDER_DRIVER, "opengl");
    //SDL_SetHint(SDL_HINT_RENDER_DRIVER, "opengles2");

    sdl_window = SDL_CreateWindow("Mitsubishi RM-501 Movemaster II Robot Simulator",
                                  SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                  width, height, sdl_flags);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    sdl_glcontext = SDL_GL_CreateContext(sdl_window);

    sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_ACCELERATED);

    set_window_icon(sdl_window);

    SDL_RenderClear(sdl_renderer);

    if (do_fullscreen)
    {
      cur_bounds = toggle_fake_fullscreen(cur_bounds);
    }

    //SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    //glEnable(GL_DEBUG_OUTPUT);

    {
      GLfloat pos[4] = {3., 5., 2., 1.};
      GLfloat white[4] = {0.75, 0.75, 0.75, 1.};
      GLfloat black[4] = {0., 0., 0., 0.};

      glDisable(GL_LIGHTING);
      glEnable(GL_LIGHT1);
      glLightfv(GL_LIGHT1, GL_POSITION, pos);
      glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
      glLightfv(GL_LIGHT1, GL_SPECULAR, black);

      glEnable(GL_COLOR_MATERIAL);
      glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
      glMaterialfv(GL_FRONT, GL_SPECULAR, black);
    }

    glEnable(GL_TEXTURE_2D);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

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
  if (do_sdl)
  {
#ifdef ENABLE_FPS_LIMIT
    frames = 0;
    ft = SDL_GetTicks();
#endif
  }
#endif

#ifdef HAVE_HAL
  if (do_hal)
  {
    hal_ready(hal_comp_id);
  }
#endif

#ifdef HAVE_TRAJGEN
  tg_init(1000, 25000, 1000.0, 1, 3);
  tg_echk(trajgen_switch_state(TRAJ_STATE_COORDINATED));
  while (!tg_tg.is_done)
    tg_echk(trajgen_tick());
  tg_speed = 100;
  tg_accel = 5000;
  tg_blending = 15;
  tg_line(1.6, 3.2, 0.0);
  while (!tg_tg.is_done)
    tg_echk(trajgen_tick());
#endif

  while (!done)
  {
#ifdef HAVE_MQTT
    coord_t coord = bot2coord(&bot_inv);
    bot_t bot_aux = bot_inv;

    if (mqtt_periodic_callback(&coord, rw_mode))
    {
      // an update arrived from mqtt

      int try = 100;
      do
      {
        // try to convert multiple times until result is good
        coord2bot(&bot_aux, coord);

        bot_fwd = bot_aux;
        bot_inv = bot_aux;
        update_model(&bot_fwd, &bot_inv, 1, 1);
        bot_aux = bot_inv;
      } while (!coord_equal(coord, bot2coord(&bot_aux), EPSILON) && --try > 0);
    }
#endif

    static double old_pos[5];

    for (i = 0; i < 5; i++)
    {
      old_pos[i] = bot_fwd.j[i].pos;
    }

    static double old_pos_cart_fwd[6];
    old_pos_cart_fwd[0] = bot_fwd.t[12];
    old_pos_cart_fwd[1] = bot_fwd.t[13];
    old_pos_cart_fwd[2] = bot_fwd.t[14];
    double y, r, p;
    pmMatRpyConvert(bot_fwd.t, &y, &r, &p);
    old_pos_cart_fwd[3] = r;
    old_pos_cart_fwd[4] = p;
    old_pos_cart_fwd[5] = y;

    static double old_pos_cart_inv[6];
    old_pos_cart_inv[0] = bot_inv.t[12];
    old_pos_cart_inv[1] = bot_inv.t[13];
    old_pos_cart_inv[2] = bot_inv.t[14];
    pmMatRpyConvert(bot_inv.t, &y, &r, &p);
    old_pos_cart_inv[3] = r;
    old_pos_cart_inv[4] = p;
    old_pos_cart_inv[5] = y;

#ifdef HAVE_SDL
    if (do_sdl)
    {
      SDL_PollEvent(&ev);

      if (ev.type == SDL_QUIT ||
          (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE))
        done = 1;

      if (ev.type == SDL_KEYUP && ev.key.keysym.sym == SDLK_F11)
      {
        cur_bounds = toggle_fake_fullscreen(cur_bounds);
      }

      if (ev.type == SDL_KEYUP && ev.key.keysym.sym == SDLK_TAB)
      {
        sdlk_tab_pressed = 0;
      }

      double d = 0.05;
      double cnt = 1.00;

      if (keys[SDL_SCANCODE_LCTRL] || keys[SDL_SCANCODE_RCTRL])
      {
        d *= 2;
        cnt *= 2;
      }

      if (keys[SDL_SCANCODE_Q])
      {
        jog_joint(&bot_fwd, 0, cnt);
      }
      if (keys[SDL_SCANCODE_W])
      {
        jog_joint(&bot_fwd, 1, cnt);
      }
      if (keys[SDL_SCANCODE_E])
      {
        jog_joint(&bot_fwd, 2, cnt);
      }
      if (keys[SDL_SCANCODE_R])
      {
        jog_joint(&bot_fwd, 3, cnt);
      }
      if (keys[SDL_SCANCODE_T])
      {
        jog_joint(&bot_fwd, 4, cnt);
      }

      if (keys[SDL_SCANCODE_A])
      {
        jog_joint(&bot_fwd, 0, -cnt);
      }
      if (keys[SDL_SCANCODE_S])
      {
        jog_joint(&bot_fwd, 1, -cnt);
      }
      if (keys[SDL_SCANCODE_D])
      {
        jog_joint(&bot_fwd, 2, -cnt);
      }
      if (keys[SDL_SCANCODE_F])
      {
        jog_joint(&bot_fwd, 3, -cnt);
      }
      if (keys[SDL_SCANCODE_G])
      {
        jog_joint(&bot_fwd, 4, -cnt);
      }

      if (keys[SDL_SCANCODE_O])
      {
        bot_fwd.grip = true;
        bot_inv.grip = true;
      }
      if (keys[SDL_SCANCODE_L])
      {
        bot_fwd.grip = false;
        bot_inv.grip = false;
      }

      if (!keys[SDL_SCANCODE_LSHIFT] && !keys[SDL_SCANCODE_RSHIFT])
      {
        if (keys[SDL_SCANCODE_LEFT])
        {
          move_tool(&bot_inv, 0, -d);
        }
        if (keys[SDL_SCANCODE_RIGHT])
        {
          move_tool(&bot_inv, 0, d);
        }
        if (keys[SDL_SCANCODE_I])
        {
          move_tool(&bot_inv, 1, d);
        }
        if (keys[SDL_SCANCODE_K])
        {
          move_tool(&bot_inv, 1, -d);
        }
        if (keys[SDL_SCANCODE_UP])
        {
          move_tool(&bot_inv, 2, d);
        }
        if (keys[SDL_SCANCODE_DOWN])
        {
          move_tool(&bot_inv, 2, -d);
        }
      }
      else
      {
        if (keys[SDL_SCANCODE_UP])
        {
          rotate_tool(&bot_inv, -cnt, sin(deg2rad(bot_inv.j[0].pos)), 0, cos(deg2rad(bot_inv.j[0].pos)));
        }
        if (keys[SDL_SCANCODE_DOWN])
        {
          rotate_tool(&bot_inv, cnt, sin(deg2rad(bot_inv.j[0].pos)), 0, cos(deg2rad(bot_inv.j[0].pos)));
        }
        if (keys[SDL_SCANCODE_LEFT])
        {
          rotate_tool(&bot_inv, -cnt, 0, 1, 0);
        }
        if (keys[SDL_SCANCODE_RIGHT])
        {
          rotate_tool(&bot_inv, cnt, 0, 1, 0);
        }
      }

#ifdef PROJ2
      ///////////////////////////////////////////////////////////////////////////////
      if (keys[SDL_SCANCODE_H])
      {
        bot_fwd.j[0].tar = 30;
        bot_fwd.j[1].tar = 0;
        bot_fwd.j[2].tar = 0;
        bot_fwd.j[3].tar = 90;
        bot_fwd.j[4].tar = 0;
      }

      if (keys[SDL_SCANCODE_N])
      {
        bot_fwd.j[0].tar = -120;
        bot_fwd.j[1].tar = 100;
        bot_fwd.j[2].tar = -90;
        bot_fwd.j[3].tar = 0;
        bot_fwd.j[4].tar = 0;
      }
#ifdef PROJ3
      // C: move in a circle
      if (keys[SDL_SCANCODE_C])
      {
        bot_inv.proj3counter %= 360;
        bot_inv.proj3counter++;
        double theta = deg2rad(bot_inv.proj3counter);
        bot_inv.t[12] = 2.8 + cos(theta) * 0.6;
        bot_inv.t[13] = 2.4;
        bot_inv.t[14] = sin(theta) * 0.6;

        do_kins_inv = 1;
      }

      // V: reset circle counter
      if (keys[SDL_SCANCODE_V])
      {
        bot_inv.proj3counter = 0;
      }

#endif
      // H/N: move to home positions (try with the shift key to see the difference)
      if (keys[SDL_SCANCODE_H] || keys[SDL_SCANCODE_N])
      {
        i = 1;
        while (1)
        {
          if (bot_fwd.j[i].tar > bot_fwd.j[i].pos)
          {
            if (bot_fwd.j[i].tar - bot_fwd.j[i].pos < cnt)
            {
              bot_fwd.j[i].pos = bot_fwd.j[i].tar;
            }
            else
            {
              jog_joint(&bot_fwd, i, cnt);
              if (!keys[SDL_SCANCODE_LSHIFT] && !keys[SDL_SCANCODE_RSHIFT])
              {
                break;
              }
            }
          }
          if (bot_fwd.j[i].tar < bot_fwd.j[i].pos)
          {
            if (bot_fwd.j[i].tar - bot_fwd.j[i].pos > -cnt)
            {
              bot_fwd.j[i].pos = bot_fwd.j[i].tar;
            }
            else
            {
              jog_joint(&bot_fwd, i, -cnt);
              if (!keys[SDL_SCANCODE_LSHIFT] && !keys[SDL_SCANCODE_RSHIFT])
              {
                break;
              }
            }
          }
          if (i >= 1 && i <= 3)
            i++;
          else if (i == 4)
            i = 0;
          else if (i == 0)
            break;
        }
      }
///////////////////////////////////////////////////////////////////////////////
#endif

      if (ev.type == SDL_KEYDOWN)
      {

        switch (ev.key.keysym.sym)
        {
        case SDLK_TAB:
          if (!sdlk_tab_pressed)
          {
            if (view_mode == 0)
            {
              view_mode = 1;
            }
            else
            {
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
      if (ev.type == SDL_JOYAXISMOTION)
      {
#define JOY_SCALE (1.0 / (32768.0 * 2.0))
        if (ev.jaxis.which == 0)
        {
          // X-axis motion
          if (ev.jaxis.axis == 0)
          {
            // Left of dead zone
            if (ev.jaxis.value < -JOYSTICK_DEAD_ZONE)
            {
              move_tool(&bot_inv, 0, (double)(d * ev.jaxis.value * JOY_SCALE));
            }
            // Right of dead zone
            else if (ev.jaxis.value > JOYSTICK_DEAD_ZONE)
            {
              move_tool(&bot_inv, 0, (double)(d * ev.jaxis.value * JOY_SCALE));
            }
          } // Y-axis motion
          else if (ev.jaxis.axis == 1)
          {
            // Left of dead zone
            if (ev.jaxis.value < -JOYSTICK_DEAD_ZONE || ev.jaxis.value > JOYSTICK_DEAD_ZONE)
            {
              move_tool(&bot_inv, 2, (double)(-d * ev.jaxis.value * JOY_SCALE));
            }
          } // Z-axis motion
          else if (ev.jaxis.axis == 3)
          {
            // Left of dead zone
            if (ev.jaxis.value < -JOYSTICK_DEAD_ZONE || ev.jaxis.value > JOYSTICK_DEAD_ZONE)
            {
              move_tool(&bot_inv, 1, (double)(-d * ev.jaxis.value * JOY_SCALE));
            }
          }
        }
      }
#endif
    }
#endif

#ifdef HAVE_SPACENAV
    if (sn.fd)
    {
      spacenav_read(&sn);

#define JOG_MIN 15
#define JOG_SPEED 0.0001
#define JOG_SPEED_ROT 0.008
      if (abs(sn.pos[0]) > JOG_MIN)
      {
        move_tool(&bot_inv, 0, sn.pos[0] * JOG_SPEED);
      }

      if (abs(sn.pos[1]) > JOG_MIN)
      {
        move_tool(&bot_inv, 2, -sn.pos[1] * JOG_SPEED);
      }

      if (abs(sn.pos[2]) > JOG_MIN)
      {
        move_tool(&bot_inv, 1, -sn.pos[2] * JOG_SPEED);
      }

      if (abs(sn.pos[5]) > JOG_MIN)
      {
        rotate_tool(&bot_inv, -sn.pos[5] * JOG_SPEED_ROT, 0, 1, 0);
      }

#ifdef SPACENAV_JOG_A
      if (abs(sn.pos[4]) > JOG_MIN)
      {
        rotate_tool(&bot_inv, -sn.pos[4] * JOG_SPEED_ROT,
                    sin(deg2rad(bot_inv.j[0].pos)), 0, cos(deg2rad(bot_inv.j[0].pos)));
      }
#endif
      if (sn.key[0])
      {
        done = 1;
      }
    }
#endif

#ifdef HAVE_TRAJGEN
    if (do_trajgen_test)
    {
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

      if (trajgen_num_queued() < 2)
      {
        tg_line(2.8 + randpos(0.5), 1.0 + randpos(1), randpos(2));
      }
    }
#endif

    update_model(&bot_fwd, &bot_inv, do_kins_fwd, do_kins_inv);

    if (do_kins_inv || do_kins_fwd)
    {
      memcpy(&bot_inv, &bot_fwd, sizeof(bot_t));
      kins_inv(&bot_inv);
      kins_fwd(&bot_inv);
      kins_fwd(&bot_fwd);
    }

    do_kins_fwd = 0;
    do_kins_inv = 0;

    // update joint velocity values
    for (i = 0; i < 5; i++)
    {
      bot_fwd.j[i].vel = bot_fwd.j[i].pos - old_pos[i];
      bot_inv.j[i].vel = bot_inv.j[i].pos - old_pos[i];
    }
    // update cartesian velocity values
    bot_fwd.cart[0].vel = bot_fwd.t[12] - old_pos_cart_fwd[0];
    bot_fwd.cart[1].vel = bot_fwd.t[13] - old_pos_cart_fwd[1];
    bot_fwd.cart[2].vel = bot_fwd.t[14] - old_pos_cart_fwd[2];
    pmMatRpyConvert(bot_fwd.t, &y, &r, &p);
    bot_fwd.cart[3].vel = r - old_pos_cart_fwd[3];
    bot_fwd.cart[4].vel = p - old_pos_cart_fwd[4];
    bot_fwd.cart[5].vel = y - old_pos_cart_fwd[5];

    bot_inv.cart[0].vel = bot_inv.t[12] - old_pos_cart_inv[0];
    bot_inv.cart[1].vel = bot_inv.t[13] - old_pos_cart_inv[1];
    bot_inv.cart[2].vel = bot_inv.t[14] - old_pos_cart_inv[2];
    pmMatRpyConvert(bot_inv.t, &y, &r, &p);
    bot_inv.cart[3].vel = r - old_pos_cart_inv[3];
    bot_inv.cart[4].vel = p - old_pos_cart_inv[4];
    bot_inv.cart[5].vel = y - old_pos_cart_inv[5];

#ifdef HAVE_SDL
    if (do_sdl)
    {
      display(&bot_fwd, &bot_inv);
      SDL_RenderPresent(sdl_renderer);
      // screenshot(0, 0, "screenshot.png");
#ifdef HAVE_AUDIO
      if (do_audio)
      {
        for (i = 0; i < 5; i++)
        {
          MIDDLE_C[i] = fabs(bot_fwd.j[i].vel * 261.626);
        }
      }
#endif // HAVE_AUDIO
    }
#endif // HAVE_SDL

#ifdef HAVE_NCURSES
    if (do_curses)
    {
      int key = getch();

      if (key == 'q')
      {
        done = 1;
      }

      move(8, 1);
      printw("x: %7.2f", bot_inv.t[12]);
      move(9, 1);
      printw("y: %7.2f", bot_inv.t[13]);
      move(10, 1);
      printw("z: %7.2f", bot_inv.t[14]);

      double r, p, y;

      pmMatRpyConvert(bot_inv.t, &r, &p, &y);

      move(12, 1);
      printw("a: %7.2f", rad2deg(r));
      move(13, 1);
      printw("b: %7.2f", rad2deg(p));
      move(14, 1);
      printw("c: %7.2f", rad2deg(y));

      int i;
      for (i = 0; i < 5; i++)
      {
        move(18, 1 + i * 11);
        printw("%7.2f", bot_inv.j[i].pos);
      }

      move(20, 1);
      if (strlen(bot_inv.msg))
      {
        printw("%s", bot_inv.msg);
      }
      else
      {
        printw("                             ");
      }

      touchwin(stdscr);
      refresh();
    }
#endif

#ifdef HAVE_HAL
    if (do_hal)
    {
      for (i = 0; i < 5; i++)
      {
        *(hal_pos_data->axis[i]) = bot_fwd.j[i].pos;
      }
    }
#endif

#ifdef HAVE_SDL
    if (do_sdl)
    {
#ifdef ENABLE_FPS_LIMIT
      while (frames * 1000.0 / ((float)(SDL_GetTicks() - ft + 1)) > (float)(DEFAULT_FPS))
      {
        SDL_Delay(10);
      }
      frames++;
#endif
    }
#endif
  } // while ! done

#ifdef HAVE_SDL
  if (do_sdl)
  {
    SDL_RenderClear(sdl_renderer);

    if (sdl_font)
    {
      TTF_CloseFont(sdl_font);
    }

#ifdef HAVE_JOYSTICK
    if (joy)
    {
      SDL_JoystickClose(joy);
      joy = NULL;
    }
#endif

    TTF_Quit();

    SDL_DestroyRenderer(sdl_renderer);
    SDL_GL_DeleteContext(sdl_glcontext);
    SDL_DestroyWindow(sdl_window);

#ifdef HAVE_AUDIO
    if (do_audio)
    {
      close_audio();
    }
#endif

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
  if (do_hal)
  {
    hal_exit(hal_comp_id);
  }
fail0:
#endif

#ifdef HAVE_MQTT
  if (do_mqtt)
  {
    mqtt_handler_close();
  }
#endif

  return 0;
}
