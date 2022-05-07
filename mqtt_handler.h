#ifndef _MQTT_HANDLER_H_
#define _MQTT_HANDLER_H_

#define ADDRESS     "tcp://localhost:1883"
#define TOPIC       "Robot/Coordinates"
#define QOS         1
#define TIMEOUT     10000L

#define MIN_DELTA_T 0.005 // seconds
#define EPSILON 1       // min error

typedef struct {
    float x, y, z, pitch, roll;
    bool grip;
} coord_t;

void mqtt_handler_init();
int mqtt_handler_close();
int  mqtt_periodic_callback(coord_t*, bool);
bool coord_equal(coord_t, coord_t, float);

void print_error(coord_t, coord_t);

#endif //_MQTT_HANDLER_H_

