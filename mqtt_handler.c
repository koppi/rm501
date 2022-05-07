#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>

#include <MQTTClient.h>

#include "mqtt_handler.h"

MQTTClient client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
MQTTClient_message pubmsg = MQTTClient_message_initializer;
MQTTClient_deliveryToken token;
int rc;

char incoming_message[64] = "";
char incoming_flag = 0;

pthread_mutex_t incoming_mutex;

// forward declaration
void publish_cordinate(coord_t);
int subscribe_callback(void* , char*, int, MQTTClient_message*);

void print_error(coord_t c1, coord_t c2){
    fprintf(stderr, "\nerror: %.1f, %.1f, %.1f, %.1f, %.1f", \
              c1.x-c2.x, c1.y-c2.y, c1.z-c2.z,\
              c1.pitch-c2.pitch, c1.roll-c2.roll);
}

/**
 * @brief verifies if two coordinates are equal
 * @param c1 first coordinate
 * @param c2 second coordinate
 * @param epsilon allowable error
 * @retval result (true or false)
 */
bool coord_equal(coord_t c1, coord_t c2, float epsilon) {
    if ( fabs( c1.x-c2.x ) < epsilon &&
        fabs( c1.y-c2.y ) < epsilon &&
        fabs( c1.z-c2.z ) < epsilon &&
        fabs( c1.pitch-c2.pitch ) < epsilon &&
        fabs( c1.roll-c2.roll ) < epsilon &&
        c1.grip==c2.grip
    ) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Handle lost connection
 */
void connection_lost_callback() {
    fprintf(stderr, "\nConnectin lost\n");
    while(1) {
        sleep(5);
        fprintf(stderr, "\nTrying to reconnect\n");
        if (MQTTClient_connect(client, &conn_opts) == MQTTCLIENT_SUCCESS) {
            fprintf(stderr, "\nConnection re-established\n");
            MQTTClient_subscribe(client, TOPIC, QOS);
            break;
        }
    }
}

/**
 * @brief Initialize the handler
 */
void mqtt_handler_init() {

    pthread_mutex_init(&incoming_mutex, NULL);

    char clientID[10];
    //generate unique ID
    snprintf(clientID, sizeof(clientID), "sim%ld", time(0)%100000);

    MQTTClient_create(&client, ADDRESS, clientID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
  
    // MQTT Connection parameters
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    rc = MQTTClient_setCallbacks(client, NULL, connection_lost_callback, subscribe_callback, NULL);

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "Error: failed to connect to '%s', return code %d\n", ADDRESS, rc);
        exit(-1);
    }

    if (rc != MQTTCLIENT_SUCCESS) {
		MQTTClient_destroy(&client);
		return;
	}

    rc = MQTTClient_subscribe(client, TOPIC, QOS);
}

/**
 * @brief Close the handler
 */
int mqtt_handler_close() {

    // Disconnect
    rc = MQTTClient_unsubscribe(client, TOPIC);

    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    
    return rc;
}

/**
 * @brief this function must be called on the infinite loop
 * @param coord pointer to actual coordintes
 * @param allowPub flag to allow publishing
 * @retval flag if an update was made
 */
int mqtt_periodic_callback(coord_t* coord, bool allowPub) {
    static int run_flag = 0;
    static coord_t last_coord;
    static time_t last_time = 0;
    time_t now = clock();
    int ret_flag = 0;

    //check if min delta t has passed
    if ( ((double)(now - last_time) / (double)CLOCKS_PER_SEC) >= (double)MIN_DELTA_T \
            && allowPub) {
    
        last_time = now;
        
        //if this is the first time
        if (run_flag == 0) { 
            run_flag = 1;
            last_coord = *coord; //this will prevent publish
        }

        //check if simulator has moved
        if ( !coord_equal(last_coord,*coord, EPSILON) ) {
            last_coord = *coord; //publish movement
            publish_cordinate(last_coord);
        }
    }

    //check subscription
    //take mutex
    pthread_mutex_lock(&incoming_mutex);

    if (incoming_flag) { //if a message arrived
        incoming_flag = 0;

        coord_t aux;
        char grip = 0;
        double time;
        
        //decode the message
        sscanf(incoming_message, "%lf %f %f %f %f %f %c",
               &time, &aux.x, &aux.y, &aux.z, &aux.pitch, &aux.roll, &grip);
        aux.grip = (grip=='C') ? 0 : 1;

        //only apply if movement is significant
        if (!coord_equal(last_coord, aux, EPSILON) ){
            printf("\nApply    : '%s'\n", incoming_message);
            *coord = aux; //apply coordinates
            last_coord = aux; //remember coordinates
            ret_flag = 1; //notify update         

        }
    }

    //release mutex
    pthread_mutex_unlock(&incoming_mutex);

    return ret_flag;
}

void publish_cordinate(coord_t c) {
    char message[64];
    struct timeval tv;
    gettimeofday(&tv, NULL);

    snprintf(message, sizeof(message),
            "%lf %+.1f %+.1f %+.1f %+.1f %.1f %c",
             tv.tv_sec + tv.tv_usec * 0.000001,
             c.x, c.y, c.z, c.pitch, c.roll, c.grip?'O':'C');

    fprintf(stderr, "\npublish:  '%s'\n", message);

    // Publish message
    pubmsg.payload = message;
    pubmsg.payloadlen = strlen(message);
    pubmsg.qos = QOS;
    pubmsg.retained = 1;
    MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
}

int subscribe_callback(void* context, char* topicName, int topicLen, MQTTClient_message* m) {
    if (m->payloadlen >= sizeof(incoming_message))
        return 1; // reject if message is too big
    
    // take mutex
    pthread_mutex_lock(&incoming_mutex);

	strncpy(incoming_message, m->payload, m->payloadlen);
    incoming_message[m->payloadlen] = '\0'; // terminate string
    incoming_flag = 1;

    fprintf(stderr, "\nsubscribe: '%s'\n", incoming_message);

    // release mutex
    pthread_mutex_unlock(&incoming_mutex);

	MQTTClient_free(topicName);
	MQTTClient_freeMessage(&m);
	return 1;
}
