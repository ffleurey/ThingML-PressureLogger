#include <stdint.h>
#include <Arduino.h>
/*****************************************************************************
 * Headers for type : ArduinoSensor
 *****************************************************************************/


// BEGIN: Code from the c_header annotation ArduinoSensor
#include <Dps310.h>
// END: Code from the c_header annotation ArduinoSensor

// Definition of the instance struct:
struct ArduinoSensor_Instance {

// Instances of different sessions
bool active;
// Variables for the ID of the ports of the instance
uint16_t id_gateway;
uint16_t id_clock;
// Variables for the current instance state
int ArduinoSensor_Main_State;
// Variables for the properties of the instance

};
// Declaration of prototypes outgoing messages :
void ArduinoSensor_Main_OnEntry(int state, struct ArduinoSensor_Instance *_instance);
void ArduinoSensor_handle_clock_sample_clock(struct ArduinoSensor_Instance *_instance);
// Declaration of callbacks for incoming messages:
void register_ArduinoSensor_send_gateway_pressure_listener(void (*_listener)(struct ArduinoSensor_Instance *, long, long));
void register_external_ArduinoSensor_send_gateway_pressure_listener(void (*_listener)(struct ArduinoSensor_Instance *, long, long));

// Definition of the states:
#define ARDUINOSENSOR_MAIN_SAMPLING_STATE 0
#define ARDUINOSENSOR_MAIN_STATE 1


/*****************************************************/
//                    Serial
/*****************************************************/

struct Serial_instance_type {
    uint16_t listener_id;
    /*INSTANCE_INFORMATION*/
};

extern struct Serial_instance_type Serial_instance;

void Serial_setup();

void Serial_set_listener_id(uint16_t id);

void Serial_forwardMessage(byte * msg, uint8_t size);

void Serial_read();

/********************* FORWARDERS *********************/

/*FORWARDERS*/// Forwarding of messages Serial::ArduinoSensor::gateway::pressure
void forward_Serial_ArduinoSensor_send_gateway_pressure(struct ArduinoSensor_Instance *_instance, long a, long b);

struct timer2_instance_type {
    uint16_t listener_id;
    /*INSTANCE_INFORMATION*/
};
extern struct timer2_instance_type timer2_instance;

void timer2_setup();
void timer2_read();

//void forward_timer2_SoftButton_send_Timer_timer_start(struct SoftButton_Instance *_instance, uint8_t id, uint32_t time);
//void forward_timer2_SoftButton_send_Timer_timer_cancel(struct SoftButton_Instance *_instance, uint8_t id);

/* Adds and instance to the runtime and returns its id */
uint16_t add_instance(void * instance_struct);
/* Returns the instance with id */
void * instance_by_id(uint16_t id);

/* Returns the number of byte currently in the fifo */
int fifo_byte_length();
/* Returns the number of bytes currently available in the fifo */
int fifo_byte_available();
/* Returns true if the fifo is empty */
int fifo_empty();
/* Return true if the fifo is full */
int fifo_full();
/* Enqueue 1 byte in the fifo if there is space
   returns 1 for sucess and 0 if the fifo was full */
int fifo_enqueue(byte b);
/* Enqueue 1 byte in the fifo without checking for available space
   The caller should have checked that there is enough space */
int _fifo_enqueue(byte b);
/* Dequeue 1 byte in the fifo.
   The caller should check that the fifo is not empty */
byte fifo_dequeue();

#define MAX_INSTANCES 4
#define FIFO_SIZE 256

/*********************************
 * Instance IDs and lookup
 *********************************/

void * instances[MAX_INSTANCES];
uint16_t instances_count = 0;

void * instance_by_id(uint16_t id) {
  return instances[id];
}

uint16_t add_instance(void * instance_struct) {
  instances[instances_count] = instance_struct;
  return instances_count++;
}

/******************************************
 * Simple byte FIFO implementation
 ******************************************/

byte fifo[FIFO_SIZE];
int fifo_head = 0;
int fifo_tail = 0;

// Returns the number of byte currently in the fifo
int fifo_byte_length() {
  if (fifo_tail >= fifo_head)
    return fifo_tail - fifo_head;
  return fifo_tail + FIFO_SIZE - fifo_head;
}

// Returns the number of bytes currently available in the fifo
int fifo_byte_available() {
  return FIFO_SIZE - 1 - fifo_byte_length();
}

// Returns true if the fifo is empty
int fifo_empty() {
  return fifo_head == fifo_tail;
}

// Return true if the fifo is full
int fifo_full() {
  return fifo_head == ((fifo_tail + 1) % FIFO_SIZE);
}

// Enqueue 1 byte in the fifo if there is space
// returns 1 for sucess and 0 if the fifo was full
int fifo_enqueue(byte b) {
  int new_tail = (fifo_tail + 1) % FIFO_SIZE;
  if (new_tail == fifo_head) return 0; // the fifo is full
  fifo[fifo_tail] = b;
  fifo_tail = new_tail;
  return 1;
}

// Enqueue 1 byte in the fifo without checking for available space
// The caller should have checked that there is enough space
int _fifo_enqueue(byte b) {
  fifo[fifo_tail] = b;
  fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
  return 0; // Dummy added by steffend
}

// Dequeue 1 byte in the fifo.
// The caller should check that the fifo is not empty
byte fifo_dequeue() {
  if (!fifo_empty()) {
    byte result = fifo[fifo_head];
    fifo_head = (fifo_head + 1) % FIFO_SIZE;
    return result;
  }
  return 0;
}

/*****************************************************************************
 * Implementation for type : ArduinoSensor
 *****************************************************************************/


// BEGIN: Code from the c_global annotation ArduinoSensor

Dps310 ifxDps310_1 = Dps310();
Dps310 ifxDps310_2 = Dps310();

// END: Code from the c_global annotation ArduinoSensor

// Declaration of prototypes:
//Prototypes: State Machine
void ArduinoSensor_Main_OnExit(int state, struct ArduinoSensor_Instance *_instance);
//Prototypes: Message Sending
void ArduinoSensor_send_gateway_pressure(struct ArduinoSensor_Instance *_instance, long a, long b);
//Prototypes: Function
void f_ArduinoSensor_initialize_sensors(struct ArduinoSensor_Instance *_instance);
void f_ArduinoSensor_sample_pressure(struct ArduinoSensor_Instance *_instance);
// Declaration of functions:
// Definition of function initialize_sensors
void f_ArduinoSensor_initialize_sensors(struct ArduinoSensor_Instance *_instance) {
ifxDps310_1.begin(Wire, 0x76);
  		 ifxDps310_2.begin(Wire, 0x77);
  		 ifxDps310_1.correctTemp();
  		 ifxDps310_2.correctTemp();
}
// Definition of function sample_pressure
void f_ArduinoSensor_sample_pressure(struct ArduinoSensor_Instance *_instance) {
;long p1;
;long p2;
int32_t temperature;
		  ifxDps310_1.measureTempOnce(temperature, 3);
ifxDps310_2.measureTempOnce(temperature, 3);
ifxDps310_1.measurePressureOnce(p1, 3);
ifxDps310_2.measurePressureOnce(p2, 3);
ArduinoSensor_send_gateway_pressure(_instance, p1, p2);
}

// Sessions functionss:


// On Entry Actions:
void ArduinoSensor_Main_OnEntry(int state, struct ArduinoSensor_Instance *_instance) {
switch(state) {
case ARDUINOSENSOR_MAIN_STATE:{
_instance->ArduinoSensor_Main_State = ARDUINOSENSOR_MAIN_SAMPLING_STATE;
f_ArduinoSensor_initialize_sensors(_instance);
ArduinoSensor_Main_OnEntry(_instance->ArduinoSensor_Main_State, _instance);
break;
}
case ARDUINOSENSOR_MAIN_SAMPLING_STATE:{
break;
}
default: break;
}
}

// On Exit Actions:
void ArduinoSensor_Main_OnExit(int state, struct ArduinoSensor_Instance *_instance) {
switch(state) {
case ARDUINOSENSOR_MAIN_STATE:{
ArduinoSensor_Main_OnExit(_instance->ArduinoSensor_Main_State, _instance);
break;}
case ARDUINOSENSOR_MAIN_SAMPLING_STATE:{
break;}
default: break;
}
}

// Event Handlers for incoming messages:
void ArduinoSensor_handle_clock_sample_clock(struct ArduinoSensor_Instance *_instance) {
if(!(_instance->active)) return;
//Region Main
uint8_t ArduinoSensor_Main_State_event_consumed = 0;
if (_instance->ArduinoSensor_Main_State == ARDUINOSENSOR_MAIN_SAMPLING_STATE) {
if (ArduinoSensor_Main_State_event_consumed == 0 && 1) {
f_ArduinoSensor_sample_pressure(_instance);
ArduinoSensor_Main_State_event_consumed = 1;
}
}
//End Region Main
//End dsregion Main
//Session list: 
}

// Observers for outgoing messages:
void (*external_ArduinoSensor_send_gateway_pressure_listener)(struct ArduinoSensor_Instance *, long, long)= 0x0;
void (*ArduinoSensor_send_gateway_pressure_listener)(struct ArduinoSensor_Instance *, long, long)= 0x0;
void register_external_ArduinoSensor_send_gateway_pressure_listener(void (*_listener)(struct ArduinoSensor_Instance *, long, long)){
external_ArduinoSensor_send_gateway_pressure_listener = _listener;
}
void register_ArduinoSensor_send_gateway_pressure_listener(void (*_listener)(struct ArduinoSensor_Instance *, long, long)){
ArduinoSensor_send_gateway_pressure_listener = _listener;
}
void ArduinoSensor_send_gateway_pressure(struct ArduinoSensor_Instance *_instance, long a, long b){
if (ArduinoSensor_send_gateway_pressure_listener != 0x0) ArduinoSensor_send_gateway_pressure_listener(_instance, a, b);
if (external_ArduinoSensor_send_gateway_pressure_listener != 0x0) external_ArduinoSensor_send_gateway_pressure_listener(_instance, a, b);
;
}



#define timer2_NB_SOFT_TIMER 4
uint32_t timer2_timer[timer2_NB_SOFT_TIMER];
uint32_t  timer2_prev_1sec = 0;

uint8_t timer2_tic_flags = 0;

void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id);

uint8_t timer2_interrupt_counter = 0;
SIGNAL(TIMER2_OVF_vect) {
TCNT2 = 5;
timer2_interrupt_counter++;
if((timer2_interrupt_counter % 100) == 0) {
timer2_tic_flags |= 0b00000001;
}
if(timer2_interrupt_counter >= 100) {
timer2_interrupt_counter = 0;
}
}



//struct timer2_instance_type {
//    uint16_t listener_id;
//    /*INSTANCE_INFORMATION*/
//} timer2_instance;

struct timer2_instance_type timer2_instance;


void timer2_setup() {
	// Run timer2 interrupt up counting at 250kHz 
		 TCCR2A = 0;
		 TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;
		
		 //Timer2 Overflow Interrupt Enable
		 TIMSK2 |= 1<<TOIE2;


	timer2_prev_1sec = millis() + 1000;
}

void timer2_set_listener_id(uint16_t id) {
	timer2_instance.listener_id = id;
}

void timer2_100ms_tic() {
{
uint8_t enqueue_buf[2];
enqueue_buf[0] = (1 >> 8) & 0xFF;
enqueue_buf[1] = 1 & 0xFF;
externalMessageEnqueue(enqueue_buf, 2, timer2_instance.listener_id);
}
}





void timer2_read() {
    uint32_t tms = millis();
    
    if (timer2_prev_1sec < tms) {
        timer2_prev_1sec += 1000;
    }
    if((timer2_tic_flags & 0b00000001) >> 0) {
timer2_100ms_tic();
timer2_tic_flags &= 0b11111110;
}

}
/*****************************************************/
//                    Serial
/*****************************************************/

#define Serial_BAUDRATE 115200
#define Serial_MAX_LOOP 0
#define Serial_MAX_MSG_SIZE 0
/*OTHER_VARS*/


#define Serial_LISTENER_STATE_IDLE 0
#define Serial_LISTENER_STATE_READING 1
#define Serial_LISTENER_STATE_ESCAPE 2
#define Serial_LISTENER_STATE_ERROR 3


#define Serial_START_BYTE 18
#define Serial_STOP_BYTE 19
#define Serial_ESCAPE_BYTE 125

struct Serial_instance_type Serial_instance;

void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id);

void Serial_setup() {
  Serial.begin(Serial_BAUDRATE);
}

void Serial_set_listener_id(uint16_t id) {
  Serial_instance.listener_id = id;
}

void Serial_forwardMessage(byte * msg, uint8_t size) {
  Serial.write(Serial_START_BYTE);
  for(uint8_t i = 0; i < size; i++) {
	if(msg[i] == Serial_ESCAPE_BYTE || msg[i] == Serial_START_BYTE || msg[i] == Serial_STOP_BYTE) {
    	Serial.write(Serial_ESCAPE_BYTE);
	}
    Serial.write(msg[i]);
  }
  Serial.write(Serial_STOP_BYTE);
}

void Serial_parser(byte * msg, uint16_t size) {
}


uint8_t Serial_serialListenerState = 0;
uint8_t Serial_msg_buf[Serial_MAX_MSG_SIZE];
uint16_t Serial_msg_index = 0;
uint8_t Serial_incoming = 0;

void Serial_read() {
  byte limit = 0;
  while ((Serial.available()) && (limit < Serial_MAX_LOOP)) {
   limit++;
    Serial_incoming = Serial.read();
    
    switch(Serial_serialListenerState) {
      case Serial_LISTENER_STATE_IDLE:
        if(Serial_incoming == Serial_START_BYTE) {
          Serial_serialListenerState = Serial_LISTENER_STATE_READING;
          Serial_msg_index = 0;
        }
      break;
      
      case Serial_LISTENER_STATE_READING:
        if (Serial_msg_index > Serial_MAX_MSG_SIZE) {
          Serial_serialListenerState = Serial_LISTENER_STATE_ERROR;
        } else {
          if(Serial_incoming == Serial_STOP_BYTE) {
            Serial_serialListenerState = Serial_LISTENER_STATE_IDLE;
            
            
            Serial_parser(Serial_msg_buf, Serial_msg_index);
            
          } else if (Serial_incoming == Serial_ESCAPE_BYTE) {
            Serial_serialListenerState = Serial_LISTENER_STATE_ESCAPE;
          } else {
            Serial_msg_buf[Serial_msg_index] = Serial_incoming;
            Serial_msg_index++;
          }
        }
      break;
      
      case Serial_LISTENER_STATE_ESCAPE:
        if (Serial_msg_index >= Serial_MAX_MSG_SIZE) {
          Serial_serialListenerState = Serial_LISTENER_STATE_ERROR;
        } else {
          Serial_msg_buf[Serial_msg_index] = Serial_incoming;
          Serial_msg_index++;
          Serial_serialListenerState = Serial_LISTENER_STATE_READING;
        }
      break;
      
      case Serial_LISTENER_STATE_ERROR:
        Serial_serialListenerState = Serial_LISTENER_STATE_IDLE;
        Serial_msg_index = 0;
      break;
    }
  }
  
}

/*FORWARDERS*/// Forwarding of messages Serial::ArduinoSensor::gateway::pressure
void forward_Serial_ArduinoSensor_send_gateway_pressure(struct ArduinoSensor_Instance *_instance, long a, long b){
byte forward_buf[10];
forward_buf[0] = (20 >> 8) & 0xFF;
forward_buf[1] =  20 & 0xFF;


// parameter a
union u_a_t {
long p;
byte bytebuffer[4];
} u_a;
u_a.p = a;
forward_buf[2] =  (u_a.bytebuffer[3] & 0xFF);
forward_buf[3] =  (u_a.bytebuffer[2] & 0xFF);
forward_buf[4] =  (u_a.bytebuffer[1] & 0xFF);
forward_buf[5] =  (u_a.bytebuffer[0] & 0xFF);

// parameter b
union u_b_t {
long p;
byte bytebuffer[4];
} u_b;
u_b.p = b;
forward_buf[6] =  (u_b.bytebuffer[3] & 0xFF);
forward_buf[7] =  (u_b.bytebuffer[2] & 0xFF);
forward_buf[8] =  (u_b.bytebuffer[1] & 0xFF);
forward_buf[9] =  (u_b.bytebuffer[0] & 0xFF);

//Forwarding with specified function 
Serial_forwardMessage(forward_buf, 10);
}




/*****************************************************************************
 * Definitions for configuration : ArduinoSensor
 *****************************************************************************/

//Declaration of instance variables
//Instance sensor
// Variables for the properties of the instance
struct ArduinoSensor_Instance sensor_var;
// Variables for the sessions of the instance




//New dispatcher for messages
void dispatch_sample_clock(uint16_t sender) {
if (sender == timer2_instance.listener_id) {
ArduinoSensor_handle_clock_sample_clock(&sensor_var);

}

}


int processMessageQueue() {
if (fifo_empty()) return 0; // return 0 if there is nothing to do

uint8_t mbufi = 0;

// Read the code of the next port/message in the queue
uint16_t code = fifo_dequeue() << 8;

code += fifo_dequeue();

// Switch to call the appropriate handler
switch(code) {
case 1:{
byte mbuf[4 - 2];
while (mbufi < (4 - 2)) mbuf[mbufi++] = fifo_dequeue();
uint8_t mbufi_sample_clock = 2;
dispatch_sample_clock((mbuf[0] << 8) + mbuf[1] /* instance port*/);
break;
}
}
return 1;
}

void forward_ArduinoSensor_send_gateway_pressure(struct ArduinoSensor_Instance *_instance, long a, long b){
if(_instance->id_gateway == sensor_var.id_gateway) {
forward_Serial_ArduinoSensor_send_gateway_pressure(_instance, a, b);
}
}

//external Message enqueue
void externalMessageEnqueue(uint8_t * msg, uint8_t msgSize, uint16_t listener_id) {
if ((msgSize >= 2) && (msg != NULL)) {
uint8_t msgSizeOK = 0;
switch(msg[0] * 256 + msg[1]) {
case 1:
if(msgSize == 2) {
msgSizeOK = 1;}
break;
}

if(msgSizeOK == 1) {
if ( fifo_byte_available() > (msgSize + 2) ) {
	uint8_t i;
	for (i = 0; i < 2; i++) {
		_fifo_enqueue(msg[i]);
	}
	_fifo_enqueue((listener_id >> 8) & 0xFF);
	_fifo_enqueue(listener_id & 0xFF);
	for (i = 2; i < msgSize; i++) {
		_fifo_enqueue(msg[i]);
	}
}
}
}
}

void initialize_configuration_ArduinoSensor() {
// Initialize connectors
register_external_ArduinoSensor_send_gateway_pressure_listener(&forward_ArduinoSensor_send_gateway_pressure);

// Init the ID, state variables and properties for external connector timer2
// Init the ID, state variables and properties for external connector Serial

// Network Initialization

Serial_instance.listener_id = add_instance(&Serial_instance);

Serial_setup();


timer2_instance.listener_id = add_instance(&timer2_instance);

timer2_setup();

// End Network Initialization

// Init the ID, state variables and properties for instance sensor
sensor_var.active = true;
sensor_var.id_gateway = add_instance( (void*) &sensor_var);
sensor_var.id_clock = add_instance( (void*) &sensor_var);
sensor_var.ArduinoSensor_Main_State = ARDUINOSENSOR_MAIN_SAMPLING_STATE;

ArduinoSensor_Main_OnEntry(ARDUINOSENSOR_MAIN_STATE, &sensor_var);
}




void setup() {
initialize_configuration_ArduinoSensor();

}

void loop() {

// Network Listener
Serial_read();

timer2_read();
// End Network Listener

int emptyEventConsumed = 1;
while (emptyEventConsumed != 0) {
emptyEventConsumed = 0;
}

    processMessageQueue();
}
