/*
 * new_remote_receiver.h
 * Based on NewRemoteSwitch library v1.0.0 (20121229) made by Randy Simons http://randysimons.nl/
 * Created: 22.01.2016 10:58:21
 *  Author: magne.normann
 */ 

/**
* See RemoteSwitch for introduction.
*
* NewRemoteReceiver decodes the signal received from a 433MHz-receiver, like the "KlikAanKlikUit"-system
* as well as the signal sent by the RemoteSwtich class. When a correct signal is received,
* a user-defined callback function is called.
*
* Note that in the callback function, the interrupts are still disabled. You can enabled them, if needed.
* A call to the callback must be finished before NewRemoteReceiver will call the callback function again, thus
* there is no re-entrant problem.
*
* When sending your own code using NewRemoteSwich, disable() the receiver first.
*/

#ifndef NEW_REMOTE_RECEIVER_H_
#define NEW_REMOTE_RECEIVER_H_

#define MIN_REPEATS	0

struct tc_module tc_instance;
volatile signed short	receiver_state;			// State of decoding process. There are 49 states, 1 for "waiting for signal" and 48 for decoding the 48 edges in a valid code.
static bool				receiver_in_callback;	// When true, the callback function is being executed; prevents re-entrance.

struct NewRemoteCode {
	unsigned int	period;		// Detected duration in microseconds of 1T in the received signal
	unsigned long	address;	// Address of received code. [0..2^26-1]
	bool			groupBit;	// Group bit set or not
	unsigned short	switchType;	// 0: swich off; 1: switch on; 2: set dim level
	unsigned short	unit;		// Unit code of received code [0..15]
	unsigned short	dimLevel;	// Dim level [0..15] iff switchType == 2
};

/**
* Initializes the decoder.
*
* If interrupt >= 0, init will register pin <interrupt> to this library.
* If interrupt < 0, no interrupt is registered. In that case, you have to call interruptHandler()
* yourself whenever the output of the receiver changes, or you can use InterruptChain.
*
* @param interrupt 	The interrupt as is used by Arduino's attachInterrupt function. See attachInterrupt for details.
					If < 0, you must call interruptHandler() yourself.
* @param minRepeats The number of times the same code must be received in a row before the callback is called
* @param callback Pointer to a callback function, with signature void (*func)(NewRemoteCode)
*/
void receiver_init(void);

/**
	* Called every time the signal level changes (high to low or vice versa). Usually called by interrupt.
	*/
void receiver_interruptHandler(void);

/**
* Handler that is run when a complete message has been received
*/
void reciever_msg_handler(struct NewRemoteCode msg);

/**
* Handler that is run when a data is being received from the 433 receiver
*/
void reciever_data_pin_init(void);

void configure_tc(void);

uint32_t micros(void);


#endif /* NEW_REMOTE_RECEIVER_H_ */