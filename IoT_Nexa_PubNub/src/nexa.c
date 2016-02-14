/*
 * new_remote_reciever.c
 *
 * Created: 22.01.2016 10:57:56
 *  Author: magne.normann
 */ 

#include <asf.h>
#include "nexa.h"
#include "conf_extint.h"


/************
* NewRemoteReceiver
Protocol. (Copied from Wieltje, http://www.circuitsonline.net/forum/view/message/1181410#1181410,
but with slightly different timings, as measured on my device.)
		_   _
'0':   | |_| |_____ (T,T,T,5T)
		_       _
'1':   | |_____| |_	(T,5T,T,T)
		_   _
dim:   | |_| |_     (T,T,T,T)
T = short period of ~260µs. However, this code tries
to figure out the correct period
A full frame looks like this:
- start pulse: 1T high, 10.44T low
- 26 bit:  Address
- 1  bit:  group bit
- 1  bit:  on/off/[dim]
- 4  bit:  unit
- [4 bit:  dim level. Only present of [dim] is chosen]
- stop pulse: 1T high, 40T low
************/

void receiver_init(void) {
	/* Init struct variables */
	receiver_in_callback	= false;
	receiver_state			= -1;
	
	/* Init TC for millis() */
	configure_tc();
	
	/* Init external interrupt on CONF_EXTINT_PIN */
	reciever_data_pin_init();
}

void receiver_interruptHandler(void) {
	static unsigned short		receivedBit;		// Contains "bit" currently receiving
	static struct NewRemoteCode	receivedCode;		// Contains received code
	static struct NewRemoteCode	previousCode;		// Contains previous received code
	static unsigned short		repeats = 0;		// The number of times the an identical code is received in a row.
	static unsigned long		edgeTimeStamp[3] = {0, };	// Timestamp of edges
	static unsigned int			min1Period, max1Period, min5Period, max5Period;
	static bool					skip;

	// Filter out too short pulses. This method works as a low pass filter.
	edgeTimeStamp[1] = edgeTimeStamp[2];
	edgeTimeStamp[2] = micros(); //sample time
	//restart counter

	if (skip) {
		skip = false;
		return;
	}

	if (receiver_state >= 0 && edgeTimeStamp[2]-edgeTimeStamp[1] < min1Period) {
		// Last edge was too short.
		// Skip this edge, and the next too.
		skip = true;
		return;
	}

	unsigned int duration = edgeTimeStamp[1] - edgeTimeStamp[0];
	edgeTimeStamp[0] = edgeTimeStamp[1];

	// Note that if state>=0, duration is always >= 1 period.

	if (receiver_state == -1) {
		// wait for the long low part of a stop bit.
		// Stopbit: 1T high, 40T low
		// By default 1T is 260us, but for maximum compatibility go as low as 120us
		if (duration > 4800) { // =40*120us, minimal time between two edges before decoding starts.
			// Sync signal received.. Preparing for decoding
			repeats = 0;

			receivedCode.period = duration / 40; // Measured signal is 40T, so 1T (period) is measured signal / 40.

			// Allow for large error-margin. ElCheapo-hardware :(
			min1Period = receivedCode.period * 3 / 10; // Lower limit for 1 period is 0.3 times measured period; high signals can "linger" a bit sometimes, making low signals quite short.
			max1Period = receivedCode.period * 3; // Upper limit for 1 period is 3 times measured period
			min5Period = receivedCode.period * 3; // Lower limit for 5 periods is 3 times measured period
			max5Period = receivedCode.period * 8; // Upper limit for 5 periods is 8 times measured period
		}
		else {
			return;
		}
	} else if (receiver_state == 0) { // Verify start bit part 1 of 2
		// Duration must be ~1T
		if (duration > max1Period) {
			receiver_state = -1;
			return;
		}
		// Start-bit passed. Do some clean-up.
		receivedCode.address = receivedCode.unit = receivedCode.dimLevel = 0;
	} else if (receiver_state == 1) { // Verify start bit part 2 of 2
		// Duration must be ~10.44T
		if (duration < 7 * receivedCode.period || duration > 15 * receivedCode.period) {
			receiver_state = -1;
			return;
		}
	} else if (receiver_state < 146) { // state 146 is first edge of stop-sequence. All bits before that adhere to default protocol, with exception of dim-bit
		receivedBit <<= 1;

		// One bit consists out of 4 bit parts.
		// bit part durations can ONLY be 1 or 5 periods.
		if (duration <= max1Period) {
			receivedBit &= 0b1110; // Clear LSB of receivedBit
		}
		else if (duration >= min5Period && duration <= max5Period) {
			receivedBit |= 0b1; // Set LSB of receivedBit
		}
		else { // Otherwise the entire sequence is invalid
			receiver_state = -1;
			return;
		}

		if (receiver_state % 4 == 1) { // Last bit part? Note: this is the short version of "if ( (_state-2) % 4 == 3 )"
			// There are 3 valid options for receivedBit:
			// 0, indicated by short short short long == 0b0001.
			// 1, short long shot short == 0b0100.
			// dim, short shot short shot == 0b0000.
			// Everything else: inconsistent data, trash the whole sequence.


			if (receiver_state < 106) {
				// States 2 - 105 are address bit states

				receivedCode.address <<= 1;

				// Decode bit. Only 4 LSB's of receivedBit are used; trim the rest.
				switch (receivedBit & 0b1111) {
					case 0b0001: // Bit "0" received.
						// receivedCode.address |= 0; But let's not do that, as it is wasteful.
						break;
					case 0b0100: // Bit "1" received.
						receivedCode.address |= 1;
						break;
					default: // Bit was invalid. Abort.
						receiver_state = -1;
						return;
				}
			} else if (receiver_state < 110) {
				// States 106 - 109 are group bit states.
				switch (receivedBit & 0b1111) {
					case 0b0001: // Bit "0" received.
						receivedCode.groupBit = false;
						break;
					case 0b0100: // Bit "1" received.
						receivedCode.groupBit = true;
						break;
					default: // Bit was invalid. Abort.
						receiver_state = -1;
						return;
				}
			} else if (receiver_state < 114) {
				// States 110 - 113 are switch bit states.
				switch (receivedBit & 0b1111) {
					case 0b0001: // Bit "0" received.
						receivedCode.switchType = 0;
						break;
					case 0b0100: // Bit "1" received.
						receivedCode.switchType = 1;
						break;
					case 0b0000: // Bit "dim" received.
						receivedCode.switchType = 2;
						break;
					default: // Bit was invalid. Abort.
						receiver_state = -1;
						return;
				}
			} else if (receiver_state < 130){
				// States 114 - 129 are unit bit states.
				receivedCode.unit <<= 1;

				// Decode bit.
				switch (receivedBit & 0b1111) {
					case 0b0001: // Bit "0" received.
						// receivedCode.unit |= 0; But let's not do that, as it is wasteful.
						break;
					case 0b0100: // Bit "1" received.
						receivedCode.unit |= 1;
						break;
					default: // Bit was invalid. Abort.
						receiver_state = -1;
						return;
				}

				// Only if there is a dim-action chosen the dim-bits are present in the signal.
				// Thus, if switchType is on or off, skip these dim-bits.
				if (receiver_state == 129 && receivedCode.switchType != 2) {
					receiver_state = 145; // 4 bits x 4 states = 16 states to skip.
				}
			} else if (receiver_state < 146){
				// States 130 - 145 are dim bit states. Note that these are skipped unless switchType == 2.
				receivedCode.dimLevel <<= 1;

				// Decode bit.
				switch (receivedBit) {
					case 0b0001: // Bit "0" received.
						// receivedCode.dimLevel |= 0; But let's not do that, as it is wasteful.
						break;
					case 0b0100: // Bit "1" received.
						receivedCode.dimLevel |= 1;
						break;
					default: // Bit was invalid. Abort.
						receiver_state = -1;
						return;
				}
			}
		}
	} else if (receiver_state == 146) { // Verify stop bit part 1 of 2
		// Duration must be ~1T
		if (duration < min1Period || duration > max1Period) {
			receiver_state = -1;
			return;
		}
	} else if (receiver_state == 147) { // Verify stop bit part 2 of 2
		// Duration must be ~40T
		if (duration < 20 * receivedCode.period || duration > 80 * receivedCode.period) {
			receiver_state = -1;
			return;
		}

		// receivedCode is a valid code!

		if (
				receivedCode.address != previousCode.address ||
				receivedCode.unit != previousCode.unit ||
				receivedCode.dimLevel != previousCode.dimLevel ||
				receivedCode.groupBit != previousCode.groupBit ||
				receivedCode.switchType != previousCode.switchType
			) { // memcmp isn't deemed safe
			repeats=0;
			previousCode = receivedCode;
		}

		repeats++;

		if (repeats>=MIN_REPEATS) {
			if (!receiver_in_callback) {
				receiver_in_callback = true;
				reciever_msg_handler(receivedCode); //
				receiver_in_callback = false;
			}
			// Reset after callback.
			receiver_state=-1;
			return;
		}

		// Reset for next round
		receiver_state=0; // no need to wait for another sync-bit!
		return;
	}

	receiver_state++;
	return;
}

void reciever_data_pin_init(void){
	/* configure external interrupt */
	struct extint_chan_conf config_extint_chan;

	extint_chan_get_config_defaults(&config_extint_chan);

	config_extint_chan.gpio_pin           = CONF_EXTINT_PIN;
	config_extint_chan.gpio_pin_mux       = CONF_EXTINT_MUX;
	config_extint_chan.gpio_pin_pull      = CONF_EXTINT_PIN_PULL;
	config_extint_chan.detection_criteria = CONF_EXTINT_DETECTION_CRITERIA;

	extint_chan_set_config(CONF_EXTINT_EIC_LINE, &config_extint_chan);
	
	/* Configure callbacks */
	extint_register_callback(receiver_interruptHandler,
	EXT1_IRQ_INPUT,
	EXTINT_CALLBACK_TYPE_DETECT);

	extint_chan_enable_callback(EXT1_IRQ_INPUT,
	EXTINT_CALLBACK_TYPE_DETECT);
}

void reciever_msg_handler(struct NewRemoteCode msg){
	//port_pin_toggle_output_level(LED_0_PIN);
	port_pin_set_output_level(LED_0_PIN, !msg.switchType);
}


void configure_tc(void){

	struct tc_config config_tc;

	tc_get_config_defaults(&config_tc);

	config_tc.clock_source = GCLK_GENERATOR_2;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV1;
	config_tc.counter_size    = TC_COUNTER_SIZE_32BIT;
	tc_init(&tc_instance, TC4, &config_tc);

	tc_enable(&tc_instance);
}

uint32_t micros(void){
	return tc_get_count_value(&tc_instance);
}