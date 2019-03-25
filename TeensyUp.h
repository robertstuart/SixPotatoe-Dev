/******************************************************************************
 * TeensyUp.h
 *
 *  Constants for communication between UpBoard and Arduino Teensy
 *
 *****************************************************************************/

#ifndef TEENSYUP_H_
#define TEENSYUP_H_


// Messages from the Teensy to the Up-Board
const int TOUP_START_LOG      = 130;  // Current X position in meters
const int TOUP_END_LOG        = 131;  // Current Y position in meters
const int TOUP_LOG            = 132;  // Current Pitch for adjusting vertical row

const int TOUP_KILL_UP        = 140;  // Current Heading



//Messages from the UpBoard to the Teensy
const int FRUP_STAT           = 129; // true = route in progress, periodic alive indicator
const int FRUP_QUERY          = 130; // request data  


#endif /* TEENSYUP_H_ */

