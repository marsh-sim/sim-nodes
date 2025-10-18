// CLSstream.cc
//
// CLS STREAM
//
// MARSH node providing basic interface to Wittenstein Control Loading System (CLS), by
// reading Data Trasfer Messages set up as defined in
//
//   WAT-MAN-CPD-1i5.0[Communication Protocol].pdf 
//   -- Section 2.2.3 Repetitive Asynchronous Data Transmissions
//
// Makes extensive usage of (rewritten) libraries from PoliMiClsDemo, provided by
// Wittenstein.
//
// Author: Andrea Zanoni <andrea.zanoni@polimi.it>

#include <iostream>
#include "Wittenstein/CLSUtils.h"
#include <chrono>

using namespace std::chrono_literals;

int main(void)
{
	pdFLOAT fData[CLS_NO_AXES];

	// Initialize UDP connection
	clsUDPInit();

	// Initialize and Activate the 2 axes
	clsAllTraverse();
	clsSleep(5000ms);
	clsAllActive();
	clsSleep(1000ms);

	// start the thread to deal with incoming messages
	// TODO: POSIX-equivalent to _beginthread(clsCommsThread, 0, NULL)

	// send message to initiate async data transmission
	// TODO: send Data Transfer message with data code msgESTABLISH_LINK_FOR_ASYCH_COMMS [53]
	//       (see WAT-MAN-CPD-1i5.0[COmmunication Protocol].pdf -- Section 2.2.3)

	// TODO: receive messages, encode them in MAVLINK, and send them to MARSH
}
