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

#include "Wittenstein/CLSUtils.h"
#include <iostream>

using namespace std::chrono_literals;

int main(void)
{
	// initialize the interface
	CLSInterface CLS;

	// initialize the MARSH node handle
	// CLSMarshNode MarshNode;

	if (!CLS.initialize())
	{
		std::cerr << CLS.getLastError() << std::endl;
		return 1;
	}

	// Traverse and activate all the axes
	CLS.traverseAll();
	CLS.sleep(5000ms);
	CLS.activateAll();
	CLS.sleep(1000ms);

	// start the thread to deal with incoming messages
	// TODO: POSIX-equivalent to _beginthread(clsCommsThread, 0, NULL)
	// TODO: receive messages, encode them in MAVLINK, and send them to MARSH
	// MarshNode.beginTread()

	// send message to initiate async data transmission
	// TODO: send Data Transfer message with data code msgESTABLISH_LINK_FOR_ASYCH_COMMS [53]
	//       (see WAT-MAN-CPD-1i5.0[COmmunication Protocol].pdf -- Section 2.2.3)
	// CLS.beginDataTransfer()

	// M
}
