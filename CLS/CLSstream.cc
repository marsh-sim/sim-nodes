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
#include <atomic>
#include <cstdio>
#include <iostream>
#include <csignal>

using namespace std::chrono_literals;

// Global flags for signal handling
std::atomic<bool> signalReceived(false);

void signalHandler(int signal)
{
	switch (signal)
	{
		case SIGINT:
		case SIGTERM:
			std::cout << "\nReceived termination signal, shutting down..." << std::endl;
			signalReceived.store(true);
			break;

		case SIGTSTP:
			std::cout << "\nPausing communication..." << std::endl;
			paused.store(true);
			break;

		case SIGCONT:
			std::cout << "\nResuming communication..." << std::endl;
			paused.store(false);
			break;

		default:
			break;
	}
}

void setupSignalHandlers(void)
{
	struct sigaction sa;
	sa.sa_handler = signalHandler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;

  // Handle Ctrl+C (SIGINT)
  if (sigaction(SIGINT, &sa, nullptr) == -1)
  {
  std::cerr << "Failed to set SIGINT handler" << std::endl;
  }
  // Handle termination signal
  if (sigaction(SIGTERM, &sa, nullptr) == -1)
  {
    std::cerr << "Failed to set SIGTERM handler" << std::endl;
  }

  // Handle Ctrl+Z (SIGTSTP)
  if (sigaction(SIGTSTP, &sa, nullptr) == -1)
  {
    std::cerr << "Failed to set SIGTSTP handler" << std::endl;
  }

  // Handle continue signal
  if (sigaction(SIGCONT, &sa, nullptr) == -1)
  {
    std::cerr << "Failed to set SIGCONT handler" << std::endl;
  }
}

int main(void)
{
	// initialize the interface
	CLSInterface CLS;

	setupSignalHandlers();

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
	// TODO: encode messages in MAVLINK, and send them to MARSH
	CLS.startCommsThread();
	std::cout << "Press Ctrl+C to exit, Ctrl+Z to pause, Ctrl+Q to resume" << std::endl;

	// send message to initiate async data transmission
	// TODO: send Data Transfer message with data code msgESTABLISH_LINK_FOR_ASYCH_COMMS [53]
	//       (see WAT-MAN-CPD-1i5.0[COmmunication Protocol].pdf -- Section 2.2.3)
	// CLS.beginDataTransfer()
	//

	// main loop: for now, we just wait for a signal
	while (!signalReceived.load())
	{
		// could do other work here if needed

		// handle paused state
		if (paused.load())
		{
			// do something
		}
	}

	std::cout << "Shutting down..." << std::endl;

	// Stop the communication thread
	CLS.stopCommsThread();

	std::cout << "Shutdown complete." << std::endl;

	return 0;
}
