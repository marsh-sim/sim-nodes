#include "CLSUtils.h"
#include "../../src/controlloadingdata.h"
#include "debug.h"
#include <asm-generic/socket.h>
#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
// #include <mutex>

using namespace std::chrono_literals;

// Previous positions for velocity calculation
static std::array<float, MAX_CONTROL_AXES> previous_positions = {0.0f};
static std::chrono::steady_clock::time_point last_position_time = std::chrono::steady_clock::now();

bool isValidMessageCode(const int code)
{
	switch(static_cast<CLSMessageCode>(code))
	{
		case CLSMessageCode::NO_MESSAGE:
		case CLSMessageCode::GO_ACTIVE:
		case CLSMessageCode::START_TRAVERSAL:
		case CLSMessageCode::NEW_MASTER_CURVE:
		case CLSMessageCode::SELECT_MASTER_CURVE:
		case CLSMessageCode::TRIM_VALUE:
		case CLSMessageCode::QFEEL:
		case CLSMessageCode::NEW_ADDIN_CURVE:
		case CLSMessageCode::TRIM_RATE:
		case CLSMessageCode::SAVE_CONFIG:
		case CLSMessageCode::GO_PASSIVE:
		case CLSMessageCode::POS_END_STOP:
		case CLSMessageCode::NEG_END_STOP:
		case CLSMessageCode::SELECT_ADDIN:
		case CLSMessageCode::ADDIN_AMPLITUDE:
		case CLSMessageCode::DATUM_POSITION:
		case CLSMessageCode::STICK_SHAKE_HZ:
		case CLSMessageCode::STATIC_FRICTION:
		case CLSMessageCode::DYNAMIC_FRICTION:
		case CLSMessageCode::STICK_SHAKE_AMP:
		case CLSMessageCode::NUMBER_OF_AXIS:
		case CLSMessageCode::FORCES:
		case CLSMessageCode::POSITIONS:
		case CLSMessageCode::MODEL_DAMPING:	
		case CLSMessageCode::MODEL_HZ:
		case CLSMessageCode::MASTER_PHASE_IN_TIME:
		case CLSMessageCode::ADDIN_PHASE_IN_TIME:
		case CLSMessageCode::ADDIN_START:
		case CLSMessageCode::ADDIN_END:
		case CLSMessageCode::ADDIN_ABQF:
		case CLSMessageCode::ADDIN_TTD:
		case CLSMessageCode::FORCE_CALIBRATION_FACTOR:
		case CLSMessageCode::AXIS_TO_LINK_TO:
		case CLSMessageCode::MAX_QFEEL_CHANGE_RATE:
		case CLSMessageCode::MAX_ADDIN_AMP_CHANGE_RATE:
		case CLSMessageCode::NEGATIVE_QFEEL:
		case CLSMessageCode::COMMS_HEART_BEAT:
		case CLSMessageCode::RESTORE_CONFIG:
		case CLSMessageCode::TRIM_RELEASE_STATE:
		case CLSMessageCode::STATUS:
		case CLSMessageCode::CAN_IO_MESSAGE:
		case CLSMessageCode::ESTABLISH_LINK_FOR_ASYCH_COMMS:
		case CLSMessageCode::AUTOPILOT_MODE:
		case CLSMessageCode::AUTOPILOT_VALUE:
		case CLSMessageCode::STATIC_FRICTION_IN_TRIM_RELEASE:
		case CLSMessageCode::DYNAMIC_FRICTION_IN_TRIM_RELEASE:
		case CLSMessageCode::STICK_SHAKE_ENABLE:
		case CLSMessageCode::FORCE_BIAS:
		case CLSMessageCode::LOW_SWITCH_GRADIENT:
		case CLSMessageCode::SCM_HEARTBEAT:
		case CLSMessageCode::GATE_1_ADDIN:
		case CLSMessageCode::GATE_1_POSITION:
		case CLSMessageCode::GATE_1_RESTORE_MARGIN:
		case CLSMessageCode::GATE_1_FORCE_THRESHOLD:
		case CLSMessageCode::GATE_2_ADDIN:
		case CLSMessageCode::GATE_2_POSITION:
		case CLSMessageCode::GATE_2_RESTORE_MARGIN:
		case CLSMessageCode::GATE_2_FORCE_THRESHOLD:
		case CLSMessageCode::GATE_1_ACTIVE:
		case CLSMessageCode::GATE_1_RELEASE_BUTTON:
		case CLSMessageCode::GATE_1_BUTTON_ACTIVE:
		case CLSMessageCode::GATE_2_ACTIVE:
		case CLSMessageCode::GATE_2_RELEASE_BUTTON:
		case CLSMessageCode::GATE_2_BUTTON_ACTIVE:
		case CLSMessageCode::MOTOR_ERROR_STATUS:
		case CLSMessageCode::DAMPING_FACTOR:
		case CLSMessageCode::END_STOP_TORQUE_GAIN:
		case CLSMessageCode::IMPULSE_AMPLITUDE:
		case CLSMessageCode::IMPULSE_FREQUENCY:
		case CLSMessageCode::MARK_SPACE_RATIO:
		case CLSMessageCode::ZERO_LEVEL_OFFSET:
			return true;
		default:
			return false;
	}
}

CLSInterface::~CLSInterface(void)
{
  stopCommsThread();
}

bool CLSInterface::initialize(void) {
  int bindResult = initUDPSocket();

  if (bindResult != 0) {
    std::ostringstream oss;
    oss << "Failed to bind UDP socket: " << strerror(errno)
        << " (errno: " << errno << ")";

    switch (errno) {
    case EADDRINUSE:
      oss << "\nAddress already in use. Another process may be using this port.";
      break;
    case EACCES:
      oss << "\nPermission denied. You may need elevated privileges.";
      break;
    case EADDRNOTAVAIL:
      oss << "\nCannot assign requested address. Check that the IP address is valid.";
      break;
    case EINVAL:
      oss << "\nSocket is already bound to an address.";
      break;
    }

    lastError = oss.str();
    socketInitialized = false;
    return false;
  }

  socketInitialized = true;
  DEBUG_COUT("CLSInitialize(): socket bound to address " << scm_address << ":" << scm_port)
  return true;
}

bool CLSInterface::extractDataValues(pdINT &numValues,
                                        std::array<float, MAX_AXES> &outBuffer,
                                        const char *cBuf)
{
  numValues = 0;

  for (int channel = 0; channel < MAX_AXES; ++channel) {
    const int bufPos = channel * prtclLEN_TAKEN_BY_ONE_DATA_VALUE;

    // Stop if end-of-transmission marker found
    if (cBuf[bufPos] == EOT)
    {
      break;
    }

    // Decode sign
    const int sign = (cBuf[bufPos] == '+') ? 1 : -1;

    // Decode float (skip sign char)
    float value = 0.0f;
    if (sscanf(cBuf + bufPos + 1, "%f", &value) != 1)
    {
      return false; // malformed data
    }

    outBuffer[channel] = value * sign;
    ++numValues;
  }

  // Ensure we didn’t overflow our buffer
  // TODO: throw exception if we do?
  if (numValues >= MAX_AXES)
  {
    return false;
  }

  return true;
}

bool CLSInterface::decodeDataParameters(const pdCHAR *cBuf,
                                        pdINT iEnquiry,
                                        CLSmsg &msg)
{

  // Extract the message type (2 ASCII hex chars -> int)
  char cMsgType[3] = {0};
  cMsgType[0] = cBuf[prtclFIRST_DATA_TYPE_BYTE_OFFSET];
  cMsgType[1] = cBuf[prtclFIRST_DATA_TYPE_BYTE_OFFSET + 1];

  // Parse message type as hexadecimal
  int tempMessageType;
  if (sscanf(cMsgType, "%02x", &tempMessageType) != 1 || tempMessageType <= 0) {
    return false; // invalid message
  }

  // Convert to enum type, if we recognise the value. Otherwise, we'll just assign
  // CLSMessageCode::UNKNOWN
  if (!isValidMessageCode(tempMessageType))
  {
  	msg.messageType = CLSMessageCode::UNKNOWN;	
  }
  else
  {

    msg.messageType = static_cast<CLSMessageCode>(tempMessageType);
  }

  // Decode other parameters
  msg.tag = ASCII_HEX_TO_INT(cBuf[prtclTAG_OFFSET]);
  msg.startAxis = ASCII_HEX_TO_INT(cBuf[prtclSTART_AXIS_OFFSET]);
  msg.numberOfAxes = ASCII_HEX_TO_INT(cBuf[prtclNUMBER_OF_AXIS_OFFSET]);

  // If not an enquiry, extract data values
  if (iEnquiry == pdFALSE)
  {

		if (!extractDataValues(msg.numberOfDataValues, msg.data, cBuf + prtclSTART_OF_DATA_OFFSET))
		{
			// TODO: implement exceptions, for clarity
			return false; // data extraction failes
		}

		// Clamp to array capacity
		if (msg.numberOfDataValues > static_cast<int>(msg.data.size()))
		{
			msg.numberOfDataValues = static_cast<int>(msg.data.size());		
		}
		// TODO: warn user if we did not write to msg.

  }

  return true;
}

void CLSInterface::startCommsThread(void)
{
	if (threadRunning.load())
	{
		std::cerr << "Comms thread already running" << std::endl;
		return;
	}

	if (!socketInitialized)
	{
		std::cerr << "Socket not initialized, cannot start thread" << std::endl;
		return;
	}

	threadRunning.store(true);
	commsThread = std::thread(&CLSInterface::CommsThread, this);
}

void CLSInterface::stopCommsThread(void)
{
	closeSocket();
}

/* CLS interface utilities which set up connection + talk to the API functions */
void CLSInterface::CommsThread(void) {

	pdINT iMessageSize = -1;

	while (threadRunning.load())
	{

	  // Check if we're paused
	  if (paused.load())
	  {
	  	return;
	  }

		// set a timeout on recvfrom to allow checking threadRunning periodically
		struct timeval tv;
		tv.tv_sec = scm_socket_timeout;
		tv.tv_usec = 0;

		if (setsockopt(cls_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)))
		{
			std::cerr << "Failed to set socket timeout: " << strerror(errno) << std::endl;
			break;
		}

		// receive the next message
		iMessageSize = recvfrom(cls_socket, rxmsg, sizeof(rxmsg), 0, (struct sockaddr*)&rxpeer, &rxpeerlen);

		if (iMessageSize > 0)
		{
			// Process the message
			CLSmsg msg;
			if (decodeDataParameters(rxmsg, DATA_MESSAGE, msg))
			{
				switch (msg.messageType)
				{
					case CLSMessageCode::POSITIONS:
						usePositions(msg.data.data());
						break;
					case CLSMessageCode::FORCES:
						useForces(msg.data.data());
						break;
					case CLSMessageCode::UNKNOWN:
					default:
						DEBUG_COUT("Got message with code " << msg.messageType)
						break;
				}
			}
		}
		else if (iMessageSize < 0)
		{
			// Check if it is just a timeout (EAGAIN/EWOULDBLOCK) or a real error
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				std::cerr << "Error receiving UDP packet: " << strerror(errno) << std::endl;
				break;
			}
			// otherwise, timeout is normal -- loop will continue and check threadRunning
		}

		std::cout << "Comms thread exiting" << std::endl;
		
	}

	for( ;; )
	{
		/* See if there is a message to collect */
		iMessageSize = recvfrom( cls_socket, rxmsg, sizeof( rxmsg ), 0, ( struct sockaddr * )&rxpeer, &rxpeerlen );

		if( iMessageSize != -1 )
		{
			/* We have a message - process it */
			CLSmsg msg;
			decodeDataParameters(rxmsg, DATA_MESSAGE, msg);
			switch(msg.messageType)
			{
				case CLSMessageCode::FORCES:
					useForces(msg.data.data());
					break;
				case CLSMessageCode::POSITIONS:
					usePositions(msg.data.data());
					break;
				case CLSMessageCode::STATUS:
					useStatus(msg.data.data());
					break;
				case CLSMessageCode::LOW_SWITCH_GRADIENT:
					useLowSwitchGradient(msg.data.data());
					break;
				case CLSMessageCode::CAN_IO_MESSAGE:
					useIO(msg.data.data(), msg.tag);
					break;
				default:
				  // Just ignore unknown message types
				  // TODO: add (optional) debug output
				  break;
			}
		}
	}
}

/* Initialise the UDP socket */
int CLSInterface::initUDPSocket(void)
{

	// tx settings
	txpeer.sin_family = AF_INET;
	txpeer.sin_port = htons(scm_port);
	txpeer.sin_addr.s_addr = inet_addr(scm_address.c_str());

	// udp_tx_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	// rx settings
	rxpeer.sin_family = AF_INET;
	rxpeer.sin_port = htons(rx_port);
	rxpeer.sin_addr.s_addr = htonl(INADDR_ANY);
	rxpeerlen = sizeof(rxpeer);

	cls_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	return bind(cls_socket, (struct sockaddr*)&rxpeer, sizeof(rxpeer));
}

/* keep the system tidy and close the network connection */
void CLSInterface::closeSocket(void)
{
	  // Thread-safe version with mutex (if needed)
    // // std::lock_guard<std::mutex> lock(socketMutex);
    
    if (cls_socket < 0)
    {
        return;
    }
    
    shutdown(cls_socket, SHUT_RDWR);
    
    if (close(cls_socket) < 0)
    {
        std::cerr << "Warning: Failed to close socket: " 
                  << strerror(errno) << std::endl;
    }
    
    cls_socket = -1;
}

// Free function for CRC generation (used internally)
static unsigned pdSHORT prvGenerateCRC_internal(const pdCHAR * cBuf, pdINT iSize)
{
	unsigned pdSHORT usCRC = 0;

	// for each Byte
	for (int i = 0; i < iSize; i++)
	{
		usCRC ^= static_cast<unsigned char>(cBuf[i]);

		// for each bit
		for (int j = 0; j < 8; j++)
		{
			if (usCRC & 0x0001)
			{
				usCRC = (usCRC >> 1) ^ 0xa001;
			}
			else
			{
				usCRC >>= 1;
			}
		}
	}
	return usCRC;
}

// Member function wrapper
unsigned pdSHORT CLSInterface::generateCRC(const pdCHAR* cBuf, pdINT iSize)
{
	return prvGenerateCRC_internal(cBuf, iSize);
}

pdINT CLSInterface::generateQuery(pdCHAR *cBuf, 
                                   pdINT iBufSize,  // ADD buffer size parameter!
                                   pdINT iStartAxis, 
                                   pdINT iNumberOfAxis, 
                                   pdINT iFuncCode, 
                                   pdINT iTag, 
                                   pdINT iNode)
{
    // Validate input parameters
    if (cBuf == nullptr || iBufSize < 20)  // Minimum message size
    {
        return -1;  // Error: invalid buffer
    }
    
    // Build the message (without CRC)
    int bytesWritten = snprintf(cBuf, iBufSize, 
                                "%c%01x%02x%01x%01x%01x%c", 
                                STX, iNode, iFuncCode, iTag, 
                                iStartAxis, iNumberOfAxis, ENQ);
    
    // Check for truncation
    if (bytesWritten < 0 || bytesWritten >= iBufSize)
    {
        return -1;  // Error: buffer too small
    }
    
    // Calculate CRC on the message so far
    unsigned pdSHORT usCRC = generateCRC(cBuf, bytesWritten);
    
    // Append CRC bytes (handle as binary, not string!)
    if (bytesWritten + 2 >= iBufSize)
    {
        return -1;  // Error: not enough space for CRC
    }
    
    // Add CRC bytes in little-endian order
    cBuf[bytesWritten] = static_cast<pdCHAR>(usCRC & 0xFF);         // LSB
    cBuf[bytesWritten + 1] = static_cast<pdCHAR>((usCRC >> 8) & 0xFF);  // MSB
    
    // Return total message length (including CRC)
    return bytesWritten + 2;
}

void CLSInterface::request(CLSMessageCode msgcode)
{
	msgSize = generateQuery(txmsg, sizeof(txmsg), CLS_START_AXIS, CLS_NO_AXES, static_cast<int>(msgcode), 0, scm_node);

	if (msgSize < 0)
	{
		std::cerr << "generateQuery() Error: Failed to generate query message" << std::endl;
		return;
	}

	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*)&txpeer, sizeof(txpeer));
}

pdINT CLSInterface::createDataTransferString(pdINT iNode, 
                                              pdINT iStartAxis, 
                                              pdINT iNumOfAxis, 
                                              pdINT iFuncCode, 
                                              pdINT iTag, 
                                              const pdFLOAT *fBuf,  // const since we don't modify
                                              pdCHAR *cBuf, 
                                              pdINT iBufferSize)
{
    // Validate inputs
    if (cBuf == nullptr || fBuf == nullptr || iBufferSize <= 0 || iNumOfAxis < 0)
    {
        return -1;  // Error: invalid parameters
    }
    
    pdINT iBufPos = 0;  // Current position in buffer
    pdINT bytesWritten;
    
    // Create the header
    bytesWritten = snprintf(cBuf + iBufPos, iBufferSize - iBufPos,
                            "%c%01x%02x%01x%01x%01x", 
                            STX, iNode, iFuncCode, iTag, iStartAxis, iNumOfAxis);
    
    if (bytesWritten < 0 || bytesWritten >= iBufferSize - iBufPos)
    {
        return -1;  // Error: buffer too small for header
    }
    
    iBufPos += bytesWritten;
    
    // Add each data value
    for (pdINT iLoop = 0; iLoop < iNumOfAxis; iLoop++)
    {
        bytesWritten = snprintf(cBuf + iBufPos, iBufferSize - iBufPos,
                                "%+01.04f", fBuf[iLoop]);
        
        if (bytesWritten < 0 || bytesWritten >= iBufferSize - iBufPos)
        {
            return -1;  // Error: buffer too small for data values
        }
        
        iBufPos += bytesWritten;
    }
    
    // Add EOT character
    if (iBufPos + 1 >= iBufferSize)
    {
        return -1;  // Error: no space for EOT
    }
    
    cBuf[iBufPos] = EOT;
    iBufPos++;
    
    // Calculate CRC on the message so far
    unsigned pdSHORT usCRC = generateCRC(cBuf, iBufPos);
    
    // Check space for CRC bytes
    if (iBufPos + 2 >= iBufferSize)
    {
        return -1;  // Error: no space for CRC
    }
    
    // Append CRC bytes directly (not as string!)
    cBuf[iBufPos] = static_cast<pdCHAR>(usCRC & 0xFF);         // LSB
    cBuf[iBufPos + 1] = static_cast<pdCHAR>((usCRC >> 8) & 0xFF);  // MSB
    
    // Return total message size
    return iBufPos + 2;
}

/* Generate the Traverse command String + send */
void CLSInterface::traverseAll(void)
{
	//send traversal message
	msgSize = iCreateDataTransferString(scm_node, CLS_START_AXIS, CLS_NO_AXES, static_cast<int>(CLSMessageCode::START_TRAVERSAL), 0, txdata, txmsg, sizeof( txdata ) ); 
	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr *)&txpeer, sizeof(txpeer));

	//wait a moment
	sleep(5000ms);
}

/* Generate the Activate command String + send */
/* Note the extra command to enable can IO messages */
void CLSInterface::activateAll(void)
{
	//send go active message
	msgSize = iCreateDataTransferString(scm_node, CLS_START_AXIS, CLS_NO_AXES, static_cast<int>(CLSMessageCode::GO_ACTIVE), 0, txdata, txmsg, sizeof(txdata)); 
	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*)&txpeer, sizeof(txpeer));

	/* are we using CANIO */
	if(pdTRUE == CLS_USE_CAN_IO)
	{
		txdata[0] = 1.0F;
		msgSize = iCreateDataTransferString(scm_node, CLS_START_AXIS, 1, static_cast<int>(CLSMessageCode::ESTABLISH_LINK_FOR_ASYCH_COMMS), 0, txdata, txmsg, sizeof(txdata)); 
		sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*)&txpeer, sizeof(txpeer));
	}
}

/* Generate the GoPassive command String + send */
void CLSInterface::deactivateAll(void)
{
	//send go passive message
	msgSize = iCreateDataTransferString(scm_node, CLS_START_AXIS, CLS_NO_AXES, static_cast<int>(CLSMessageCode::GO_PASSIVE), 0, txdata, txmsg, sizeof(txdata));
	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*)&txpeer, sizeof(txpeer));
	return;
}

/* Begin asynchronous data transfer - establishes link for async comms */
void CLSInterface::beginDataTransfer(void)
{
	// Set data value to 1.0 to enable async communication
	txdata[0] = 1.0F;

	// Create and send ESTABLISH_LINK_FOR_ASYCH_COMMS message
	msgSize = createDataTransferString(scm_node, CLS_START_AXIS, 1,
	                                    static_cast<int>(CLSMessageCode::ESTABLISH_LINK_FOR_ASYCH_COMMS),
	                                    0, txdata, txmsg, sizeof(txdata));

	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*)&txpeer, sizeof(txpeer));

	std::cout << "Asynchronous data transfer link established" << std::endl;
}

/* Generate the Reload command String + send */
void CLSInterface::restore(pdINT iLocation)
{
	//send restore message
	msgSize = iCreateDataTransferString(scm_node, CLS_START_AXIS, CLS_NO_AXES, static_cast<int>(CLSMessageCode::RESTORE_CONFIG), iLocation, txdata, txmsg, sizeof(txdata)); 
	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*)&txpeer, sizeof(txpeer));

	return;
}

/* Do something with a status response */
void CLSInterface::useStatus(const pdFLOAT *fData)
{
	printf("Status :");
	for(int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES ; iLoop++)
	{
		printf("\tAxis %d: %f", iLoop, fData[iLoop]);
	}
	printf("\r\n");
}

/* Do something with a position response */
void CLSInterface::usePositions(const pdFLOAT *fData)
{
	// Get current time for velocity calculation
	auto current_time = std::chrono::steady_clock::now();
	auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_position_time).count();
	float time_delta_sec = time_delta / 1000.0f;

	printf("Positions :");
	for(int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES && iLoop < static_cast<int>(MAX_CONTROL_AXES) ; iLoop++)
	{
		printf("\tAxis %d: %f", iLoop, fData[iLoop]);

		// Update global control loading data
		g_controlLoadingData[iLoop].axis = static_cast<uint8_t>(iLoop);
		g_controlLoadingData[iLoop].position = fData[iLoop];

		// Calculate velocity (deg/s) from position change
		if (time_delta_sec > 0.001f) // Avoid division by very small numbers
		{
			g_controlLoadingData[iLoop].velocity = (fData[iLoop] - previous_positions[iLoop]) / time_delta_sec;
		}

		// Store current position for next iteration
		previous_positions[iLoop] = fData[iLoop];
	}
	printf("\r\n");

	// Update timestamp for next velocity calculation
	last_position_time = current_time;
}

/* Do something with a force response */
void CLSInterface::useForces(const pdFLOAT *fData)
{
	printf("Forces :");
	for(int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES && iLoop < static_cast<int>(MAX_CONTROL_AXES) ; iLoop++)
	{
		printf("\tAxis %d: %f", iLoop, fData[iLoop]);

		// Update global control loading data with force
		g_controlLoadingData[iLoop].force = fData[iLoop];
	}
	printf("\r\n");
}

/* Do something with a low switch gradient response */
void CLSInterface::useLowSwitchGradient(const pdFLOAT *fData)
{
	printf("Low Switch Gradients :");
	for(int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES ; iLoop++)
	{
		printf("\tAxis %d: %f", iLoop, fData[iLoop]);
	}
	printf("\r\n");
}

/* Do something with a CAN IO response */
void CLSInterface::useIO(const pdFLOAT *fData,const pdINT iBus)
{
	printf("CAN_IO :\tCARD %x\t%x %x %x %x %x\r\n", (pdINT)fData[0], (pdINT)fData[1], (pdINT)fData[2], (pdINT)fData[3], (pdINT)fData[4], (pdINT)fData[5]);
}


/* Write Low Switch Gradient to SCM */
void CLSInterface::setLowSwitchGradient(pdFLOAT* fData, pdINT iStartAxis, pdINT iNoOfAxes)
{
	msgSize = createDataTransferString(scm_node, iStartAxis, iNoOfAxes, static_cast<int>(CLSMessageCode::LOW_SWITCH_GRADIENT), 0, fData, txmsg, 512);
	sendto(cls_socket, txmsg, msgSize, 0, (struct sockaddr*) &txpeer, sizeof(txpeer));

}

void CLSInterface::sleep(std::chrono::milliseconds sleep_time)
{
  using namespace std::chrono;

  std::cout << "Sleeping for " << sleep_time.count() << " ms..."
            << std::endl;

  const auto start = high_resolution_clock::now();
  std::this_thread::sleep_for(sleep_time);
  const auto end = high_resolution_clock::now();

  const duration<double, std::milli> elapsed = end - start;

  std::cout << "Waited " << elapsed.count() << " ms\n";
}
