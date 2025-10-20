#include "CLSUtils.h"
#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>
#include <array>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

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


CLSInterface::CLSInterface(void)
{
	initUDPSocket();
}

CLSInterface::CLSInterface(const std::string &address, const unsigned int scm_port_in, const unsigned int rx_port_in)
    : cls_socket(-1),          // initialize socket to invalid
      scm_port(scm_port_in),   // assign provided SCM port
      rx_port(rx_port_in),     // assign provided RX port
      cls_address(address)     // assign provided address
{
	initUDPSocket();
}

bool CLSInterface::extractDataValues(int &numValues,
                                        std::array<float, MAX_AXES> &outBuffer,
                                        const char *cBuf)
{
  numValues = 0;

  for (int channel = 0; channel < MAX_AXES; ++channel) {
    const int bufPos = channel * prtclLEN_TAKEN_BY_ONE_DATA_VALUE;

    // Stop if end-of-transmission marker found
    if (cBuf[bufPos] == EOT)
      break;

    // Decode sign
    const int sign = (cBuf[bufPos] == '+') ? 1 : -1;

    // Decode float (skip sign char)
    float value = 0.0f;
    if (sscanf(cBuf + bufPos + 1, "%f", &value) != 1)
      return false; // malformed data

    outBuffer[channel] = value * sign;
    ++numValues;
  }

  // Ensure we didn’t overflow our buffer
  // TODO: throw exception if we do?
  if (numValues >= MAX_AXES)
    return false;

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

/* CLS interface utilities which set up connection + talk to the API functions */
void CLSInterface::CommsThread(void) {
	pdINT iMessageSize;
	int axis;
	int num_axes;
	int data_code;
	int tag;
	int rxsize;

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
					useForces(rxdata);
					break;
				case CLSMessageCode::POSITIONS:
					usePositions(rxdata);
					break;
				case CLSMessageCode::STATUS:
					useStatus(rxdata);
					break;
				case CLSMessageCode::LOW_SWITCH_GRADIENT:
					useLowSwitchGradient(rxdata);
					break;
				case CLSMessageCode::CAN_IO_MESSAGE:
					useIO(rxdata, tag);
					break;
				default:
				  // Just ignore unknown message types
				  // TODO: add (optional) debug output
				  break;
			}
		}
	}
}

/* This is the message receive function - intended to run as a separate thread / task in the background */
void clsCommsThread( void *dummy )
{
	pdINT iMessageSize;
	int axis;
	int num_axes;
	int data_code;
	int tag;
	int rxsize;

	for( ;; )
	{
		/* See if there is a message to collect */
		iMessageSize = recvfrom( cls_socket, rxmsg, sizeof( rxmsg ), 0, ( struct sockaddr * )&rxpeer, &rxpeerlen );

		if( iMessageSize != -1 )
		{
			/* We have a message - process it */
			prvDecodeDataParameters( rxmsg, &data_code, &tag, &axis, &num_axes, rxdata, DATA_MESSAGE, &rxsize);
			switch( data_code )
			{
				case msgFORCES:				clsUseForces( rxdata );
											break;
				case msgPOSITIONS:			clsUsePositions( rxdata );
											break;
				case msgSTATUS:				clsUseStatus( rxdata );
											break;
				case msgLOW_SWITCH_GRADIENT:		clsUseLowSwitchGradient( rxdata );
											break;
				case msgCAN_IO_MESSAGE:		clsUseIO( rxdata, tag );
											break;
				default:					break;
			}
		}
	}
}

/* Initialise the UDP socket */
void CLSInterface::initUDPSocket(void)
{

	// tx settings
	txpeer.sin_family = AF_INET;
	txpeer.sin_port = htons(cls_port);
	txpeer.sin_addr.s_addr = inet_addr(cls_address);

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
void clsUDPClose(void)
{
	closesocket(cls_socket);
}

/* Generate the Status Query String + send */
void clsRequestStatus( )
{
	//send status request message
	msgSize = prvGenerateQuery( txmsg, CLS_START_AXIS, CLS_NO_AXES, msgSTATUS, 0, SCM_NODE );
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );
}

/* Generate the Traverse command String + send */
void clsAllTraverse( )
{
	//send traversal message
	msgSize = iCreateDataTransferString( SCM_NODE, CLS_START_AXIS, CLS_NO_AXES, msgSTART_TRAVERSAL, 0, txdata, txmsg, sizeof( txdata ) ); 
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );

	//wait a moment
	Sleep( 5 );
}

/* Generate the Activate command String + send */
/* Note the extra command to enable can IO messages */
void clsAllActive( )
{
	//send go active message
	msgSize = iCreateDataTransferString( SCM_NODE, CLS_START_AXIS, CLS_NO_AXES, msgGO_ACTIVE, 0, txdata, txmsg, sizeof( txdata ) ); 
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );

	/* are we using CANIO */
	if( pdTRUE == CLS_USE_CAN_IO )
	{
		txdata[ 0 ] = 1.0F;
		msgSize = iCreateDataTransferString( SCM_NODE, CLS_START_AXIS, 1, msgESTABLISH_LINK_FOR_ASYCH_COMMS, 0, txdata, txmsg, sizeof( txdata ) ); 
		sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );
	}
}

/* Generate the GoPassive command String + send */
void clsAllPassive( )
{
	//send go passive message
	msgSize = iCreateDataTransferString( SCM_NODE, CLS_START_AXIS, CLS_NO_AXES, msgGO_PASSIVE, 0, txdata, txmsg, sizeof( txdata ) ); 
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );
	return;
}

/* Generate the Reload command String + send */
void clsRestore( pdINT iLocation )
{
	//send restore message
	msgSize = iCreateDataTransferString( SCM_NODE, CLS_START_AXIS, CLS_NO_AXES, msgRESTORE_CONFIG, iLocation, txdata, txmsg, sizeof( txdata ) ); 
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );

	return;
}

/* Generate the Position Query String + send */
void clsRequestPositions( )
{
	//send get positions message
	msgSize = prvGenerateQuery( txmsg, CLS_START_AXIS, CLS_NO_AXES, msgPOSITIONS, 0, SCM_NODE );
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );
}

/* Generate the Force Query String + send */
void clsRequestForces( )
{
	//send get positions message
	msgSize = prvGenerateQuery( txmsg, CLS_START_AXIS, CLS_NO_AXES, msgFORCES, 0, SCM_NODE );
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );
}

/* Generate the Low Switch Gradient Query String + send */
void clsRequestLowSwitchGradient( )
{
	//send get Low Switch Gradient message
	msgSize = prvGenerateQuery( txmsg, CLS_START_AXIS, CLS_NO_AXES, msgLOW_SWITCH_GRADIENT, 0, SCM_NODE );
	sendto( cls_socket, txmsg, msgSize, 0, ( struct sockaddr * )&txpeer, sizeof( txpeer ) );
}

/* Do something with a status response */
void clsUseStatus( pdFLOAT *fData )
{
	printf("Status :" );
	for( int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES ; iLoop++ )
	{
		printf( "\tAxis %d: %f", iLoop, fData[ iLoop ] );
	}
	printf( "\r\n" );
}

/* Do something with a position response */
void clsUsePositions( pdFLOAT *fData )
{
	printf("Positions :" );
	for( int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES ; iLoop++ )
	{
		printf( "\tAxis %d: %f", iLoop, fData[ iLoop ] );
	}
	printf( "\r\n" );
}

/* Do something with a force response */
void clsUseForces( pdFLOAT *fData )
{
	printf("Forces :" );
	for( int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES ; iLoop++ )
	{
		printf( "\tAxis %d: %f", iLoop, fData[ iLoop ] );
	}
	printf( "\r\n" );
}

/* Do something with a force response */
void clsUseLowSwitchGradient( pdFLOAT *fData )
{
	printf("Low Switch Gradients :" );
	for( int iLoop = CLS_START_AXIS ; iLoop < CLS_NO_AXES ; iLoop++ )
	{
		printf( "\tAxis %d: %f", iLoop, fData[ iLoop ] );
	}
	printf( "\r\n" );
}

/* Do something with a CAN IO response */
void clsUseIO( pdFLOAT *fData, pdINT iBus )
{
	printf("CAN_IO :\tCARD %x\t%x %x %x %x %x\r\n", (pdINT) fData[ 0 ], (pdINT) fData[ 1 ], (pdINT) fData[ 2 ], (pdINT) fData[ 3 ], (pdINT) fData[ 4 ], (pdINT) fData[ 5 ] );
}


/* Write Low Switch Gradient to SCM */
void clsSetLowSwitchGradient( pdFLOAT* fData, pdINT iStartAxis, pdINT iNoOfAxes )
{
	msgSize = iCreateDataTransferString( SCM_NODE, iStartAxis, iNoOfAxes, msgLOW_SWITCH_GRADIENT, 0, fData, txmsg, 512 );
	sendto( cls_socket, txmsg, msgSize, 0, (struct sockaddr*) &txpeer, sizeof( txpeer ) );

}

void clsSleep(std::chrono::milliseconds sleep_time)
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
