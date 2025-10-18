#ifndef __CLS_UTIL__
#define __CLS_UTIL__

#include "WAS_Comms_Api.h"
#include "cls_config.h"
#include <chrono>
#include <thread>
#include <array>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>


enum class CLSStatus
{
	UNINITIALIZED = 0,
	INITIALIZING  = 1,
	PASSIVE       = 2,
	ACTIVE        = 3,
	UNRESPONSIVE  = 4,
	TERMINATED    = 5
};

struct CLSmsg
{
	pdINT      messageType = -1;
	pdINT      tag = 0;
	pdINT      startAxis = 0;
	pdINT      numberOfAxes = 0;
	pdINT      numberOfDataValues = 0;
	std::array<pdFLOAT, MAX_AXES> data{};    // fixed-size, zero-initialized
};

enum class CLSMessageCode
{
		// Invalid value
		msgNO_MESSAGE                     = 0,   
    msgGO_ACTIVE                      = 1,   // Move from passive to active mode. Held pending if not yet initialised.
    msgSTART_TRAVERSAL                = 2,   // Move from uninitialised to initialising mode.
    msgNEW_MASTER_CURVE               = 3,   // Accept a new master curve file definition.
    msgSELECT_MASTER_CURVE            = 4,   // Select a previously defined master curve.
    msgTRIM_VALUE                     = 6,
    msgQFEEL                          = 7,
    msgNEW_ADDIN_CURVE                = 8,   // Accept a new addin curve file definition.
    msgTRIM_RATE                      = 9,   // Maximum trim rate in units per second.
    msgSAVE_CONFIG                    = 11,  // Save the configuration currently in use to the SCM flash disk.
    msgGO_PASSIVE                     = 12,  // Move from an active state to a passive state.
    msgPOS_END_STOP                   = 13,  // Positive end stop position.
    msgNEG_END_STOP                   = 14,  // Negative end stop position.
    msgSELECT_ADDIN                   = 15,  // Select a previously defined addin.
    msgADDIN_AMPLITUDE                = 17,  // The amplitude of the addin.
    msgDATUM_POSITION                 = 18,  // The position of the datum (0 reference).
    msgSTICK_SHAKE_HZ                 = 20,  // Stick Shake frequency in Hz.
    msgSTATIC_FRICTION                = 21,  // Must be >= dynamic friction.
    msgDYNAMIC_FRICTION               = 22,  // Must be <= static friction.
    msgSTICK_SHAKE_AMP                = 23,  // Stick Shake amplitude in position units.
    msgNUMBER_OF_AXIS                 = 24,  // The number of axes being controlled.
    msgFORCES                         = 25,  // Force input request.
    msgPOSITIONS                      = 26,  // Position input request.
    msgMODEL_DAMPING                  = 27,
    msgMODEL_HZ                       = 28,
    msgMASTER_PHASE_IN_TIME           = 29,  // Time (s) to phase from one master curve to another.
    msgADDIN_PHASE_IN_TIME            = 30,  // Time (s) to phase from one addin curve to another.
    msgADDIN_START                    = 31,  // Start of the addin (numerically lower extreme).
    msgADDIN_END                      = 32,  // End of the addin (numerically higher extreme).
    msgADDIN_ABQF                     = 33,  // Addin affected by qfeel.
    msgADDIN_TTD                      = 34,  // Addin tied to datum.
    msgFORCE_CALICBRATION_FACTOR      = 35,  // Volts to Newtons scaler.
    msgAXIS_TO_LINK_TO                = 36,  // Set to -1 if not linked.
    msgMAX_QFEEL_CHANGE_RATE          = 37,  // Max QFeel rate change.
    msgMAX_ADDIN_AMP_CHANGE_RATE      = 38,  // Max addin amplitude change rate.
    msgNEGATIVE_QFEEL                 = 42,  // QFeel on the negative side of the datum.
    msgCOMMS_HEART_BEAT               = 45,  // Healthy communications link signal.
    msgRESTORE_CONFIG                 = 47,  // Restore configuration from SCM flash disk.
    msgTRIM_RELEASE_STATE             = 50,  // Enables trim release from Toolkit.
    msgSTATUS                         = 51,  // Request current status.
    msgCAN_IO_MESSAGE                 = 52,  // Pass externally generated CANBus messages.
    msgESTABLISH_LINK_FOR_ASYCH_COMMS = 53,  // For linking async CANBUS_MESSAGE_COPY.
    msgAUTOPILOT_MODE                 = 57,  // Either position (2) or off (0).
    msgAUTOPILOT_VALUE                = 58,  // Autopilot position value.
    msgSTATIC_FRICTION_IN_TRIM_RELEASE = 70,
    msgDYNAMIC_FRICTION_IN_TRIM_RELEASE = 71,
    msgSTICK_SHAKE_ENABLE             = 81,  // Boolean flag to enable/disable stick shake.
    msgFORCE_BIAS                     = 89,  // Added to pilot input force (not cross linked).
    msgLOW_SWITCH_GRADIENT            = 113,
    msgSCM_HEARTBEAT                  = 150,
    msgGATE_1_ADDIN                   = 151,
    msgGATE_1_POSITION                = 152,
    msgGATE_1_RESTORE_MARGIN          = 153,
    msgGATE_1_FORCE_THRESHOLD         = 154,
    msgGATE_2_ADDIN                   = 157,
    msgGATE_2_POSITION                = 158,
    msgGATE_2_RESTORE_MARGIN          = 159,
    msgGATE_2_FORCE_THRESHOLD         = 160,
    msgGATE_1_ACTIVE                  = 163,
    msgGATE_1_RELEASE_BUTTON          = 164,
    msgGATE_1_BUTTON_ACTIVE           = 165,
    msgGATE_2_ACTIVE                  = 166,
    msgGATE_2_RELEASE_BUTTON          = 167,
    msgGATE_2_BUTTON_ACTIVE           = 168,
    msgMOTOR_ERROR_STATUS             = 177,
    msgDAMPING_FACTOR                 = 181,
    msgEND_STOP_TORQUE_GAIN           = 184,
    msgIMPULSE_AMPLITUDE              = 186, // Mark-space ratio impulse input.
    msgIMPULSE_FREQUENCY              = 187, // Mark-space ratio impulse input.
    msgMARK_SPACE_RATIO               = 188,
    msgZERO_LEVEL_OFFSET              = 189  // Mark-space ratio impulse input offset.
};

enum class CLSStatusCode
{
	clsSTATUS_UNINITIALIZED             = ((unsigned pdLONG) 0x001),
	clsSTATUS_INITIALIZING              = ((unsigned pdLONG) 0x002),
	clsSTATUS_PASSIVE                   = ((unsigned pdLONG) 0x004),
	clsSTATUS_ACTIVE                    = ((unsigned pdLONG) 0x008),
	clsSTATUS_CAN_MOTOR_MESSAGE_FAILED  = ((unsigned pdLONG) 0x010),
	clsSTATUS_STATUS_TERMINATED         = ((unsigned pdLONG) 0x040)
};

class CLSInterface
{
	private:
		int cls_socket = -1;
		unsigned int scm_port = DEFAULT_SCM_PORT;
		unsigned int rx_port = DEFAULT_SCM_PORT;
		std::string cls_address = DEFAULT_SCM_ADDRESS;
		struct sockaddr_in txpeer;
		struct sockaddr_in rxpeer;
		socklen_t rxpeerlen;
		char txmsg[MAX_SIZE_TX];
		char rxmsg[MAX_SIZE_RX];
		int BufLen;
		int msxSize;
		float txdata[MAX_AXES];
		float rxdata[MAX_AXES];

    bool extractDataValues(pdINT &numValues, std::array<pdFLOAT, MAX_AXES> &outBuffer, const char *cBuf);
		bool decodeDataParameters(const pdCHAR *cBuf, pdINT iEnquiry, CLSmsg &msg);

	protected:
		void CommsThread(void);

		void useStatus(pdFLOAT* fData);
		void useForces(pdFLOAT* fData);
		void usePositions(pdFLOAT* fData);
		void useLowSwitchGradient(pdFLOAT* fData);
    void setLowSwitchGradient(pdFLOAT *fData, pdINT iStartAxis, pdINT iNoOfAxes);

    void useIO(pdFLOAT *fData, pdINT iBUS);

  public:

		CLSInterface(void);
		CLSInterface(const std::string&, const unsigned int, const unsigned int);
  	
  	void setSCMport(pdUINT);
  	void setSCMaddress();
    void initUDPSocket(void);
		void closeSocket(void);

		void requestCLSStatus(void);

		void clsTraverseAll(void);
		void clsActivateAll(void);
		void clsDeactivateAll(void);
		void clsRestore(pdINT iLocation);

		void clsRequest(CLSMessageCode msgcode);

		void clsSleep(std::chrono::milliseconds sleep_time);
};




#endif
