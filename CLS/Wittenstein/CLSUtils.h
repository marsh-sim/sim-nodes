#ifndef __CLS_UTIL__
#define __CLS_UTIL__

#include "WAS_Comms_Api.h"
#include "cls_config.h"
#include <chrono>
#include <thread>
#include <array>
#include <string>
#include <cstring>
#include <sstream>
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

enum class CLSMessageCode
{
		// Invalid value
		NO_MESSAGE                       = 0,   
    GO_ACTIVE                        = 1,   // Move from passive to active mode. Held pending if not yet initialised.
    START_TRAVERSAL                  = 2,   // Move from uninitialised to initialising mode.
    NEW_MASTER_CURVE                 = 3,   // Accept a new master curve file definition.
    SELECT_MASTER_CURVE              = 4,   // Select a previously defined master curve.
    TRIM_VALUE                       = 6,
    QFEEL                            = 7,
    NEW_ADDIN_CURVE                  = 8,   // Accept a new addin curve file definition.
    TRIM_RATE                        = 9,   // Maximum trim rate in units per second.
    SAVE_CONFIG                      = 11,  // Save the configuration currently in use to the SCM flash disk.
    GO_PASSIVE                       = 12,  // Move from an active state to a passive state.
    POS_END_STOP                     = 13,  // Positive end stop position.
    NEG_END_STOP                     = 14,  // Negative end stop position.
    SELECT_ADDIN                     = 15,  // Select a previously defined addin.
    ADDIN_AMPLITUDE                  = 17,  // The amplitude of the addin.
    DATUM_POSITION                   = 18,  // The position of the datum (0 reference).
    STICK_SHAKE_HZ                   = 20,  // Stick Shake frequency in Hz.
    STATIC_FRICTION                  = 21,  // Must be >= dynamic friction.
    DYNAMIC_FRICTION                 = 22,  // Must be <= static friction.
    STICK_SHAKE_AMP                  = 23,  // Stick Shake amplitude in position units.
    NUMBER_OF_AXIS                   = 24,  // The number of axes being controlled.
    FORCES                           = 25,  // Force input request.
    POSITIONS                        = 26,  // Position input request.
    MODEL_DAMPING                    = 27,
    MODEL_HZ                         = 28,
    MASTER_PHASE_IN_TIME             = 29,  // Time (s) to phase from one master curve to another.
    ADDIN_PHASE_IN_TIME              = 30,  // Time (s) to phase from one addin curve to another.
    ADDIN_START                      = 31,  // Start of the addin (numerically lower extreme).
    ADDIN_END                        = 32,  // End of the addin (numerically higher extreme).
    ADDIN_ABQF                       = 33,  // Addin affected by qfeel.
    ADDIN_TTD                        = 34,  // Addin tied to datum.
    FORCE_CALIBRATION_FACTOR         = 35,  // Volts to Newtons scaler.
    AXIS_TO_LINK_TO                  = 36,  // Set to -1 if not linked.
    MAX_QFEEL_CHANGE_RATE            = 37,  // Max QFeel rate change.
    MAX_ADDIN_AMP_CHANGE_RATE        = 38,  // Max addin amplitude change rate.
    NEGATIVE_QFEEL                   = 42,  // QFeel on the negative side of the datum.
    COMMS_HEART_BEAT                 = 45,  // Healthy communications link signal.
    RESTORE_CONFIG                   = 47,  // Restore configuration from SCM flash disk.
    TRIM_RELEASE_STATE               = 50,  // Enables trim release from Toolkit.
    STATUS                           = 51,  // Request current status.
    CAN_IO_MESSAGE                   = 52,  // Pass externally generated CANBus messages.
    ESTABLISH_LINK_FOR_ASYCH_COMMS   = 53,  // For linking async CANBUS_MESSAGE_COPY.
    AUTOPILOT_MODE                   = 57,  // Either position (2) or off (0).
    AUTOPILOT_VALUE                  = 58,  // Autopilot position value.
    STATIC_FRICTION_IN_TRIM_RELEASE  = 70,
    DYNAMIC_FRICTION_IN_TRIM_RELEASE = 71,
    STICK_SHAKE_ENABLE               = 81,  // Boolean flag to enable/disable stick shake.
    FORCE_BIAS                       = 89,  // Added to pilot input force (not cross linked).
    LOW_SWITCH_GRADIENT              = 113,
    SCM_HEARTBEAT                    = 150,
    GATE_1_ADDIN                     = 151,
    GATE_1_POSITION                  = 152,
    GATE_1_RESTORE_MARGIN            = 153,
    GATE_1_FORCE_THRESHOLD           = 154,
    GATE_2_ADDIN                     = 157,
    GATE_2_POSITION                  = 158,
    GATE_2_RESTORE_MARGIN            = 159,
    GATE_2_FORCE_THRESHOLD           = 160,
    GATE_1_ACTIVE                    = 163,
    GATE_1_RELEASE_BUTTON            = 164,
    GATE_1_BUTTON_ACTIVE             = 165,
    GATE_2_ACTIVE                    = 166,
    GATE_2_RELEASE_BUTTON            = 167,
    GATE_2_BUTTON_ACTIVE             = 168,
    MOTOR_ERROR_STATUS               = 177,
    DAMPING_FACTOR                   = 181,
    END_STOP_TORQUE_GAIN             = 184,
    IMPULSE_AMPLITUDE                = 186, // Mark-space ratio impulse input.
    IMPULSE_FREQUENCY                = 187, // Mark-space ratio impulse input.
    MARK_SPACE_RATIO                 = 188,
    ZERO_LEVEL_OFFSET                = 189,  // Mark-space ratio impulse input offset.
    UNKNOWN                          = 999
};

struct CLSmsg
{
	CLSMessageCode messageType;
	pdINT          tag = 0;
	pdINT          startAxis = 0;
	pdINT          numberOfAxes = 0;
	pdINT          numberOfDataValues = 0;
	std::array<pdFLOAT, MAX_AXES> data{};    // fixed-size, zero-initialized
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
		unsigned int scm_node = DEFAULT_SCM_NODE;
		struct sockaddr_in txpeer;
		struct sockaddr_in rxpeer;
		socklen_t rxpeerlen;
		char txmsg[MAX_SIZE_TX];
		char rxmsg[MAX_SIZE_RX];
		int BufLen;
		int msgSize;
		float txdata[MAX_AXES];
		float rxdata[MAX_AXES];

		bool socketInitialized;
		std::string lastError;

		unsigned pdSHORT generateCRC(const pdCHAR* cBuf, pdINT iSize);
		pdINT generateQuery(pdCHAR* cBuf, pdINT iBufSize, pdINT iStartAxis,  pdINT iNumberOfAxis, pdINT iFuncCode, pdINT iTag, pdINT iNode);

    bool extractDataValues(pdINT &numValues, std::array<pdFLOAT, MAX_AXES> &outBuffer, const char *cBuf);
		bool decodeDataParameters(const pdCHAR *cBuf, pdINT iEnquiry, CLSmsg &msg);
		pdINT createDataTransferString(pdINT iNode, pdINT iStartAxis, pdINT iNumOfAxis, pdINT iFuncCode, pdINT iTag, const pdFLOAT *fBuf, pdCHAR *cBuf, pdINT iBufferSize);

		
	protected:
    int initUDPSocket(void);
		void CommsThread(void);

		void useStatus(const pdFLOAT* fData);
		void useForces(const pdFLOAT* fData);
		void usePositions(const pdFLOAT* fData);
		void useLowSwitchGradient(const pdFLOAT* fData);

    void useIO(const pdFLOAT *fData, const pdINT iBUS);

  public:

		CLSInterface()
		:socketInitialized(false)
		{
			// minimal constructor
		};

		bool initialize(void);
		bool isInitialized() const { return socketInitialized; }
    std::string getLastError() const { return lastError; }

		CLSInterface(const std::string&, const unsigned int, const unsigned int);
  	
  	void setSCMport(pdUINT);
  	void setSCMaddress();
		void closeSocket(void);

		void requestCLSStatus(void);

		void traverseAll(void);
		void activateAll(void);
		void deactivateAll(void);
		void restore(pdINT iLocation);

		void request(CLSMessageCode msgcode);
    void setLowSwitchGradient(pdFLOAT *fData, pdINT iStartAxis, pdINT iNoOfAxes);

		void sleep(std::chrono::milliseconds sleep_time);
};




#endif
