/* Required includes */
#include <stdio.h>

/* Macros */
#define ASCII_HEX_TO_INT( c ) ( int ) ( ( c >= '0' && c <= '9' ) ? ( c - 48 ) : ( c - 87 ) )

/* UDP message processing/construction */
#define prtclLEN_TAKEN_BY_ONE_DATA_VALUE 7
#define prtclFIRST_DATA_TYPE_BYTE_OFFSET 2
#define prtclSECOND_DATA_TYPE_BYTE_OFFSET 3
#define prtclNODE_OFFSET 1
#define prtclTAG_OFFSET 4
#define prtclSTART_AXIS_OFFSET 5
#define prtclNUMBER_OF_AXIS_OFFSET 6
#define prtclSTART_OF_DATA_OFFSET 7

#define STX	2
#define EOT	4
#define ENQ 5

#define MAX_SIZE_TX							512
#define MAX_SIZE_RX							512
#define pdMAX_DATA_VALUES_IN_ONE_MESSAGE	30
#define MAX_AXES							20
#define MAX_CAN_IO							992		/* 16 per card, 31 cards, 2 busses */

#define QUERY_MESSAGE	pdTRUE
#define DATA_MESSAGE	pdFALSE

// Predefines
#define pdPASS	0
#define pdFAIL	1
#define pdTRUE	1
#define pdFALSE	0

#define pdINT		  int  				/* Integers should be defined as 32 bit. */
#define pdUINT    uint32_t
#define pdCHAR		char				/* Char should be defined as 8 bit. */
#define pdLONG		long                /* Long should be defined as 32 bit. */ 
#define pdFLOAT		float				/* Float should be defined as single precision. */
#define pdSHORT		short				/* Short should be defined as 16 bi6. */
#define pdDOUBLE	double

#define POSITIVE 1
#define NEGATIVE -1

/* CAN IO Definitions - Note it is possible to have the same card address on each bus - the TAG holds the bus number */
#define canFIRST_CAN_IO_CARD	0x3E0
#define canLAST_CAN_IO_CARD		0x3FF

#define canBUTTONS_PER_CARD		16
#define canANALOGS_PER_CARD		2

#define canCARD_0_BUTTON_0		0	/* Rename to be appropriate or useful */
#define canCARD_0_BUTTON_1		1

/* CLS data codes */
// #define msgNO_MESSAGE						0	/* Invalid value. */
// #define msgGO_ACTIVE						1	/* Move from passive to active mode.  Held pending if not yet initialised. */
// #define msgSTART_TRAVERSAL					2	/* Move from uninitialised to initialising mode. */
// #define msgNEW_MASTER_CURVE					3	/* Accept a new master curve file definition. */
// #define msgSELECT_MASTER_CURVE				4	/* Select a previously defined master curve. */
// #define msgTRIM_VALUE						6	
// #define msgQFEEL							7	
// #define msgNEW_ADDIN_CURVE					8   /* Accept a new addin curve file definition. */
// #define msgTRIM_RATE						9	/* Maximum trim rate in units per second. */
// #define msgSAVE_CONFIG						11	/* Save the configuration currently in use to the SCM flash disk. */
// #define msgGO_PASSIVE						12	/* Move from an active state to a passive state. */
// #define msgPOS_END_STOP						13	/* Positive end stop position - must be specified as a signed number & greater than negative endstop. */
// #define msgNEG_END_STOP						14	/* Negative end stop position - must be specified as a signed number & less than positive endstop. */
// #define msgSELECT_ADDIN						15	/* Select a previously defined addin. */
// #define msgADDIN_AMPLITUDE					17	/* The amplitude of the addin.  If the addin is tied to the datum then the amplitude is applied on both sides of the datum. */
// #define msgDATUM_POSITION					18	/* The position of the datum (0 reference) as opposed to the trim. */
// #define msgSTICK_SHAKE_HZ					20  /* Stick Shake frequency in Hz. */
// #define msgSTATIC_FRICTION					21	/* Must be higher than (or equal to) dynamic friction. */
// #define msgDYNAMIC_FRICTION					22	/* Must be lower than (or equal to) static friction. */
// #define msgSTICK_SHAKE_AMP					23	/* Stick Shake amplitude in position units. */
// #define msgNUMBER_OF_AXIS					24	/* The number of axis being controlled. */
// #define msgFORCES							25	/* Force input request.  This is the combined force after linking and filtering. */
// #define msgPOSITIONS						26	/* Position input request. */
// #define msgMODEL_DAMPING					27	
// #define msgMODEL_HZ							28	
// #define msgMASTER_PHASE_IN_TIME				29  /* The time in seconds taken to phase from one master curve to another. */
// #define msgADDIN_PHASE_IN_TIME				30	/* The time in seconds taken to phase from one addin curve to another. */
// #define msgADDIN_START						31  /* The start of the addin.  This is the numerically lower extreme of its position which ever side of the datum the addin is on. */
// #define msgADDIN_END						32	/* The end of the addin.  This is the numerically higher extreme of its position which ever side of the datum the addin is on. */
// #define msgADDIN_ABQF						33	/* Addin affected by qfeel. */
// #define msgADDIN_TTD						34	/* Addin Tied to datum. */
// #define msgFORCE_CALICBRATION_FACTOR		35	/* Volts to Newtons scaler. */
// #define msgAXIS_TO_LINK_TO					36	/* Set to -1 if not linked to any other axis. */
// #define msgMAX_QFEEL_CHANGE_RATE			37	/* Maximum rate in units per second the QFeel can change at. */
// #define msgMAX_ADDIN_AMP_CHANGE_RATE		38	/* Maximum rate in units per second the ampliutde of an addin can change at.  This is also used when an addin is deselected as it is phased to 0 amplitude. */
// #define msgNEGATIVE_QFEEL					42  /* The qfeel to use on the negative side of the datum. */
// #define msgCOMMS_HEART_BEAT					45	/* Used to signal the existance of a healthy communications link. */
// #define msgRESTORE_CONFIG					47  /* Restore a configuration from the flash disk of the SCM to being the configuration currently in use. */
// #define msgTRIM_RELEASE_STATE				50	/* Enables the trim release to be enabled from the Toolkit. */
// #define msgSTATUS							51	/* Request the current status (i.e. active, passive, initialising, etc.). */
// #define msgCAN_IO_MESSAGE					52	/* Used to pass externally generated CANBus messages up to the host system. */
// #define msgESTABLISH_LINK_FOR_ASYCH_COMMS	53	/* Goes with msgCANBUS_MESSAGE_COPY.  As msgCANBUS_MESSAGE_COPY messages are asynchronous this message has to be sent first so the destination of the msgCANBUS_MESSAGE_COPY is known. */
// #define msgAUTOPILOT_MODE					57	/* Either position (2)  or off (0). */
// #define msgAUTOPILOT_VALUE					58	/* The position value used for the autopilot. */
// #define msgSTATIC_FRICTION_IN_TRIM_RELEASE	70  
// #define msgDYNAMIC_FRICTION_IN_TRIM_RELEASE	71	
// #define msgSTICK_SHAKE_ENABLE				81	/* Boolean flag to turn on or off the stick shake without having to modify the frequency of amplitude of the shake. */
// #define msgFORCE_BIAS						89	/* Simply added to the pilot input force.  This is not cross linked. */
// #define msgLOW_SWITCH_GRADIENT              113
// #define msgSCM_HEARTBEAT					150
// #define msgGATE_1_ADDIN						151
// #define msgGATE_1_POSITION					152
// #define msgGATE_1_RESTORE_MARGIN			153
// #define msgGATE_1_FORCE_THRESHOLD			154
// #define msgGATE_2_ADDIN						157	
// #define msgGATE_2_POSITION					158	
// #define msgGATE_2_RESTORE_MARGIN			159	
// #define msgGATE_2_FORCE_THRESHOLD			160	
// #define msgGATE_1_ACTIVE					163	
// #define msgGATE_1_RELEASE_BUTTON			164	
// #define msgGATE_1_BUTTON_ACTIVE				165	
// #define msgGATE_2_ACTIVE					166	
// #define msgGATE_2_RELEASE_BUTTON			167	
// #define msgGATE_2_BUTTON_ACTIVE				168	
// #define msgMOTOR_ERROR_STATUS				177 
// #define msgDAMPING_FACTOR					181 
// #define msgEND_STOP_TORQUE_GAIN				184 
// #define msgIMPULSE_AMPLITUDE				186 /* Mark space ratio impulse input. */
// #define msgIMPULSE_FREQUENCY				187 /* Mark space ratio impulse input. */
// #define msgMARK_SPACE_RATIO					188 
// #define msgZERO_LEVEL_OFFSET				189 /* Mark space ratio impulse input offset */

/* System status bit definitions. */
// #define clsSTATUS_UNINITIALISED						( ( unsigned pdLONG ) 0x0001 )
// #define clsSTATUS_INITIALISING						( ( unsigned pdLONG ) 0x0002 )
// #define clsSTATUS_PASSIVE							( ( unsigned pdLONG ) 0x0004 )
// #define clsSTATUS_ACTIVE							( ( unsigned pdLONG ) 0x0008 )
// #define clsSTATUS_CAN_MOTOR_MESSAGE_FAILED			( ( unsigned pdLONG ) 0x0010 )
// #define clspdSTATUS_TERMINATED						( ( unsigned pdLONG ) 0x0040 )


/* Public Declarations */
pdINT prvGenerateQuery( pdCHAR *cBuf, pdINT iStartAxis, pdINT iNumberOfAxis, pdINT iFuncCode, pdINT iTag, pdINT iNode );
pdINT prvDecodeDataParameters( pdCHAR *cBuf, pdINT *iMessageType, pdINT *iTag, pdINT *iStartAxis, pdINT *iNumberOfAxis, pdFLOAT *fBuffer, pdINT iEnquiry, pdINT *iNumberOfDataValues );
pdINT prvExtractDataValues( pdINT *iNumberOfDataValues, pdFLOAT *fBuffer, pdCHAR *cBuf );
pdINT iCreateDataTransferString(pdINT iNode, pdINT iStartAxis, pdINT iNumOfAxis, pdINT iFuncCode, pdINT iTag, pdFLOAT *fBuf, pdCHAR *cBuf, pdINT iBufferSize );
