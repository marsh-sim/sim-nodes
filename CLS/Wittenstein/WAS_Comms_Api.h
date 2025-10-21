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


/* Public Declarations */
pdINT prvGenerateQuery( pdCHAR *cBuf, pdINT iStartAxis, pdINT iNumberOfAxis, pdINT iFuncCode, pdINT iTag, pdINT iNode );
pdINT prvDecodeDataParameters( pdCHAR *cBuf, pdINT *iMessageType, pdINT *iTag, pdINT *iStartAxis, pdINT *iNumberOfAxis, pdFLOAT *fBuffer, pdINT iEnquiry, pdINT *iNumberOfDataValues );
pdINT prvExtractDataValues( pdINT *iNumberOfDataValues, pdFLOAT *fBuffer, pdCHAR *cBuf );
pdINT iCreateDataTransferString(pdINT iNode, pdINT iStartAxis, pdINT iNumOfAxis, pdINT iFuncCode, pdINT iTag, pdFLOAT *fBuf, pdCHAR *cBuf, pdINT iBufferSize );
