#define _CRT_SECURE_NO_WARNINGS

#include <string.h>
#include "WAS_Comms_Api.h"

/* WA&S Protocol Functions */
/*  These functions are those shown in the protocol document */

static unsigned pdSHORT prvGenerateCRC( pdCHAR * cBuf, pdINT iSize )
{
	unsigned pdSHORT usCRC, usLSB, i, j;
	pdCHAR *c;
	
	/* Generate CRC from data in cBuf. */
	usCRC=0xffff;

	/* For each byte in the buffer. */
	for( i = 0 ; i < iSize; i++ )
	{
		usCRC ^= *( cBuf + i );
		/* For each bit in the byte. */
		
		for( j = 0; j < 8; j++ )
		{
			usLSB = ( usCRC & 0x0001 );
			usCRC >>= 1;
			if( usLSB )
				usCRC ^= 0xa001;
		}
	}
	/* Often code for proessing ASCII protocol includes C string handling functions. These
	may cause errors to occur if the checksum bytes contain a zero (C string terminator). If either
	byte is zero, set the byte to 'x' */
	c = ( pdCHAR * ) &usCRC;
	if( c[ 0 ] == 0x00 )
	c[ 0 ] = 'x';
	if( c[ 1 ] == 0x00 )
	c[ 1 ] = 'x';
	return usCRC;
}

pdINT iCreateDataTransferString(pdINT iNode, pdINT iStartAxis, pdINT iNumOfAxis, pdINT iFuncCode, pdINT iTag, pdFLOAT *fBuf, pdCHAR *cBuf, pdINT iBufferSize )
{
	pdINT iLoop; /* Loop counter only. */
//	char cCRC[5]; /* Space in which the checksum string is created before being added to the main buffer. */
	pdINT iBufPos; /* Index into buffer of position into which the next character will be written. */
	unsigned pdSHORT usCRC;
	pdINT iMsgSize = -1;

	/* Take the floats from fBuf and format them into a message for transmission. */
	/* Create the 'header' of the buffer - up to the point where the data starts. */
	sprintf( cBuf, "%c%x%02x%x%x%x", STX, iNode, iFuncCode, iTag, iStartAxis, iNumOfAxis );

	/* Note the position of the end of the buffer so far. */
	iBufPos = strlen( cBuf );

	/* The header has now been created. Extend the string with each data value in turn. */
	for( iLoop = 0; iLoop < iNumOfAxis; iLoop++ )
	{
		/* Write the value to the buffer in the correct format. The number of decmimal places will depend on the
		magnatude of the number. */
		sprintf( cBuf + iBufPos, "%+01.04f", fBuf[ iLoop ] );

		/* We have just increased the length of the string, note the new position of the
		end of the buffer. */
		iBufPos += prtclLEN_TAKEN_BY_ONE_DATA_VALUE;
	}
	/* All the parameters have been added to the string, add the end of transmission character. */
	sprintf( cBuf + iBufPos, "%c", EOT );

	/* Increase the counter to the next free position past the EOT just added. */
	iBufPos++;

	/* Generate the checksum string from the Tx string. */
	usCRC = prvGenerateCRC( cBuf, iBufPos );
	sprintf( cBuf + iBufPos, "%c%c", (unsigned pdCHAR)( ( usCRC & 0xff00 ) >> 8 ) /* most sig byte */,
		(unsigned pdCHAR)( usCRC & 0x00ff ) /* Least sig byte */ );

	/* Include the checksum characters in the message length (+2). */
	iMsgSize = iBufPos + 2;

	return iMsgSize;
}

pdINT prvExtractDataValues( pdINT *iNumberOfDataValues, pdFLOAT *fBuffer, pdCHAR *cBuf )
{
	pdINT iChannel, iSign, iBufPos;
	pdINT iRet = pdTRUE;
	pdFLOAT fVal;
	/* A list of data values is contained within cBuf. Loop through each data value, decode into a float, and place
	in fBuffer. We know when we have come to the end of the data values when an EOT is found. */

	/* We have not yet found any data values. //correction */
	*iNumberOfDataValues = 0;

	/* We are looking for the first data value. The name iChannel is used instead of iAxis as the first data value is not necessarliy for axis 0. */
	iChannel = 0;

	/* The position within cBuffer that we are going to read the next string from. Start at the beginning – position 0. */
	iBufPos = 0;

	/* Make sure we do not go off the end of the array. */
	while( iChannel < pdMAX_DATA_VALUES_IN_ONE_MESSAGE )
		{
			/* Jump on to start of next data value. Each data value takes prtclLEN_TAKEN_BY_ONE_DATA_VALUE bytes. */
			iBufPos = iChannel * prtclLEN_TAKEN_BY_ONE_DATA_VALUE;
			
			/* Check we have not come to the end of the data. If an EOT is found then there is no more data passed this point and we can leave the loop. */
			if( cBuf[ iBufPos ] == EOT )
			{
				break;
			}
			/* The first character is the sign. Store the sign of the data for later use. */
			iSign = ( cBuf[ iBufPos ] == '+' ) ? 1 : -1;

			/* Read the next float out of the buffer + 1 to skip over sign character. */
			sscanf( cBuf + iBufPos + 1, "%f", &fVal );
			
			/* Adjust for the sign of the value and store the value in the next buffer position. */
			fBuffer[ iChannel ] = fVal * iSign;
			
			/* Move onto the next buffer position. */
			iChannel++;
	}

	/* Did we find the end of the data? If we gone past the end of the array then we did not. */
	if( iChannel >= pdMAX_DATA_VALUES_IN_ONE_MESSAGE )
	{
		iRet = pdFALSE;
	}
	else
	{
		*iNumberOfDataValues = iChannel;
	}
	return iRet;
}

pdINT prvGenerateQuery( pdCHAR *cBuf, pdINT iStartAxis, pdINT iNumberOfAxis, pdINT iFuncCode, pdINT iTag, pdINT iNode )
{
	pdCHAR cCRC[ 3 ];
	pdINT iMsgLen = 0;
	unsigned pdSHORT usCRC;

	/* Create a query message in the protocol format from the values passed in. Place the message in cBuf. */
	/* Create the Tx String excluding the checksum. */
	sprintf( cBuf, "%c%x%02x%x%x%x%c", STX, iNode, iFuncCode, iTag, iStartAxis, iNumberOfAxis, ENQ );

	/* Generate the checksum string from the Tx string. */
	usCRC = prvGenerateCRC( cBuf, strlen( cBuf ) );

	/* Create a string containing the checksum. */
	sprintf( cCRC, "%c%c", (unsigned pdCHAR)( ( usCRC & 0xff00 ) >> 8 ) /* most sig byte */,
		(unsigned pdCHAR)( usCRC & 0x00ff ) /* Least sig byte */ );

	/* Add the checksum string to the end of the buffer. */
	strcat( cBuf, cCRC );

	/* Return the length of the created string. */
	iMsgLen = strlen( cBuf );

	return iMsgLen;
}
