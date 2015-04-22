/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:Loovee
  2012-4-24
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"
#define MAX_CHAR_IN_MESSAGE 8

//each frame in one of these buffers is 16 bytes so take it easy turbo. 2K of RAM is pitiful.
#define SIZE_RX_BUFFER	8 //RX incoming ring buffer is this big
#define SIZE_TX_BUFFER	4 //TX ring buffer is this big

typedef union {
    uint64_t value;
	struct {
		uint32_t low;
		uint32_t high;
	};
	struct {
        uint16_t s0;
		uint16_t s1;
		uint16_t s2;
		uint16_t s3;
    };
	uint8_t bytes[8];
	uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;

typedef struct
{
	uint32_t id;		// EID if ide set, SID otherwise
	uint8_t rtr;		// Remote Transmission Request
	uint8_t priority;	// Priority but only important for TX frames and then only for special uses.
	uint8_t extended;	// Extended ID flag
	uint8_t length;		// Number of data bytes
	BytesUnion data;	// 64 bits - lots of ways to access it.
} CAN_FRAME;

class MCP_CAN
{
    private:
    
    INT8U   m_nExtFlg;                                                  /* identifier xxxID             */
                                                                        /* either extended (the 29 LSB) */
                                                                        /* or standard (the 11 LSB)     */
    INT32U  m_nID;                                                      /* can id                       */
    INT8U   m_nDlc;                                                     /* data length:                 */
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                            	/* data                         */
    INT8U   m_nRtr;                                                     /* rtr                          */
    INT8U   m_nfilhit;

	volatile CAN_FRAME rx_frames[SIZE_RX_BUFFER];
	volatile CAN_FRAME tx_frames[SIZE_TX_BUFFER];
	volatile uint8_t rx_frame_write_pos, rx_frame_read_pos;
	volatile uint8_t tx_frame_write_pos, tx_frame_read_pos;


/*
*  mcp2515 driver function 
*/
   // private:
   private:

    void mcp2515_reset(void);                                           /* reset mcp2515                */

    INT8U mcp2515_readRegister(const INT8U address);                    /* read mcp2515's register      */
    
    void mcp2515_readRegisterS(const INT8U address, 
	                       INT8U values[], 
                               const INT8U n);
    void mcp2515_setRegister(const INT8U address,                       /* set mcp2515's register       */
                             const INT8U value);

    void mcp2515_setRegisterS(const INT8U address,                      /* set mcp2515's registers      */
                              const INT8U values[],
                              const INT8U n);
    
    void mcp2515_initCANBuffers(void);
    
    void mcp2515_modifyRegister(const INT8U address,                    /* set bit of one register      */
                                const INT8U mask,
                                const INT8U data);

    INT8U mcp2515_readStatus(void);                                     /* read mcp2515's Status        */
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);                 /* set mode                     */
    INT8U mcp2515_configRate(const INT8U canSpeed);                     /* set boadrate                 */
    INT8U mcp2515_init(const INT8U canSpeed);                           /* mcp2515init                  */

    void mcp2515_write_id( const INT8U mcp_addr,                        /* write can id                 */
                               const INT8U ext,
                               const INT32U id );

    void mcp2515_read_id( const INT8U mcp_addr,                        /* read can id                  */
                                    INT8U* ext,
                                    INT32U* id );

    void mcp2515_write_canMsg( const INT8U buffer_sidh_addr );          /* write can msg                */
    void mcp2515_read_canMsg( const INT8U buffer_sidh_addr);            /* read can msg                 */
    void mcp2515_start_transmit(const INT8U mcp_addr);                  /* start transmit      */      
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);                     /* get Next free txbuf          */
	void EnqueueRX(CAN_FRAME& newFrame);

/*
*  can operator function
*/    

    INT8U setMsg(INT32U id, INT8U ext, INT8U len, INT8U *pData);    /* set message                  */  
    INT8U clearMsg();                                               /* clear all message to zero    */
    INT8U readMsg();                                                /* read message                 */
    INT8U sendMsg();                                                /* send message                 */
public:
    INT8U begin(INT8U speedset);                              /* init can                     */
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);           /* init Masks                   */
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);           /* init filters                 */
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);  /* send buf                     */
    INT8U readMsgBuf(INT8U *len, INT8U *buf);                       /* read buf                     */
	INT8U sendFrame(CAN_FRAME &frame);
	INT8U receiveFrame(CAN_FRAME &frame);
    INT8U checkReceive(void);                                       /* if something received        */
    INT8U checkError(void);                                         /* if something error           */
    INT32U getCanId(void);                                          /* get can id when receive      */
	void handleInt();
	bool GetRXFrame(CAN_FRAME &frame);
	void EnqueueTX(CAN_FRAME& newFrame);
};

extern MCP_CAN CAN;
#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
