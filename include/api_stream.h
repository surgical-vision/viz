/***************************************************************************
* Intuitive Surgical Application Programming Interface
*
* Confidential.  Not to be copied or shared with any third party.
*
* Any use of this API software other than pursuant to the terms of the
* API Collaboration Agreement with Intuitive Surgical, Inc. is prohibited.  
* Any information, ideas or property, whether intellectual or otherwise, 
* that result from any unauthorized use of this API software shall be the 
* sole property of Intuitive Surgical, Inc.  All rights reserved
*
* Copyright (C) 2002 Intuitive Surgical, Inc.
*
***************************************************************************/
/**********************************************************************
 *
 * File Name:   API_STREAM.H
 *
 * Processor:   This file is shared by the PC, MSC, CTP, and CEs.
 *
 * Description: This file contains definitions for API streams.
 *
 *
 * PVCS Revision $Log:   W:\logs\da_Vinci\sys\laptop\API Client\api_dialog\api_stream.h_v  $
 * 
 *    Rev 1.6   Nov 11 2002 15:28:20   PhilipG
 * No change.
 * 
 *    Rev 1.5   Nov 08 2002 14:59:18   PhilipG
 * console events now displayed
 * 
 *    Rev 1.4   Nov 07 2002 13:11:28   PhilipG
 * new password field and data display
 * 
 *    Rev 1.3   Nov 04 2002 13:29:16   PhilipG
 * added password to dialog
 * 
 *    Rev 1.2   Oct 25 2002 14:44:36   PhilipG
 * trying to get V: to update
 * 
 *    Rev 1.1   Oct 25 2002 14:38:14   PhilipG
 * create tab delimited text output files for each stream
 * 
 *    Rev 1.7   Oct 23 2002 17:34:32   PhilipG
 * added more structure definitions for stream
 * 
 *    Rev 1.6   Oct 18 2002 08:56:16   PhilipG
 * initial stream functionality is working
 * 
 *    Rev 1.5   Oct 14 2002 14:21:46   PhilipG
 * added API_TERMINATE for closing socket connection
 * 
 *    Rev 1.4   09 Oct 2002 09:47:32   tedw
 * Added start1 and start2 entries and defines.
 * 
 *    Rev 1.3   08 Oct 2002 09:25:54   tedw
 * A4.1 API Development.
 * 
 *    Rev 1.2   04 Oct 2002 10:23:30   tedw
 * Added header and body to struct.  Added new define for control type.
 *
 **********************************************************************/

#ifndef STREAM_H
#define STREAM_H

/************************************************************************
 * API STREAM
 ***********************************************************************/
typedef struct
{
    float x;
    float y;
    float z;

} API_VECTOR;

typedef struct
{
    API_VECTOR xaxis;
    API_VECTOR yaxis;
    API_VECTOR zaxis;

} API_FRAME;

typedef struct
{
    API_VECTOR vec;
    API_FRAME  frm;

} API_TRANSFORM;

typedef struct
{
  // tool tip in surgeon's eye frame
  API_TRANSFORM tool_tip_pos;
  API_VECTOR tool_tip_trans_vel;
  API_VECTOR tool_tip_rot_vel;
  // joint positions
  float jnt_pos[8];
  // joint velocities
  float jnt_vel[8];
} API_MTM;

typedef struct
{
  // tool tip in camera tip frame
  API_TRANSFORM tool_tip_pos;
  API_VECTOR tool_tip_trans_vel;
  API_VECTOR tool_tip_rot_vel;
  // joint positions
  float jnt_pos[7];
  // joint velocities
  float jnt_vel[7];
  // remote center 
  API_TRANSFORM remote_center;
  // mount point
  API_TRANSFORM Tmount;
  // setup joint
  float sj_joint_angles[6];
} API_PSM;

typedef struct
{
  // tool tip in camera frame
  API_TRANSFORM tool_tip_pos;
  API_VECTOR tool_tip_trans_vel;
  API_VECTOR tool_tip_rot_vel;
  // joint positions
  float jnt_pos[4];
  // joint velocities
  float jnt_vel[4];
  // remote center 
  API_TRANSFORM remote_center;
  // mount point
  API_TRANSFORM Tmount;
  // setup joint
  float sj_joint_angles[6];
} API_ECM;

#define API_UNKNOWN_PACKET       	0
#define API_DEFAULT_PACKET_MTM      1
#define API_DEFAULT_PACKET_PSM      2
#define API_DEFAULT_PACKET_ECM      3

#define API_PACKET_TYPE_COUNT       4

#define API_DATA_LENGTH (64 - 2) //CTP_MSC_MAX_DATA_LENGTH = 64

typedef struct
{
  unsigned long timestamp;     /* Message Command */
  unsigned long struct_id;
  unsigned long data[API_DATA_LENGTH];

} API_Stream;

/************************************************************************
 * API EVENTS
 ***********************************************************************/

/* Max API event per message. */
#define API_EVENT_COUNT (10)

/* defines for source */
#define API_SOURCE_CONSOLE 6

#define API_STANDBY_SWITCH_PRESSED   199
#define API_STANDBY_SWITCH_RELEASED  200
#define API_READY_SWITCH_PRESSED     201
#define API_READY_SWITCH_RELEASED    202
#define API_MASTER_CLUCH_ON          213
#define API_MASTER_CLUCH_OFF         214
#define API_CAMERA_CONTROL_ON        215
#define API_CAMERA_CONTROL_OFF       216
#define API_HEAD_IN                  223 
#define API_HEAD_OUT                 224
#define API_ARM_SWAP                 225
#define API_VIDEO_SWAP               226

typedef struct
{
  unsigned long timestamp;     /* Message Command */
  unsigned long count;
  unsigned long reserved1;
  unsigned long reserved2;

  struct {
    unsigned long id;          /* event id (These match the usr_rqst.h enum) */
    unsigned long source;      /* console, etc. */

  } events[API_EVENT_COUNT];

} API_Event;


/************************************************************************
 * API CONTROL
 ***********************************************************************/

/* Defines for command */
#define API_STREAM_OFF     0
#define API_STREAM_ON      1
#define API_TERMINATE      2

typedef struct
{
  unsigned long password;
  unsigned long command;       /* stream on or stream off         */
  unsigned long rate;          /* Stream freq, if command is "on" */

} API_Control;


/************************************************************************
 * API ETHERNET PACKET
 ***********************************************************************/
#define API_START_PACKET1 0xa5a5a5a5
#define API_START_PACKET2 0x5a5a5a5a

#define API_PACKET_STREAM  1
#define API_PACKET_EVENT   2
#define API_PACKET_CONTROL 3

typedef struct {

    unsigned long start1;  /* Identifier for start of a packet.              */
    unsigned long start2;  /* Identifier for start of a packet.              */
    unsigned long length; /* Length is the size of the body of this message */
    unsigned long type;   /* API_PACKET... type                             */

} API_Eth_Packet_Header;

typedef union {
    API_Stream  stream;
    API_Event   event;
    API_Control control;

} API_Eth_Packet_Body;

typedef struct
{
  API_Eth_Packet_Header header;
  API_Eth_Packet_Body   body;
  
} API_Ethernet_Packet;

#endif
