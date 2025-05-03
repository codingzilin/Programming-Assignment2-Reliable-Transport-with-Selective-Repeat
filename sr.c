#include <stdlib.h>
#include <stdio.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for SR), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
   - adapted to Selective Repeat implementation
**********************************************************************/

#define true 1
#define false 0
#define RTT 16.0      /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6  /* the maximum number of buffered unacked packet */
#define SEQSPACE 12   /* the min sequence space for SR must be at least 2*windowsize + 1 */
#define NOTINUSE (-1) /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);

  return checksum;
}

int IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (0);
  else
    return (1);
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE]; /* array for storing packets waiting for ACK */
static int timers[WINDOWSIZE];        /* array for tracking which timers are active */
static int acked[WINDOWSIZE];        /* array for tracking which packets are ACKed */
static int windowfirst, windowlast;     /* array indexes of the first/last packet in window */
static int windowcount;               /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;              /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE)
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    acked[windowlast] = 0; /* track packet status */
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1){
      starttimer(A, RTT); /* start timer for the first packet in window */
      timers[windowlast] = 1;
    }
    
    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else
  {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int i;
  int found = 0; 

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    if (windowcount != 0) {
      /* find the packet in window */
      for (i = 0; i < windowcount; i++) {
        int slot = (windowfirst + i) % WINDOWSIZE;
        if (buffer[slot].seqnum == packet.acknum && !acked[slot]) {
          acked[slot] = 1;
          found = 1;
          /* packet is a new ACK */
          if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n",packet.acknum);
          new_ACKs++;

          /* stop timer if this is the first packet in window */
          if (slot == windowfirst) {
            stoptimer(A);
            timers[slot] = 0;
          }

          while (windowcount > 0 && acked[windowfirst]) {
            acked[windowfirst] = 0;
            timers[windowfirst] = 0;
            windowfirst = (windowfirst + 1) % WINDOWSIZE;
            windowcount--;
          }

          if (windowcount > 0) {
            for (int i = 0; i < WINDOWSIZE; i++) {
              int slot = (windowfirst + i) % WINDOWSIZE;
              if (!acked[slot]) {
                starttimer(A, RTT);
                timers[slot] = 1;
                break;
              }
            }
          }
          break;
        }
      }
    }
  } 
  else { 
    if (TRACE > 0)
    printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for (int i = 0; i < WINDOWSIZE; i++) {
    int slot = (windowfirst + i) % WINDOWSIZE;
    if (!acked[slot]) {
      /* resend the packet */
      if (TRACE > 0)
        printf("Sending packet %d to layer 3\n", buffer[slot].seqnum);
      tolayer3(A, buffer[slot]);
      packets_resent++;
      starttimer(A, RTT);
      timers[slot] = 1;  
      break;
    }
  }
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0; /* A starts with seq num 0, do not change this */
  windowfirst = 0; 
  windowlast = -1; 
  windowcount = 0;
  /* initialize all packet_status entries to indicate they're not in use */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = 0; /* initially all slots are available */
    timers[i] = 0; 
  }  
}

/********* Receiver (B)  variables and procedures ************/

static int recv_base; /* Base sequence number expected by receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt rcv_buffer[SEQSPACE]; /* buffer for out of order packets */
static int received[SEQSPACE]; /* track which packets have been received */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;

  /* check if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    /* calculate expected window */
    int offset = (packet.seqnum - recv_base + SEQSPACE) % SEQSPACE;
    if (offset < WINDOWSIZE) {
      if (!received[packet.seqnum]) {
        received[packet.seqnum] = 1;
        rcv_buffer[packet.seqnum] = packet;
        if (TRACE > 0)
          printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);
        packets_received++;
      }

      /* deliver in-order packets to layer 5 */
      while (received[recv_base]) {
        tolayer5(B, rcv_buffer[recv_base].payload);
        received[recv_base] = 0;
        recv_base = (recv_base + 1) % SEQSPACE;
      }
      /* send ACK for the received packet */
      sendpkt.acknum = packet.seqnum;
    } else {
      /* packet outside receive window, send ACK anyway */
      if (TRACE > 0)
        printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
      sendpkt.acknum = (recv_base - 1 + SEQSPACE) % SEQSPACE;
    }
  } else {
      if (TRACE > 0)
        printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
      sendpkt.acknum = (recv_base - 1 + SEQSPACE) % SEQSPACE;
    }

    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';

    sendpkt.checksum = ComputeChecksum(sendpkt);

    tolayer3(B, sendpkt); 
  }
  
  

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  recv_base = 0;
  B_nextseqnum = 1;
  /* initialize the receive buffer and packet */
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = 0; /* mark all buffer slots as empty */
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
