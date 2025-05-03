#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
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

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE]; /* array for storing packets waiting for ACK */
static int timers[WINDOWSIZE];        /* array for tracking which timers are active */
static bool acked[WINDOWSIZE];        /* array for tracking which packets are ACKed */
static int send_base;               /* the base of the sender's window */
static int windowcount;               /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;              /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int bufferindex;

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
    bufferindex = A_nextseqnum % WINDOWSIZE; /* calculate buffer index */
    buffer[bufferindex] = sendpkt;
    acked[bufferindex] = false; /* track packet status */

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if first packet in window */
    timers[bufferindex] = A_nextseqnum;
    starttimer(A, RTT);

    windowcount++; 
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
  int index;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    if (((send_base <= (send_base + WINDOWSIZE - 1) % SEQSPACE) && 
         (packet.acknum >= send_base && packet.acknum <= (send_base + WINDOWSIZE - 1) % SEQSPACE)) || 
        ((send_base > (send_base + WINDOWSIZE - 1) % SEQSPACE) && 
         (packet.acknum >= send_base || packet.acknum <= (send_base + WINDOWSIZE - 1) % SEQSPACE))) {
      
        index = packet.acknum % WINDOWSIZE;
    /* Find the window index for this packet */
    if (acked[index]) {
        if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
      } else {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;
        acked[index] = true; /* mark this packet as ACKed */
        stoptimer(A); /* stop the timer for this packet */

        if (packet.acknum == send_base) {
          while (acked[send_base % WINDOWSIZE]) {
            acked[send_base % WINDOWSIZE] = false; /* reset for future use */
            send_base = (send_base + 1) % SEQSPACE; 
            windowcount--;

            if (windowcount == 0) 
              break;
          } 
        }

        if (windowcount > 0) {
          for (int i = 0; i < WINDOWSIZE; i++) {
            int seq = (send_base + i) % SEQSPACE;
            if (seq == A_nextseqnum)
              break;
            index = seq % WINDOWSIZE;
            if (!acked[index]) {
              starttimer(A, RTT);
              break;
            }
          }
        } 
      }
    }  else {
      if (TRACE > 0)
      printf("----A: ACK %d outside current window, do nothing!\n");
    }
  } else { 
    if (TRACE > 0)
    printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int index;
  // static int next_timeout = 0; /* track which packet to timeout next */

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  for (int i = 0; i < WINDOWSIZE; i++) {
    index = (send_base + i) % SEQSPACE % WINDOWSIZE;
    if (!acked[index] && (send_base + i) % SEQSPACE != A_nextseqnum) {
      /* resend the packet */
      if (TRACE > 0)
        printf("----A: resending packet %d\n", buffer[index].seqnum);
      tolayer3(A, buffer[index]);
      packets_resent++;
      starttimer(A, RTT); 
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
  send_base = 0; /* the base of the sender's window */
  windowcount = 0;
  /* initialize all packet_status entries to indicate they're not in use */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = true; /* initially all slots are available */
    timers[i] = NOTINUSE; 
  }  
}

/********* Receiver (B)  variables and procedures ************/

static int recv_base; /* Base sequence number expected by receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt rcv_buffer[WINDOWSIZE]; /* buffer for out of order packets */
static bool received[WINDOWSIZE]; /* track which packets have been received */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int index;

  /* check if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    /* calculate expected window */
    int window_end = (recv_base + WINDOWSIZE - 1) % SEQSPACE;

    /* check if packet is within receive window */
    bool in_window = (recv_base <= window_end && packet.seqnum >= recv_base && packet.seqnum <= window_end) ||
                     (recv_base > window_end && (packet.seqnum >= recv_base || packet.seqnum <= window_end));
    
    
    if (in_window) {
      /* packet is within receive window */
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      
      packets_received++;
      index = packet.seqnum % WINDOWSIZE; /* calculate buffer index */
      /* store packet and mark as received */
      rcv_buffer[index] = packet;
      received[index] = true;

      /* if packet.seqnum equals recv_base, deliver it and any consecutive packets */
      if (packet.seqnum == recv_base) {
        while (received[recv_base % WINDOWSIZE]) {
          /* deliver packet to upper layer */
          tolayer5(B, rcv_buffer[recv_base % WINDOWSIZE].payload);
          
          /* mark as not received for future use */
          received[recv_base % WINDOWSIZE] = false;
          
          /* advance base */
          recv_base = (recv_base + 1) % SEQSPACE;
        }
      }

      /* send ACK for the received packet */
      sendpkt.acknum = packet.seqnum;
    } else {
      /* packet outside receive window, send ACK anyway */
      if (TRACE > 0)
        printf("----B: packet outside receive window, send ACK!\n");
      sendpkt.acknum = packet.seqnum;
    }
  } else {
      if (TRACE > 0)
        printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");

      if (recv_base == 0)
        sendpkt.acknum = SEQSPACE - 1;
      else
        sendpkt.acknum = recv_base - 1;
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
  for (i = 0; i < WINDOWSIZE; i++) {
    received[i] = false; /* mark all buffer slots as empty */
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
