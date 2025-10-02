#ifndef LLCP_H
#define LLCP_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// | ------------------- BEGIN: USER CONFIG ------------------- |

// If LLCP_COMM_HEXADECIMAL == 1, the protocol encodes data into HEXADECIMAL
// ASCII characters, which do not spooke serial line drivers. It is also
// much easier to debug, if all the data is human-readible.
#ifndef LLCP_COMM_HEXADECIMAL
#define LLCP_COMM_HEXADECIMAL 1
#endif

// should we send '\n' after each packet?
#ifndef LLCP_APPEND_ENDL
#define LLCP_APPEND_ENDL 0
#endif

#ifndef LLCP_DEBUG_PRINT
#define LLCP_DEBUG_PRINT 0
#endif

// max payload length in bytes, max 255
#define MAX_PAYLOAD_LEN 255

#ifndef LLCP_CHECK_CHECKSUM
#define LLCP_CHECK_CHECKSUM 1
#endif

// | -------------------- END: USER CONFIG -------------------- |

/* automatically-defined configs //{ */

// definition of the byte lenghts of the various section of the comm message
#define INIT_LEN 1
#define PAYLOAD_SIZE_LEN 1
#define CHECKSUM_LEN 1

#if LLCP_APPEND_ENDL == 1
#define ENDL_LEN 1
#else
#define ENDL_LEN 0
#endif

// the total max size of the message defines the size of the buffer we need
#if LLCP_COMM_HEXADECIMAL == 0
#define LLCP_RX_TX_BUFFER_SIZE (INIT_LEN + PAYLOAD_SIZE_LEN + MAX_PAYLOAD_LEN + CHECKSUM_LEN + ENDL_LEN)
#else
#define LLCP_RX_TX_BUFFER_SIZE (INIT_LEN + (PAYLOAD_SIZE_LEN + MAX_PAYLOAD_LEN + CHECKSUM_LEN) * 2 + ENDL_LEN)
#endif

#if LLCP_DEBUG_PRINT == 1
#include <stdio.h>
#endif

//}

// | ------------------------- structs ------------------------ |

/* struct LLCP_ReceiverState_t //{ */

#if LLCP_COMM_HEXADECIMAL == 0
/**
 * @brief state machine states for the LLCP receiver (binary transfer)
 */
typedef enum
{
  WAITING_FOR_MESSSAGE = 0,
  EXPECTING_SIZE       = 1,
  EXPECTING_PAYLOAD    = 2,
  EXPECTING_CHECKSUM   = 3,
} LLCP_ReceiverState_t;
#else
/**
 * @brief state machine states for the LLCP receiver (hexadecimal transfer)
 */
typedef enum
{
  WAITING_FOR_MESSSAGE = 0,
  EXPECTING_SIZE       = 1,
  EXPECTING_SIZE_2     = 2,
  EXPECTING_PAYLOAD    = 3,
  EXPECTING_CHECKSUM   = 4,
  EXPECTING_CHECKSUM_2 = 5,
} LLCP_ReceiverState_t;
#endif

//}

/* struct LLCP_Receiver_t //{ */

#if LLCP_COMM_HEXADECIMAL == 0
typedef struct __attribute__((packed))
{
  LLCP_ReceiverState_t state;
  uint16_t             payload_size;
  uint8_t              rx_buffer[LLCP_RX_TX_BUFFER_SIZE];
  uint16_t             buffer_counter;
  uint8_t              checksum;
} LLCP_Receiver_t;
#else
typedef struct __attribute__((packed))
{
  LLCP_ReceiverState_t state;
  uint16_t             payload_size;
  uint8_t              rx_buffer[LLCP_RX_TX_BUFFER_SIZE];
  uint16_t             buffer_counter;
  uint8_t              checksum;
  uint8_t              hexmem;  // for remembering the first one-byt of the 2-byte HEX byte
} LLCP_Receiver_t;
#endif

//}

/* struct LLCP_Message_t //{ */

typedef struct __attribute__((packed))
{
  uint8_t payload[MAX_PAYLOAD_LEN];
  uint8_t checksum_matched;
} LLCP_Message_t;

//}

// | ----------------------- hex <-> bin ---------------------- |

uint8_t llcp_hex2bin(const uint8_t* ptr);

void llcp_bin2hex(const uint8_t byte, uint8_t* buffer);

// | ------------------------ routines ------------------------ |

void llcp_initialize(LLCP_Receiver_t* receiver);

bool llcp_processChar(const uint8_t char_in, LLCP_Receiver_t* receiver, LLCP_Message_t** message, bool* checksum_matched);

uint16_t llcp_prepareMessage(uint8_t* what, uint8_t len, uint8_t* buffer);

#endif  // LLCP_H
