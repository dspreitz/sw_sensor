/***********************************************************************//**
 * @file		UBX_receiver.h
 * @brief		assembles UBX NAV-PVT and its heading message into epochs
 * @license 		This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#ifndef UBX_RECEIVER_H_
#define UBX_RECEIVER_H_

#include <stdint.h>

#define UBX_FRAME_OVERHEAD	8	// "u B class id size1 size2 ... cks1 cks2"
#define UBX_NAV_PVT_LENGTH	92
#define UBX_NAV_RELPOSNED_LENGTH 64
#define UBX_NAV_DAHEADING_LENGTH 60
#define UBX_NAV_PVT_ID		0x07
#define UBX_NAV_RELPOSNED_ID	0x3c
#define UBX_NAV_DAHEADING_ID	0x45

#define UBX_PVT_FRAME_SIZE	(UBX_NAV_PVT_LENGTH + UBX_FRAME_OVERHEAD)
#define UBX_SECOND_FRAME_MAX	(UBX_NAV_RELPOSNED_LENGTH + UBX_FRAME_OVERHEAD)
#define UBX_EPOCH_SIZE		(UBX_PVT_FRAME_SIZE + UBX_SECOND_FRAME_MAX)

//! Byte-wise UBX parser that pairs NAV-PVT with the heading message
//! (NAV-RELPOSNED or NAV-DAHEADING) of the same iTOW, in either order.
//! Other messages are skipped. The epoch buffer has the layout the GNSS_type
//! update functions expect: PVT frame at 0, heading frame right behind it,
//! zeroed if the epoch has no heading message.
class UBX_receiver
{
public:
  //! second_id: UBX_NAV_RELPOSNED_ID, UBX_NAV_DAHEADING_ID, or 0 for PVT only
  explicit UBX_receiver (uint8_t second_id = 0);

  //! forget everything received so far and select the heading message
  void reset (uint8_t second_id);

  //! returns true if an epoch is complete, then epoch() is valid until the next call
  bool feed (uint8_t byte);

  //! call when the line has been idle for a while: completes a waiting PVT without heading
  bool flush (void);

  const uint8_t * epoch (void) const
  {
    return epoch_buffer;
  }
  bool epoch_has_second (void) const
  {
    return epoch_second;
  }

  // statistics, for tests and debugging
  uint32_t checksum_errors;
  uint32_t skipped_messages;

private:
  enum state_t { SYNC1, SYNC2, HEADER, PAYLOAD, CHECKSUM };

  bool message_complete (void);
  bool emit_pvt (bool with_second);

  uint8_t second_id;
  uint16_t second_length;

  state_t state;
  uint16_t pos;		// bytes of the current message so far, header included
  uint16_t length;	// payload length of the current message
  bool keep;		// current message is one we store
  uint8_t ck_a, ck_b;
  uint8_t message[UBX_SECOND_FRAME_MAX > UBX_PVT_FRAME_SIZE ? UBX_SECOND_FRAME_MAX : UBX_PVT_FRAME_SIZE];

  uint8_t pvt[UBX_PVT_FRAME_SIZE];
  uint8_t second[UBX_SECOND_FRAME_MAX];
  bool pvt_waiting;	// PVT stored, its heading message not yet seen
  bool second_waiting;	// heading message stored, its PVT not yet seen
  uint32_t pvt_itow;
  uint32_t second_itow;

  uint8_t epoch_buffer[UBX_EPOCH_SIZE];
  bool epoch_second;
};

#endif /* UBX_RECEIVER_H_ */
