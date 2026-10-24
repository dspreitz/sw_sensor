/***********************************************************************//**
 * @file		UBX_receiver.cpp
 * @brief		assembles UBX NAV-PVT and its heading message into epochs
 * @license 		This project is released under the GNU Public License GPL-3.0
 **************************************************************************/

#include "UBX_receiver.h"
#include <string.h>

#define UBX_CLASS_NAV		0x01
#define MAX_SKIPPED_LENGTH	1024 // longer: most likely a false sync, search again

static uint32_t read_u32 (const uint8_t *p)
{
  return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

// iTOW is at payload offset 0 in NAV-PVT, 4 in NAV-RELPOSNED and NAV-DAHEADING
static uint32_t itow_of (const uint8_t *frame)
{
  return read_u32 (frame + 6 + (frame[3] == UBX_NAV_PVT_ID ? 0 : 4));
}

UBX_receiver::UBX_receiver (uint8_t id)
{
  reset (id);
}

void UBX_receiver::reset (uint8_t id)
{
  checksum_errors = 0;
  skipped_messages = 0;
  second_id = id;
  second_length = (id == UBX_NAV_RELPOSNED_ID) ? UBX_NAV_RELPOSNED_LENGTH :
		  (id == UBX_NAV_DAHEADING_ID) ? UBX_NAV_DAHEADING_LENGTH : 0;
  state = SYNC1;
  pos = length = 0;
  keep = false;
  ck_a = ck_b = 0;
  pvt_waiting = second_waiting = false;
  pvt_itow = second_itow = 0;
  epoch_second = false;
}

bool UBX_receiver::emit_pvt (bool with_second)
{
  memcpy (epoch_buffer, pvt, UBX_PVT_FRAME_SIZE);
  if (with_second)
    memcpy (epoch_buffer + UBX_PVT_FRAME_SIZE, second, second_length + UBX_FRAME_OVERHEAD);
  else
    memset (epoch_buffer + UBX_PVT_FRAME_SIZE, 0, UBX_SECOND_FRAME_MAX);
  epoch_second = with_second;
  pvt_waiting = false;
  if (with_second)
    second_waiting = false;
  return true;
}

bool UBX_receiver::message_complete (void)
{
  if (message[3] == UBX_NAV_PVT_ID)
    {
      bool emitted = false;
      if (pvt_waiting) // the previous epoch didn't get its heading message
	emitted = emit_pvt (false);
      memcpy (pvt, message, UBX_PVT_FRAME_SIZE);
      pvt_itow = itow_of (pvt);
      if (second_id == 0)
	return emit_pvt (false);
      pvt_waiting = true;
      // heading came first; emitted is false then, as a stored heading
      // message completes any waiting PVT
      if (second_waiting && second_itow == pvt_itow)
	return emit_pvt (true);
      second_waiting = false; // older than this PVT, can't match any more
      return emitted;
    }

  // heading message
  memcpy (second, message, second_length + UBX_FRAME_OVERHEAD);
  second_itow = itow_of (second);
  second_waiting = true;
  if (pvt_waiting)
    return emit_pvt (second_itow == pvt_itow);
  return false;
}

bool UBX_receiver::feed (uint8_t byte)
{
  switch (state)
    {
    case SYNC1:
      if (byte == 0xb5)
	state = SYNC2;
      return false;

    case SYNC2:
      if (byte == 'b')
	{
	  state = HEADER;
	  pos = 2;
	  ck_a = ck_b = 0;
	  message[0] = 0xb5;
	  message[1] = 'b';
	}
      else
	state = (byte == 0xb5) ? SYNC2 : SYNC1;
      return false;

    case HEADER:
      message[pos++] = byte;
      ck_a += byte;
      ck_b += ck_a;
      if (pos < 6)
	return false;
      length = message[4] | (message[5] << 8);
      keep = (message[2] == UBX_CLASS_NAV)
	  && (((message[3] == UBX_NAV_PVT_ID) && (length == UBX_NAV_PVT_LENGTH))
	      || ((second_id != 0) && (message[3] == second_id) && (length == second_length)));
      if (!keep && (length > MAX_SKIPPED_LENGTH))
	{
	  state = SYNC1;
	  return false;
	}
      state = (length == 0) ? CHECKSUM : PAYLOAD;
      return false;

    case PAYLOAD:
      if (keep)
	message[pos] = byte;
      ++pos;
      ck_a += byte;
      ck_b += ck_a;
      if (pos == 6 + length)
	state = CHECKSUM;
      return false;

    case CHECKSUM:
      if (keep)
	message[pos] = byte;
      ++pos;
      if (pos < 6 + length + 2)
	return false;
      state = SYNC1;
      if (!keep)
	{
	  ++skipped_messages;
	  return false;
	}
      if ((message[6 + length] != ck_a) || (message[6 + length + 1] != ck_b))
	{
	  ++checksum_errors;
	  return false;
	}
      return message_complete ();
    }
  return false;
}

bool UBX_receiver::flush (void)
{
  if (!pvt_waiting)
    return false;
  return emit_pvt (false);
}
