/*
 *   Copyright (C) 2009-2017,2020 by Jonathan Naylor G4KLX
 *   Copyright (C) 2025-2026 by Rob Williams M1BGT
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "Config.h"

#if defined(MODE_DMR)

#include "Globals.h"
#include "DMRUserRX.h"
#include "DMRSlotType.h"
#include "Utils.h"

const q15_t SCALING_FACTOR = 19505;      // Q15(0.60)

const uint8_t MAX_SYNC_SYMBOLS_ERRS = 2U;
const uint8_t MAX_SYNC_BYTES_ERRS   = 3U;

// Bursts without a verified sync before the free running frame pointer is
// abandoned. Unlike the per slot implementations this count covers both time
// slots, so the worst normal gap of 11 bursts (one voice slot carrying a sync
// every six of its own bursts, plus an idle slot that never carries one) has
// to fit comfortably below it.
const uint8_t MAX_SYNC_LOST_FRAMES  = 30U;

const uint8_t COLOR_CODE_NONE = 0xFFU;

const uint8_t BIT_MASK_TABLE[] = {0x80U, 0x40U, 0x20U, 0x10U, 0x08U, 0x04U, 0x02U, 0x01U};

#define WRITE_BIT1(p,i,b) p[(i)>>3] = (b) ? (p[(i)>>3] | BIT_MASK_TABLE[(i)&7]) : (p[(i)>>3] & ~BIT_MASK_TABLE[(i)&7])
#define READ_BIT1(p,i)    ((p[(i)>>3] & BIT_MASK_TABLE[(i)&7]) >> (7 - ((i)&7)))

const uint16_t NOENDPTR = 9999U;

const uint8_t CONTROL_DATA  = 0x40U;
const uint8_t CONTROL_VOICE = 0x20U;

const uint8_t cachInterleave[DMR_CACH_LENGTH_BITS] = {
  0, 7, 8, 9, 1, 10, 11, 12, 2, 13, 14,
  15, 3, 16, 4, 17, 18, 19, 5, 20, 21, 22, 6, 23
};

extern uint8_t decodeDMRHamming74(uint8_t *received);

// Parity bits for each of the seven EMB data bits (colour code, PI, LCSS,
// most significant first) of the quadratic residue (16,7,6) code, see
// ETSI TS 102 361-1 Annex B.3.2. Taken from OK-DMRlib.
const uint16_t EMB_PARITY[7U] = {0x04FU, 0x11EU, 0x1B7U, 0x1E2U, 0x1C9U, 0x0E5U, 0x073U};

// Extracts the EMB of a voice burst that is not a sync burst and returns the
// colour code if the quadratic residue check passes. The 16 EMB bits are split
// either side of the 32 bit embedded signalling field, that is burst bits 108
// to 115 and 148 to 155. The frame buffer holds the burst from bit 8 onwards.
static bool decodeEMB(const uint8_t* frame, uint8_t& colorCode)
{
  uint16_t emb = 0U;
  for (uint8_t i = 0U; i < 8U; i++)
    emb = (emb << 1) | READ_BIT1(frame, 8U + 108U + i);
  for (uint8_t i = 0U; i < 8U; i++)
    emb = (emb << 1) | READ_BIT1(frame, 8U + 148U + i);

  uint16_t data   = emb >> 9;
  uint16_t parity = 0U;
  for (uint8_t i = 0U; i < 7U; i++) {
    if ((data >> (6U - i)) & 1U)
      parity ^= EMB_PARITY[i];
  }

  if (parity != (emb & 0x1FFU))
    return false;

  colorCode = (data >> 3) & 0x0FU;
  return true;
}

CDMRUserRX::CDMRUserRX() :
m_bitBuffer(),
m_buffer(),
m_bitPtr(0U),
m_dataPtr(0U),
m_endPtr(NOENDPTR),
m_maxCorr(0),
m_centre(),
m_threshold(),
m_averagePtr(0U),
m_colorCode(0U),
m_slot(false),
m_dataSync(),
m_SuperFrameIndex(),
m_slotColorCode(),
m_hasSync(false),
m_syncThisBurst(false),
m_syncCount(0U)
{
  m_slotColorCode[0U] = COLOR_CODE_NONE;
  m_slotColorCode[1U] = COLOR_CODE_NONE;
}

void CDMRUserRX::reset()
{
  m_dataPtr             = 0U;
  m_bitPtr              = 0U;
  m_maxCorr             = 0;
  m_slot                = false;
  m_endPtr              = NOENDPTR;
  m_hasSync             = false;
  m_syncThisBurst       = false;
  m_syncCount           = 0U;
  m_dataSync[0U]        = false;
  m_dataSync[1U]        = false;
  m_SuperFrameIndex[0U] = 0U;
  m_SuperFrameIndex[1U] = 0U;
  m_slotColorCode[0U]   = COLOR_CODE_NONE;
  m_slotColorCode[1U]   = COLOR_CODE_NONE;
}

void CDMRUserRX::samples(const q15_t* samples, uint8_t length)
{
  for (uint8_t i = 0U; i < length; i++)
    processSample(samples[i]);
}

// Adding RSSI would be useful, but can wait
void CDMRUserRX::processSample(q15_t sample)
{
  m_bitBuffer[m_bitPtr] <<= 1;
  if (sample < 0)
  {
    m_bitBuffer[m_bitPtr] |= 0x01U;
  }

  m_buffer[m_dataPtr] = sample;

  uint8_t errs = countBits32((m_bitBuffer[m_bitPtr] & DMR_SYNC_SYMBOLS_MASK) ^ DMR_BS_DATA_SYNC_SYMBOLS);

  // The voice sync is the complement of the data sync
  bool foundBsDataSync  = (errs <= MAX_SYNC_SYMBOLS_ERRS);
  bool foundBsVoiceSync = (errs >= (DMR_SYNC_LENGTH_SYMBOLS - MAX_SYNC_SYMBOLS_ERRS));

  if (foundBsDataSync || foundBsVoiceSync) {
    // Sync pattern found

    // | CACH (24 bits/60 samples) | Payload (98 bits) | Slot type (10 bits) | Sync (DMR_SYNC_LENGTH_SAMPLES) | Slot type (10 bits) | Payload (98 bits) |

    // ptr is the start of the first sync symbol
    uint16_t ptr = m_dataPtr + DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES - DMR_SYNC_LENGTH_SAMPLES + DMR_RADIO_SYMBOL_LENGTH;
    if (ptr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
    {
      ptr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;
    }

    q31_t corr = 0;
    q15_t max = -16000;
    q15_t min =  16000;

    for (uint8_t i = 0U; i < DMR_SYNC_LENGTH_SYMBOLS; i++) {
      q15_t val = m_buffer[ptr];

      if (val > max)
        max = val;
      if (val < min)
        min = val;

      int8_t corrVal;
      if (foundBsDataSync)
        corrVal = DMR_BS_DATA_SYNC_SYMBOLS_VALUES[i];
      else
        corrVal = DMR_BS_VOICE_SYNC_SYMBOLS_VALUES[i];

      switch (corrVal) {
      case +3:
        corr -= (val + val + val);
        break;
      case +1:
        corr -= val;
        break;
      case -1:
        corr += val;
        break;
      default:  // -3
        corr += (val + val + val);
        break;
      }

      ptr += DMR_RADIO_SYMBOL_LENGTH;
      if (ptr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
        ptr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;
    }

    if (corr > m_maxCorr) {
      q15_t centre = (max + min) >> 1;

      q31_t v1 = (max - centre) * SCALING_FACTOR;
      q15_t threshold = q15_t(v1 >> 15);

      // | CACH (24 bits/60 samples) | Payload (98 bits) | Slot type (10 bits) | Sync (DMR_SYNC_LENGTH_SAMPLES) | Slot type (10 bits) | Payload (98 bits) |
      uint16_t syncPtr = m_dataPtr + DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES - DMR_SYNC_LENGTH_SAMPLES + DMR_RADIO_SYMBOL_LENGTH;
      if (syncPtr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
        syncPtr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;

      uint8_t sync[DMR_SYNC_BYTES_LENGTH];
      samplesToBits(syncPtr, DMR_SYNC_LENGTH_SYMBOLS, sync, 4U, centre, threshold);

      // A symbol level match alone is far too weak to act on, so the sync is
      // verified at the byte level before any state is changed.
      uint8_t syncErrs = 0U;
      const uint8_t* syncBytes = foundBsDataSync ? DMR_BS_DATA_SYNC_BYTES : DMR_BS_VOICE_SYNC_BYTES;
      for (uint8_t i = 0U; i < DMR_SYNC_BYTES_LENGTH; i++)
        syncErrs += countBits8((sync[i] & DMR_SYNC_BYTES_MASK[i]) ^ syncBytes[i]);

      if (syncErrs <= MAX_SYNC_BYTES_ERRS) {
        if (!m_hasSync) {
          // The very first sync has to seed all four averaging slots,
          // otherwise the average is diluted towards zero and the CACH that
          // follows cannot be decoded.
          m_threshold[0U] = m_threshold[1U] = m_threshold[2U] = m_threshold[3U] = threshold;
          m_centre[0U]    = m_centre[1U]    = m_centre[2U]    = m_centre[3U]    = centre;
          m_averagePtr    = 0U;
          m_hasSync       = true;
        } else {
          m_threshold[m_averagePtr] = threshold;
          m_centre[m_averagePtr]    = centre;

          m_averagePtr++;
          if (m_averagePtr >= 4U)
          {
            m_averagePtr = 0U;
          }
        }

        m_maxCorr = corr;

        // This burst ends half a slot type plus half an info field beyond
        // the point at which the sync was recognised.
        m_endPtr  = m_dataPtr + DMR_SLOT_TYPE_LENGTH_SAMPLES / 2U + DMR_INFO_LENGTH_SAMPLES / 2U - 1U;
        if (m_endPtr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
        {
          m_endPtr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;
        }

        // Read the CACH that starts this burst to find the time slot. It is
        // read again when the burst is complete, but doing it here primes
        // the per slot state for the frame that is about to arrive.
        uint16_t cachePtr = m_endPtr + DMR_RADIO_SYMBOL_LENGTH + 1U;
        if (cachePtr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
          cachePtr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;

        q15_t avgCentre    = (m_centre[0U]    + m_centre[1U]    + m_centre[2U]    + m_centre[3U])    >> 2;
        q15_t avgThreshold = (m_threshold[0U] + m_threshold[1U] + m_threshold[2U] + m_threshold[3U]) >> 2;

        uint8_t cach[DMR_CACH_LENGTH_BYTES];
        samplesToBits(cachePtr, DMR_CACH_LENGTH_SYMBOLS, cach, 0U, avgCentre, avgThreshold);

        bool syncSlot = false;
        if (cachSlot(cach, syncSlot)) {
          uint8_t slot = syncSlot ? 1U : 0U;

          m_dataSync[slot]        = foundBsDataSync;
          m_SuperFrameIndex[slot] = 0U;
        }

        m_syncThisBurst = true;
      }
    }
  }

  if (m_dataPtr == m_endPtr) {
    // Account for the burst that has just completed first, so that the final
    // frame of a signal that has gone away is not emitted.
    const bool syncBurst = m_syncThisBurst;
    m_syncThisBurst      = false;

    if (syncBurst) {
      m_syncCount = 0U;
    } else if (++m_syncCount >= MAX_SYNC_LOST_FRAMES) {
      // Without a sync the frame pointer is only free running on stale
      // timing, so stop rather than emitting noise forever.
      m_endPtr              = NOENDPTR;
      m_maxCorr             = 0;
      m_hasSync             = false;
      m_dataSync[0U]        = false;
      m_dataSync[1U]        = false;
      m_SuperFrameIndex[0U] = 0U;
      m_SuperFrameIndex[1U] = 0U;
      m_slotColorCode[0U]   = COLOR_CODE_NONE;
      m_slotColorCode[1U]   = COLOR_CODE_NONE;
    }

    if (m_endPtr != NOENDPTR) {
      q15_t centre    = (m_centre[0U]    + m_centre[1U]    + m_centre[2U]    + m_centre[3U])    >> 2;
      q15_t threshold = (m_threshold[0U] + m_threshold[1U] + m_threshold[2U] + m_threshold[3U]) >> 2;

      // Read the CACH in order to get the time slot
      uint16_t cachePtr = m_endPtr + DMR_RADIO_SYMBOL_LENGTH + 1U;
      if (cachePtr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
      {
        cachePtr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;
      }

      // And the 132 symbol burst that immediately follows it
      uint16_t ptr = cachePtr + DMR_CACH_LENGTH_SAMPLES;
      if (ptr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
      {
        ptr -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;
      }

      uint8_t cach[DMR_CACH_LENGTH_BYTES];
      samplesToBits(cachePtr, DMR_CACH_LENGTH_SYMBOLS, cach, 0U, centre, threshold);

      bool burstSlot = false;
      if (!cachSlot(cach, burstSlot)) {
        // No usable CACH, so nothing is sent for this burst. The alternation
        // is still advanced so the next burst is assigned to the other slot.
        m_slot = !m_slot;
      } else {
        m_slot = burstSlot;
        uint8_t slot = m_slot ? 1U : 0U;

        uint8_t frame[DMR_FRAME_LENGTH_BYTES + 1U];
        samplesToBits(ptr, DMR_FRAME_LENGTH_SYMBOLS, frame, 8U, centre, threshold);

        if (syncBurst)
          m_SuperFrameIndex[slot] = 0U;

        if (m_dataSync[slot]) {
          // Only a sync burst carries a slot type, which gives the colour
          // code of the calling party. The bursts between syncs carry an EMB
          // instead, so they cannot be decoded as data and are not sent.
          if (syncBurst) {
            uint8_t colorCode = COLOR_CODE_NONE;
            uint8_t dataType  = DT_IDLE;

            CDMRSlotType slotType;
            slotType.decode(frame + 1U, colorCode, dataType);

            if (colorCode == m_colorCode && dataType != DT_IDLE) {
              frame[0U] = CONTROL_DATA | dataType;
              serial.writeDMRData(m_slot, frame, DMR_FRAME_LENGTH_BYTES + 1U);
            }
          }
        } else if (m_SuperFrameIndex[slot] < 6U) {
          // A voice superframe is six bursts long. Only the first of them
          // carries the sync, the remaining five are numbered one to five.
          //
          // The sync burst has no EMB, so it is checked against the colour
          // code latched from the EMB of the previous superframe. Each
          // continuation burst refreshes the latch from its own EMB whenever
          // that EMB passes the quadratic residue check.
          if (!syncBurst) {
            uint8_t embColorCode = COLOR_CODE_NONE;
            if (decodeEMB(frame, embColorCode))
              m_slotColorCode[slot] = embColorCode;
          }

          // The index advances even for a burst that is not sent, so the
          // numbering stays aligned with the superframe.
          if (m_slotColorCode[slot] == m_colorCode) {
            frame[0U] = (m_SuperFrameIndex[slot] == 0U) ? CONTROL_VOICE : m_SuperFrameIndex[slot];
            serial.writeDMRData(m_slot, frame, DMR_FRAME_LENGTH_BYTES + 1U);
          }

          m_SuperFrameIndex[slot]++;
        }
      }
    }

    m_maxCorr = 0;
  }

  m_dataPtr++;
  if (m_dataPtr >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
    m_dataPtr = 0U;

  m_bitPtr++;
  if (m_bitPtr >= DMR_RADIO_SYMBOL_LENGTH)
    m_bitPtr = 0U;
}

void CDMRUserRX::extractBits(uint8_t* bytes, uint16_t start, uint8_t count_bits, bool* buffer)
{
    for (uint8_t i = 0U; i < count_bits; i++) {
        buffer[i]  = READ_BIT1(bytes, start) == 0x01;
        start++;
    }
}

// Works out the time slot of the burst that follows this CACH. The state is
// not changed, so the caller decides whether to keep the result.
//
// A clean CACH is always believed. The Hamming code cannot tell a single bit
// error that was corrected from a double error that was miscorrected, so a
// CACH that needed a correction is only believed when its slot is the one
// the alternation of the TDMA slots expects, that is the opposite of the slot
// of the previous burst (m_slot).
bool CDMRUserRX::cachSlot(uint8_t* cach, bool& slot)
{
  uint8_t i;

  bool bits[DMR_CACH_LENGTH_BITS];
  extractBits(cach, 0, DMR_CACH_LENGTH_BITS, bits);

  bool cachdata[DMR_CACH_LENGTH_BITS];
    
  for (i = 0; i < DMR_CACH_LENGTH_BITS; i++)
  {
    cachdata[cachInterleave[i]] = bits[i];
  }

  uint8_t tactBits[7];

  for (i = 0; i < 7; i++)
  {
    tactBits[i] = cachdata[i];
  }

  uint8_t corrected = decodeDMRHamming74(tactBits);

  // bool at_continuous = tactBits[0];
  bool tc = tactBits[1] != 0U; // TDMA Channel (TC)

  if (corrected != 0U && tc == m_slot)
    return false;

  slot = tc;

  return true;
}

void CDMRUserRX::samplesToBits(uint16_t start, uint8_t count, uint8_t* buffer, uint16_t offset, q15_t centre, q15_t threshold)
{
  for (uint8_t i = 0U; i < count; i++) {
    q15_t sample = m_buffer[start] - centre;

    if (sample < -threshold) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    } else if (sample < 0) {
      WRITE_BIT1(buffer, offset, false);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else if (sample < threshold) {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, false);
      offset++;
    } else {
      WRITE_BIT1(buffer, offset, true);
      offset++;
      WRITE_BIT1(buffer, offset, true);
      offset++;
    }

    start += DMR_RADIO_SYMBOL_LENGTH;
    if (start >= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES)
      start -= DMR_FRAME_PLUS_CACH_LENGTH_SAMPLES;
  }
}

void CDMRUserRX::setColorCode(uint8_t colorCode)
{
  m_colorCode = colorCode;
}

#endif
