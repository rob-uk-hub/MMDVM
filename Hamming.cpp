/*
 *   Copyright (C) 2025 by Rob Williams M1BGT
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

#include <stdint.h>
//#include <cstdio> // Only required for additional debug
#include "Globals.h"

// Map from syndrome value to error bit index in DMR order
const int syndromeMap[8] = {
    -1, // 0: no error
    4, // syndrome 1 → P1 (bit 4)
    5, // syndrome 2 → P2 (bit 5)
    2, // syndrome 3 → D3 (bit 2)
    6, // syndrome 4 → P3 (bit 6)
    0, // syndrome 5 → D1 (bit 0)
    3, // syndrome 6 → D4 (bit 3)
    1, // syndrome 7 → D2 (bit 1)
};

// Decode DMR CACH Hamming(7,4)
bool decodeSingleDMRHamming74(uint8_t *received) {
    /*
    D1 2 3 4 P1 2 3
     1 0 0 0  1 0 1. (5)
     0 1 0 0  1 1 1. (7)
     0 0 1 0  1 1 0. (3)
     0 0 0 1  0 1 1  (6)
    */
    
    uint8_t D1 = received[0];
    uint8_t D2 = received[1];
    uint8_t D3 = received[2];
    uint8_t D4 = received[3];

    uint8_t P1 = received[4];
    uint8_t P2 = received[5];
    uint8_t P3 = received[6];
    
    // Calculate syndrome bits
    uint8_t S1 = P1 ^ D1 ^ D2 ^ D3;
    uint8_t S2 = P2 ^ D2 ^ D3 ^ D4;
    uint8_t S3 = P3 ^ D1 ^ D2 ^ D4;

    uint8_t syndrome = (S3 << 2) | (S2 << 1) | S1;

    if (syndrome != 0) {
        int errorBit = syndromeMap[syndrome];
        if (errorBit != -1) {
            // A single bit error is routine on a weak signal, so it is
            // corrected silently rather than being reported as a fault.
            received[errorBit] ^= 1;  // Flip bit

            // Return false to indicate a change was made (it might be good now)
            return false;
        }
    }

    // No bits flipped - good
    return true;
}

// Corrects at most one bit of a DMR Hamming(7,4) codeword in place and returns
// the number of bits corrected (0 or 1). The code is perfect, so every 7 bit
// word has a valid correction. A return of 0 is therefore the only evidence
// that the word arrived clean, and a return of 1 means it may be wrong.
uint8_t decodeDMRHamming74(uint8_t *received) {
    return decodeSingleDMRHamming74(received) ? 0U : 1U;
}
