/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "config_polystack.pb.h"

#define BASE_MEMORY_ADDRESS 0xa0
#define BASE_SERIAL_DEVICE_ADDRESS 0xb0
#define SERIAL_WORD_ADDRESS 0x0800

typedef struct _I2CMemoryStreamState {
    uint8_t device_address;
    uint16_t next_memory_address;
/* @@protoc_insertion_point(struct:PolystackMod) */
} I2CMemoryStreamState;

void polystackAutoConfigure(void);
bool polystackRead(uint8_t index, PolystackMod* mod_info);
bool polystackReadSerial(uint8_t index, uint8_t serial_number[16]);
