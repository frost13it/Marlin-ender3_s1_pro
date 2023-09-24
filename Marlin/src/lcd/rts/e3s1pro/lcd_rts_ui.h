/**
 * The idea for this class was taken from "Professional Firmware by Author: Miguel A. Risco-Castillo (MRISCOC)"
 * This class was modified by Thomas Toka for MARLIN-E3S1PRO-FORK-BYTT
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include "../../../inc/MarlinConfigPre.h"

//#define X_BED_MIN 150
//#define Y_BED_MIN 150
//constexpr int16_t DEF_X_BED_SIZE = X_BED_SIZE;
//constexpr int16_t DEF_Y_BED_SIZE = Y_BED_SIZE;
//constexpr int16_t DEF_X_MIN_POS = X_MIN_POS;
//constexpr int16_t DEF_Y_MIN_POS = Y_MIN_POS;
//constexpr int16_t DEF_X_MAX_POS = X_MAX_POS;
//constexpr int16_t DEF_Y_MAX_POS = Y_MAX_POS;
//constexpr int16_t DEF_Z_MAX_POS = Z_MAX_POS;
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  constexpr int8_t DEF_GRID_MAX_POINTS = GRID_MAX_POINTS_X;
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)  
// constexpr int16_t DEF_MESH_MIN_X = MESH_MIN_X;
// constexpr int16_t DEF_MESH_MAX_X = MESH_MAX_X;
// constexpr int16_t DEF_MESH_MIN_Y = MESH_MIN_Y;
// constexpr int16_t DEF_MESH_MAX_Y = MESH_MAX_Y;
//#define MIN_MESH_INSET TERN(HAS_BED_PROBE, PROBING_MARGIN, 5)
//#define MAX_MESH_INSET X_BED_SIZE
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #ifndef PROBING_MARGIN
    #define PROBING_MARGIN 45
  #endif
  #ifndef PROBING_MARGIN_BACK
    #define PROBING_MARGIN_BACK 45
  #endif  
constexpr int16_t DEF_PROBING_MARGIN = PROBING_MARGIN;
//constexpr int16_t DEF_PROBING_MARGIN_LEFT = PROBING_MARGIN;
//constexpr int16_t DEF_PROBING_MARGIN_RIGHT = PROBING_MARGIN;
//constexpr int16_t DEF_PROBING_MARGIN_FRONT = PROBING_MARGIN;
constexpr int16_t DEF_PROBING_MARGIN_BACK = PROBING_MARGIN_BACK;
#define MIN_PROBE_MARGIN 5
#define MAX_PROBE_MARGIN 60
#endif

//#ifndef MULTIPLE_PROBING
//  #define MULTIPLE_PROBING 0
//#endif

typedef struct {
//  int16_t bed_size_x = DEF_X_BED_SIZE;
//  int16_t bed_size_y = DEF_Y_BED_SIZE;
//  int16_t x_min_pos  = DEF_X_MIN_POS;
//  int16_t y_min_pos  = DEF_Y_MIN_POS;
//  int16_t x_max_pos  = DEF_X_MAX_POS;
//  int16_t y_max_pos  = DEF_Y_MAX_POS;
//  int16_t z_max_pos  = DEF_Z_MAX_POS;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    uint8_t grid_max_points = DEF_GRID_MAX_POINTS;
  #endif
  #if ENABLED(AUTO_BED_LEVELING_UBL)  
//   mutable int16_t mesh_min_x = DEF_MESH_MIN_X;
//   mutable int16_t mesh_max_x = DEF_MESH_MAX_X;
//   mutable int16_t mesh_min_y = DEF_MESH_MIN_Y;
//   mutable int16_t mesh_max_y = DEF_MESH_MAX_Y;
  #endif
  //uint8_t multiple_probing = MULTIPLE_PROBING;
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)  
    mutable uint8_t probing_margin_x = DEF_PROBING_MARGIN;
    mutable uint8_t probing_margin_y = DEF_PROBING_MARGIN_BACK;    
  #endif
} lcd_rts_data_t;
//extern lcd_rts_data_t lcd_rts_data;
constexpr lcd_rts_data_t lcd_rts_data;