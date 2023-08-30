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

#if ENABLED(E3S1PRO_RTS)

#include <stddef.h>
#include "../../../core/types.h"
#include "lcd_rts_ui.h"

//#undef X_BED_SIZE
//#undef Y_BED_SIZE
//#undef X_MIN_POS
//#undef Y_MIN_POS
//#undef X_MAX_POS
//#undef Y_MAX_POS
//#undef Z_MAX_POS
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #undef PROBING_MARGIN
  #undef PROBING_MARGIN_LEFT
  #undef PROBING_MARGIN_RIGHT
  #undef PROBING_MARGIN_FRONT
  #undef PROBING_MARGIN_BACK
  #undef GRID_MAX_POINTS_X
  #undef GRID_MAX_POINTS_Y
  #undef GRID_MAX_POINTS
#endif

#if ENABLED(AUTO_BED_LEVELING_UBL)  
// #undef MESH_MIN_X
// #undef MESH_MAX_X
// #undef MESH_MIN_Y
// #undef MESH_MAX_Y
#endif
//#define X_BED_SIZE (float)lcd_rts_data.bed_size_x
//#define Y_BED_SIZE (float)lcd_rts_data.bed_size_y
//#define X_MIN_POS  (float)lcd_rts_data.x_min_pos
//#define Y_MIN_POS  (float)lcd_rts_data.y_min_pos
//#define X_MAX_POS  (float)lcd_rts_data.x_max_pos
//#define Y_MAX_POS  (float)lcd_rts_data.y_max_pos
//#define Z_MAX_POS  (float)lcd_rts_data.z_max_pos
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #define PROBING_MARGIN (float)lcd_rts_data.probing_margin
  #define PROBING_MARGIN_LEFT (float)lcd_rts_data.probing_margin
  #define PROBING_MARGIN_RIGHT (float)lcd_rts_data.probing_margin
  #define PROBING_MARGIN_FRONT (float)lcd_rts_data.probing_margin
  #define PROBING_MARGIN_BACK (float)lcd_rts_data.probing_margin
#endif
//#define GRID_MIN 3
//#define GRID_MAX 10
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #define GRID_LIMIT lcd_rts_data.grid_max_points
  #define GRID_MAX_POINTS_X lcd_rts_data.grid_max_points
  #define GRID_MAX_POINTS_Y lcd_rts_data.grid_max_points
  #define GRID_MAX_POINTS (lcd_rts_data.grid_max_points * lcd_rts_data.grid_max_points)
#endif
#if ENABLED(AUTO_BED_LEVELING_UBL)  
//  #define MESH_MIN_X lcd_rts_data.mesh_min_x
//  #define MESH_MAX_X lcd_rts_data.mesh_max_x
//  #define MESH_MIN_Y lcd_rts_data.mesh_min_y
//  #define MESH_MAX_Y lcd_rts_data.mesh_max_y
#endif
#endif