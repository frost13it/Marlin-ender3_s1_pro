/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfig.h"

#if HAS_MEDIA

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../lcd/marlinui.h"

#if ENABLED(E3S1PRO_RTS)
  #include "../../lcd/rts/e3s1pro/lcd_rts.h"
  #include "../../lcd/rts/e3s1pro/preview.h"
  int action;
#endif

/**
 * M19: Open a file readonly without starting a print
 *
 * The path is relative to the root directory
 */
void GcodeSuite::M19() {
#if ENABLED(E3S1PRO_RTS)
if (parser.seenval('S')) action = parser.value_int();
const char* filename_input = "OCTODGUS.GCO";
const char* filename_output = "TEMP.GCO";

// Declare outputCopy here
char outputCopy[strlen(filename_output) + 1];

switch(action){
    case 1: {
        char *ptr = nullptr;  // Declare ptr here
        char *end = nullptr;  // Declare end here
        if (!card.fileExists(filename_input)) {
            #if ENABLED(LCD_RTS_DEBUG_SDCARD)
                SERIAL_ECHO_MSG("Input file does not exist: ", filename_input);
            #endif
            return;        
        }
        card.openFileReadonly(filename_input);
        if (!card.isFileOpen()) {
            #if ENABLED(LCD_RTS_DEBUG_SDCARD)
                SERIAL_ECHO_MSG("Failed to open input file:", filename_input);
            #endif
            return;
        }
        size_t fileLength = card.getFileSize();
        if (fileLength > 24500) {
            #if ENABLED(LCD_RTS_DEBUG_SDCARD)
                SERIAL_ECHO_MSG("filesize too big. max 24500 bytes allowed");
            #endif
            return;    
        }
        char fileContents[fileLength + 1];
        size_t bytesRead = card.read((uint8_t*)fileContents, fileLength);
        fileContents[bytesRead] = '\0';
        card.closefile();
        char* lastThumbnailEnd = nullptr;
        const char* endMarkers[] = {"; thumbnail end", "; thumbnail_JPG end", "; jpg end"};
        for (const char* marker : endMarkers) {
            char* found = strstr(fileContents, marker);
            if (found) {
                lastThumbnailEnd = found;
            }
        }
        if (lastThumbnailEnd != nullptr) {
            lastThumbnailEnd[0] = '\0';
        }
        ptr = strstr(fileContents, "M4010");
        while (ptr != nullptr) {
            size_t replaceLength = strlen(ptr);
            *ptr = ';';
            char *curPtr = ptr + 5;
            while (*curPtr) {
                if (*curPtr == ' ') {
                    break;
                }
                curPtr++;
            }
            if (*curPtr) {
                memmove(ptr + 1, curPtr, replaceLength - 4);
            } else {
                memset(ptr + replaceLength, '\0', 1);
            }
            ptr = strstr(ptr + 1, "M4010");
        }
        end = fileContents + strlen(fileContents) - 1;
        while (end >= fileContents and (*end == ' ' || *end == '\n' || *end == '\r')) {
            *end = '\0';
            end--;
        }

        // Copy filename_output to outputCopy
        strcpy(outputCopy, filename_output);
        
        card.openFileWrite(outputCopy);
        if (!card.isFileOpen()) {
            #if ENABLED(LCD_RTS_DEBUG_SDCARD)
                SERIAL_ECHO_MSG("Failed to open output file");
            #endif
            return;
        }
        card.write((uint8_t*)fileContents, strlen(fileContents));
        card.closefile();
        int32_t ret = gcodePicDataOctoPrintSendToDwin(outputCopy, VP_OVERLAY_PIC_PTINT, PIC_FORMAT_JPG, PIC_RESOLUTION_250_250);
        #if ENABLED(LCD_RTS_DEBUG_SDCARD)
            SERIAL_ECHO_MSG("Thumbnail load via M19 ret status: ", ret);
        #endif
        if (ret == PIC_OK) {
            RTS_ResetPrintData(false);
            RTS_SendPrintData();
            RTS_ResetProgress();
            RTS_ShowPage(1);
        } else {              
            RTS_ResetPrintData(true);
        }
        card.removeFile(filename_input);
        card.removeFile(outputCopy);          
    break;
    }
    case 2:
        RTS_CleanPrintAndSelectFile();
        RTS_ResetPrintData(true);
        RTS_ResetProgress();
        RTS_SendM600Icon(false);
        RTS_SendM73Icon(false);
        lcd_rts_settings.external_m73 = false;
        RTS_ShowPage(1);
        #if ENABLED(LCD_RTS_DEBUG_LCD)
            SERIAL_ECHO_MSG("M19 CANCELLED arrived S", action);
        #endif
        return;                
    break;
    case 3:
        RTS_SendPrintData();
        RTS_LoadMeshPointOffsets();
        RTS_LoadMainsiteIcons();
        RTS_SendLevelingSiteData();
        RTS_ShowPage(10);
        #if ENABLED(LCD_RTS_DEBUG_LCD)
            SERIAL_ECHO_MSG("M19 OCTORUN arrived S", action);
        #endif
        return;
    break;
    case 4:
        RTS_ShowPage(12);
        #if ENABLED(LCD_RTS_DEBUG_LCD)
            SERIAL_ECHO_MSG("M19 PAUSED arrived S", action);
        #endif
        return;
    break;
    case 5:
        RTS_ShowPage(10);
        #if ENABLED(LCD_RTS_DEBUG_LCD)
            SERIAL_ECHO_MSG("M19 RESUMED arrived S", action);
        #endif
        return;
    break;
    case 6:
        RTS_SendM600Icon(false);                
        RTS_SendM73Icon(false);
        lcd_rts_settings.external_m73 = false;        
        RTS_ShowPage(9);
        #if ENABLED(LCD_RTS_DEBUG_LCD)
            SERIAL_ECHO_MSG("M19 PRINTDONE arrived S", action);
        #endif
        return;
    break;
    default:
        #if ENABLED(LCD_RTS_DEBUG_LCD)
            SERIAL_ECHO_MSG("Input argument is not known S", action);
        #endif
        return;
    break;
}

#endif

}

#endif // HAS_MEDIA