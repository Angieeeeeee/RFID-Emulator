#include "clock.h"
#include "display_lib.h"
#include "gpio.h"
#include "rfid.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <stdint.h>
#include <stdio.h>

#define UP_PB  4
#define DOW_PB 5
#define OK_PB  6

#define MAIN_MENU_ITEMS 4   // Read, Write, Delete, Inspect

uint8_t arrow_pos = 0;

// cursor movement
// x: 0 to 20
// y: 0 to 15  (but on this GREENTAB panel in landscape, the visible
//              region effectively ends around y=10. Keep all content
//              in y = 0 .. 10 to be safe.)

// Screen
// PA2 : SCL
// PA3 : CS
// PA5 : SDA
// PA6 : DC
// PA7 : RES
//
// PD0-3 : SPI1
// PB4-7 : SPI2

typedef enum {
    ACT_WRITE,
    ACT_DELETE,
    ACT_INSPECT
} MenuAction;

//-----------------------------------------------------------------------------
// Layout constants — kept in one place so retuning the layout is a
// one-line change. ARROW_X is the column where every "<--" gets drawn.
//-----------------------------------------------------------------------------
#define TITLE_Y       0
#define MAIN_FIRST_Y  2     // first menu item row
#define MAIN_STEP     2     // rows between menu items
#define LIST_FIRST_Y  2     // first row in the key list
#define LIST_STEP     1     // tight spacing so 8 entries + Back fit
#define ARROW_X       13

//-----------------------------------------------------------------------------
// Helpers
//-----------------------------------------------------------------------------

// Output_Clear() only paints the screen black — it does NOT reset the
// driver's StX/StY cursor globals. Pair the fill-black with an explicit
// cursor reset so subsequent prints land where we expect.
static void clear_screen(void) {
    Output_Clear();
    ST7735_SetCursor(0, 0);
}

void checkButtonDebounced(uint8_t PB) {
    waitMicrosecond(1e5);
    while (1) {
        if (getPinValue(PORTC, PB)) {
            waitMicrosecond(2e3);
            if (getPinValue(PORTC, PB))
                break;
        }
    }
}

static void wait_for_ok(void) {
    while (getPinValue(PORTC, OK_PB));   // wait for press (active low)
    checkButtonDebounced(OK_PB);          // wait for release
}

//-----------------------------------------------------------------------------
// Main menu
//-----------------------------------------------------------------------------

static void draw_arrow(uint8_t pos) {
    ST7735_SetCursor(ARROW_X, MAIN_FIRST_Y + pos * MAIN_STEP);
    ST7735_OutString("<--");
}

static void erase_arrow(uint8_t pos) {
    ST7735_SetCursor(ARROW_X, MAIN_FIRST_Y + pos * MAIN_STEP);
    ST7735_OutString("   ");
}

// direction == -1 moves the arrow DOWN, anything else moves it UP
// (kept the same sign convention as the original code)
static void update_arrow_pos(int8_t direction) {
    erase_arrow(arrow_pos);
    if (direction == -1) {
        arrow_pos = (arrow_pos + 1) % MAIN_MENU_ITEMS;
    } else {
        arrow_pos = (arrow_pos == 0) ? (MAIN_MENU_ITEMS - 1) : (arrow_pos - 1);
    }
    draw_arrow(arrow_pos);
}

void main_menu(void) {
    ST7735_SetCursor(3, TITLE_Y);
    ST7735_OutString("--- Main Menu ---");
    ST7735_SetCursor(1, MAIN_FIRST_Y + 0 * MAIN_STEP);   // y = 2
    ST7735_OutString("Read Key");
    ST7735_SetCursor(1, MAIN_FIRST_Y + 1 * MAIN_STEP);   // y = 4
    ST7735_OutString("Write Key");
    ST7735_SetCursor(1, MAIN_FIRST_Y + 2 * MAIN_STEP);   // y = 6
    ST7735_OutString("Delete Key");
    ST7735_SetCursor(1, MAIN_FIRST_Y + 3 * MAIN_STEP);   // y = 8
    ST7735_OutString("Inspect Key");
}

//-----------------------------------------------------------------------------
// Read action
//-----------------------------------------------------------------------------

void Read_Key(void) {
    clear_screen();
    ST7735_SetCursor(6, 5);
    ST7735_OutString("Reading...");
    waitMicrosecond(1e6);

    int8_t key_index = readRFID();

    clear_screen();
    if (key_index >= 0) {
        ST7735_SetCursor(1, 5);
        ST7735_OutString(rfidTable[key_index].name);
        ST7735_OutString(" Read");
    } else if (key_index == -1) {
        ST7735_SetCursor(2, 5);
        ST7735_OutString("No Tag Detected");
    } else {
        // Replaced the original \n-terminated strings with explicit
        // SetCursor calls — \n in this driver advances StY *and* paints
        // a 21-space line at the new row, leaving the cursor in an
        // unpredictable place.
        ST7735_SetCursor(5, 4);
        ST7735_OutString("No Space");
        ST7735_SetCursor(3, 6);
        ST7735_OutString("Delete a Key");
    }

    waitMicrosecond(2e6);
    clear_screen();
}

//-----------------------------------------------------------------------------
// Shared key list selector
//
// Used by Write / Delete / Inspect. Builds a compact list of all populated
// rfidTable entries followed by a "Back" option, lets the user navigate
// with UP/DOW and pick with OK.
//
// Returns:
//   >= 0  table index of the selected entry
//    -1   user picked Back
//    -2   no entries to show (empty list screen was shown, OK dismissed)
//-----------------------------------------------------------------------------

static int8_t key_menu_select(const char *title) {
    int8_t display_to_table[MAX_RFID_ENTRIES];
    int8_t entry_count = 0;
    uint8_t i;

    for (i = 0; i < MAX_RFID_ENTRIES; i++) {
        if (rfidTable[i].hasData) {
            display_to_table[entry_count++] = (int8_t)i;
        }
    }

    // Empty list: just show a message and wait for OK
    if (entry_count == 0) {
        clear_screen();
        ST7735_SetCursor(3, TITLE_Y);
        ST7735_OutString(title);
        ST7735_SetCursor(2, 4);
        ST7735_OutString("No keys stored");
        ST7735_SetCursor(2, 7);
        ST7735_OutString("OK to go back");
        wait_for_ok();
        return -2;
    }

    int8_t total_options = entry_count + 1;   // +1 for the Back row
    int8_t local_pos = 0;

    // Initial render. Spacing of LIST_STEP=1 keeps the list inside the
    // visible area even when the list is full (8 entries + Back = 9
    // rows from y=2 to y=10).
    clear_screen();
    ST7735_SetCursor(3, TITLE_Y);
    ST7735_OutString(title);
    for (i = 0; i < entry_count; i++) {
        ST7735_SetCursor(1, LIST_FIRST_Y + i * LIST_STEP);
        ST7735_OutString(rfidTable[display_to_table[i]].name);
    }
    ST7735_SetCursor(1, LIST_FIRST_Y + entry_count * LIST_STEP);
    ST7735_OutString("Back");
    ST7735_SetCursor(ARROW_X, LIST_FIRST_Y + local_pos * LIST_STEP);
    ST7735_OutString("<--");

    while (1) {
        if (!getPinValue(PORTC, UP_PB)) {
            ST7735_SetCursor(ARROW_X, LIST_FIRST_Y + local_pos * LIST_STEP);
            ST7735_OutString("   ");
            local_pos = (local_pos == 0) ? (total_options - 1) : (local_pos - 1);
            ST7735_SetCursor(ARROW_X, LIST_FIRST_Y + local_pos * LIST_STEP);
            ST7735_OutString("<--");
            checkButtonDebounced(UP_PB);
        } else if (!getPinValue(PORTC, DOW_PB)) {
            ST7735_SetCursor(ARROW_X, LIST_FIRST_Y + local_pos * LIST_STEP);
            ST7735_OutString("   ");
            local_pos = (local_pos + 1) % total_options;
            ST7735_SetCursor(ARROW_X, LIST_FIRST_Y + local_pos * LIST_STEP);
            ST7735_OutString("<--");
            checkButtonDebounced(DOW_PB);
        } else if (!getPinValue(PORTC, OK_PB)) {
            checkButtonDebounced(OK_PB);
            if (local_pos == entry_count) {
                return -1;   // Back
            }
            return display_to_table[local_pos];
        }
    }
}

//-----------------------------------------------------------------------------
// Inspect — show UID and metadata for one entry
//-----------------------------------------------------------------------------

static void inspect_display(int8_t idx) {
    char buf[20];

    clear_screen();
    ST7735_SetCursor(3, TITLE_Y);
    ST7735_OutString("--- Inspect ---");

    ST7735_SetCursor(1, 2);
    ST7735_OutString("Name: ");
    ST7735_OutString(rfidTable[idx].name);

    // UID as hex bytes (4 bytes from the table; the 5th is the BCC)
    ST7735_SetCursor(1, 4);
    ST7735_OutString("UID: ");
    uint8_t i;
    uint8_t shown = (rfidTable[idx].uidLength > 4) ? 4 : rfidTable[idx].uidLength;
    for (i = 0; i < shown; i++) {
        sprintf(buf, "%02X ", rfidTable[idx].uid[i]);
        ST7735_OutString(buf);
    }

    // 32-bit packed ID — what writeRFID() takes as its argument
    ST7735_SetCursor(1, 6);
    ST7735_OutString("ID:  ");
    sprintf(buf, "%08lX", (unsigned long)rfidTable[idx].id);
    ST7735_OutString(buf);

    ST7735_SetCursor(1, 8);
    ST7735_OutString("Blocks: ");
    sprintf(buf, "%d", rfidTable[idx].blockCount);
    ST7735_OutString(buf);

    ST7735_SetCursor(1, 10);
    ST7735_OutString("OK to go back");

    wait_for_ok();
}

//-----------------------------------------------------------------------------
// key_menu — single dispatcher used by Write / Delete / Inspect
//-----------------------------------------------------------------------------

static void key_menu(MenuAction action) {
    const char *title;
    switch (action) {
        case ACT_WRITE:   title = "--- Write Key ---";   break;
        case ACT_DELETE:  title = "--- Delete Key ---";  break;
        case ACT_INSPECT: title = "-- Inspect Key --";   break;
        default:          title = "--- ??? ---";         break;
    }

    int8_t idx = key_menu_select(title);
    if (idx < 0) return;   // Back or empty list

    switch (action) {
        case ACT_WRITE: {
            clear_screen();
            ST7735_SetCursor(2, 4);
            ST7735_OutString("Place target on");
            ST7735_SetCursor(2, 5);
            ST7735_OutString("writer, then OK");
            wait_for_ok();

            clear_screen();
            ST7735_SetCursor(4, 5);
            ST7735_OutString("Writing...");
            waitMicrosecond(5e5);

            uint8_t status = writeRFID(rfidTable[idx].id);

            clear_screen();
            if (status == STATUS_OK) {
                ST7735_SetCursor(4, 5);
                ST7735_OutString("Write OK");
            } else {
                char buf[20];
                sprintf(buf, "Write Fail: %d", status);
                ST7735_SetCursor(2, 5);
                ST7735_OutString(buf);
            }
            waitMicrosecond(2e6);
            break;
        }
        case ACT_DELETE: {
            rfidTable[idx].hasData = 0;
            if (rfidCount > 0) rfidCount--;
            clear_screen();
            ST7735_SetCursor(5, 5);
            ST7735_OutString("Deleted");
            waitMicrosecond(1e6);
            break;
        }
        case ACT_INSPECT: {
            inspect_display(idx);
            break;
        }
    }
}

//-----------------------------------------------------------------------------
// Top level
//-----------------------------------------------------------------------------

void menu_controller(void) {
    switch (arrow_pos) {
        case 0: Read_Key();             break;
        case 1: key_menu(ACT_WRITE);    break;
        case 2: key_menu(ACT_DELETE);   break;
        case 3: key_menu(ACT_INSPECT);  break;
    }
}

int main(void) {
    initSystemClockTo40Mhz();
    enablePort(PORTC);

    selectPinDigitalInput(PORTC, UP_PB);
    selectPinDigitalInput(PORTC, DOW_PB);
    selectPinDigitalInput(PORTC, OK_PB);

    initRC();

    uint8_t v2 = rc522GetVersion(RC522_2);
    char vbuf[20];
    sprintf(vbuf, "RC2 ver: %02X", v2);
    Output_Init();
    ST7735_SetRotation(1);
    ST7735_SetTextColor(ST7735_GREEN);
    clear_screen();
    ST7735_SetCursor(1, 5);
    ST7735_OutString(vbuf);
    waitMicrosecond(3e6);

    Output_Init();
    ST7735_SetRotation(1);
    ST7735_SetTextColor(ST7735_GREEN);
    waitMicrosecond(1e6);

reset:
    clear_screen();
    main_menu();
    draw_arrow(arrow_pos);

    while (1) {
        if (!getPinValue(PORTC, UP_PB)) {
            update_arrow_pos(1);
            checkButtonDebounced(UP_PB);
        } else if (!getPinValue(PORTC, DOW_PB)) {
            update_arrow_pos(-1);
            checkButtonDebounced(DOW_PB);
        } else if (!getPinValue(PORTC, OK_PB)) {
            // Debounce OK FIRST, before dispatching. Otherwise the
            // sub-menu's input loop sees OK still pressed from the
            // main menu and immediately auto-selects its first item.
            checkButtonDebounced(OK_PB);
            menu_controller();
            goto reset;
        }
    }
}
