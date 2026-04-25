#include "clock.h"
#include "display_lib.h"
#include "gpio.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <stdint.h>

#define UP_PB 0
#define DOW_PB 1
uint8_t arrow_pos = 0;
// cursor movement
// x: 0 to 20
// y: 0 to 15


void draw_arrow(uint8_t pos) {
    // 3 positions 0,1,2.
    // updates position depending on the buttons pressed
    ST7735_SetCursor(13, 4 + pos * 2);
    ST7735_OutString("<--");
}

void delete_arrow(){
    ST7735_SetCursor(13, 4 + arrow_pos * 2);
    ST7735_OutString("   ");
}

void update_arrow_pos(int8_t pos) {

    delete_arrow();

    if (pos == -1){
        if (arrow_pos == 2)
            arrow_pos = 0;
        else
            ++arrow_pos;

    }else{
        if(arrow_pos == 0)
            arrow_pos = 2;
        else
            --arrow_pos;
    }

    draw_arrow(arrow_pos);
}

void menu_text() {

    ST7735_SetCursor(3, 1);
    ST7735_OutString("--- Main Menu ---");
    ST7735_SetCursor(1, 4);
    ST7735_OutString("Read Key");
    ST7735_SetCursor(1, 6);
    ST7735_OutString("Write Key");
    ST7735_SetCursor(1, 8);
    ST7735_OutString("Delete Key");
}

void checkButtonDebounced(uint8_t PB) {

    waitMicrosecond(1e5);
    while(1){
        if (getPinValue(PORTD, PB)) {
            waitMicrosecond(2e3);
            if(getPinValue(PORTD, PB))
                break;
        }
    }
}

int main(void) {
    initSystemClockTo40Mhz();
    enablePort(PORTD);
    selectPinDigitalInput(PORTD, UP_PB);
    selectPinDigitalInput(PORTD, DOW_PB);

    Output_Init();
    ST7735_SetRotation(1);
    ST7735_SetTextColor(ST7735_GREEN);

    menu_text();
    draw_arrow(arrow_pos);

    while (1) {

        if (!getPinValue(PORTD, UP_PB)) {
            update_arrow_pos(1);
            checkButtonDebounced(UP_PB);

        } else if (!getPinValue(PORTD, DOW_PB)) {
            update_arrow_pos(-1);
            checkButtonDebounced(DOW_PB);
        }
    }
}

