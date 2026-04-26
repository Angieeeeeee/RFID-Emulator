#include "clock.h"
#include "display_lib.h"
#include "gpio.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <stdint.h>

#define UP_PB 4
#define DOW_PB 5
#define OK_PB 6

uint8_t arrow_pos = 0;
// cursor movement
// x: 0 to 20
// y: 0 to 15

//Screen
//PA2 : SCL
//PA3 : CS
//PA5 : SDA
//PA6 : DC
//PA7 : RES
//
//PD0-3 : SPI1
//PB4-7 : SPI2

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

void main_menu() {

    ST7735_SetCursor(3, 1);
    ST7735_OutString("--- Main Menu ---");
    ST7735_SetCursor(1, 4);
    ST7735_OutString("Read Key");
    ST7735_SetCursor(1, 6);
    ST7735_OutString("Write Key");
    ST7735_SetCursor(1, 8);
    ST7735_OutString("Delete Key");
}

void Read_Key(){

    Output_Clear();
    ST7735_SetCursor(6, 6);
    ST7735_OutString("Reading...");
    waitMicrosecond(2e6);
    //here we would
    //wait(success)
    //then go back
    Output_Clear();
}

//TODO: both write and delete key will print the same key menu
void Write_Key(){

    Output_Clear();
    ST7735_SetCursor(3, 1);
    ST7735_OutString("---Key Menu---");
    //here we would display all keys that we have
    //loop over the keys and implement the select
    Output_Clear();

}

void Delete_Key(){

    Output_Clear();
    ST7735_SetCursor(3, 1);
    ST7735_OutString("---Key Menu---");
    //here we would display all keys that we have
    //loop over the keys and implement the select
    Output_Clear();

}
void checkButtonDebounced(uint8_t PB) {

    waitMicrosecond(1e5);
    while(1){
        if (getPinValue(PORTC, PB)) {
            waitMicrosecond(2e3);
            if(getPinValue(PORTC, PB))
                break;
        }
    }
}

void menu_controller(){
    switch (arrow_pos) {
        case 0:
            Read_Key();
        case 1:;
            Write_Key();
        default:
            Delete_Key();
    }
}




int main(void) {
    initSystemClockTo40Mhz();
    enablePort(PORTC);
    selectPinDigitalInput(PORTC, UP_PB);
    selectPinDigitalInput(PORTC, DOW_PB);
    selectPinDigitalInput(PORTC, OK_PB);

    Output_Init();
    ST7735_SetRotation(1);
    ST7735_SetTextColor(ST7735_GREEN);

reset:
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
            menu_controller();
            checkButtonDebounced(OK_PB);
            goto reset;
        }
    }
}







