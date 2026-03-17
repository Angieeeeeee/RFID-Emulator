#include "tm4c123gh6pm.h"
#include "display_lib.h"

int main(void) {

    Output_Init();
    ST7735_SetRotation(1);
    ST7735_SetTextColor(ST7735_GREEN);
    ST7735_SetCursor(1, 1);

    // Step 4
    ST7735_OutString("Hello Angelina.");

    while(1) {
    }
}

