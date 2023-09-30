#include <SPI.h>
#include "Platform.h"
#include "App_Common.h"
#include "ScreenConfiguration.h"


/* 
* This function is used for writing text on the screen. 
* It takes the GPU context, position (x, y), text size, and the text string as parameters.
*/
void Write_Text(Gpu_Hal_Context_t *phost, int x, int y, int size, char text[])
{
  App_WrDl_Buffer(phost, BEGIN(BITMAPS));
  App_WrDl_Buffer(phost, COLOR_RGB(1, 1, 1));
  int increment = 8;

  for(int i = 0; text[i] != '\0'; i++)
  {
    
    App_WrDl_Buffer(phost, VERTEX2II(x, y, size, text[i]));

    if(isUpperCase(text[i])){
      x += size;
    } else if(text[i] == 'r' || text[i] == 'i') {
      x += size - size/3*1.5;
    } else  if(text[i] == 'm') {
      x += size + 5;
    } else { 
      x += size - increment;
    }
  }
  App_WrDl_Buffer(phost, END());
}


/* This function finishes the display list, 
* sets the display color, and displays the content on the screen. 
*/
void Finish_Display(Gpu_Hal_Context_t *phost)
{
    App_WrDl_Buffer(phost, END());
    App_WrDl_Buffer(phost, COLOR_RGB(255, 1, 1));
    App_WrDl_Buffer(phost, DISPLAY());
        
    Gpu_Hal_DLSwap(phost,DLSWAP_FRAME);
    App_Flush_DL_Buffer(phost);    
}

/* Starts a display list */
void Start_Set_Display(Gpu_Hal_Context_t *phost)
{
  App_WrDl_Buffer(phost, CLEAR_COLOR_RGB(255, 255, 255));
  App_WrDl_Buffer(phost, CLEAR(1, 1, 1));
  ///App_WrDl_Buffer(phost, BEGIN(BITMAPS));
}

/* Creates a box on the screen */
void insert_line(Gpu_Hal_Context_t *phost, int x1, int x2, int y1, int y2, int linewidth) {
  App_WrDl_Buffer(phost, BEGIN(LINES));
  App_WrDl_Buffer(phost, LINE_WIDTH(linewidth));
  App_WrDl_Buffer(phost, COLOR_RGB(10, 50, 10));
  
  App_WrDl_Buffer(phost, VERTEX2II(x1, y1, 0, 0));
  App_WrDl_Buffer(phost, VERTEX2II(x1, y2, 0, 0));
  
  App_WrDl_Buffer(phost, VERTEX2II(x1, y1, 0, 0));
  App_WrDl_Buffer(phost, VERTEX2II(x2, y1, 0, 0));
  
  App_WrDl_Buffer(phost, VERTEX2II(x2, y2, 0, 0));
  App_WrDl_Buffer(phost, VERTEX2II(x2, y1, 0, 0));
  
  App_WrDl_Buffer(phost, VERTEX2II(x2, y2, 0, 0));
  App_WrDl_Buffer(phost, VERTEX2II(x1, y2, 0, 0));

  App_WrDl_Buffer(phost, VERTEX2II(x2, y1, 0, 0));
  App_WrDl_Buffer(phost, VERTEX2II(x1, y1, 0, 0));


  
  App_WrDl_Buffer(phost, END());
}


/*
* This function inserts a single line with specified parameters, 
* such as position, line width, and color.
*/
void insert_single_line(Gpu_Hal_Context_t *phost, int x1, int x2, int y1, int y2, int linewidth, int r, int g, int b) {
  App_WrDl_Buffer(phost, BEGIN(LINES));
  App_WrDl_Buffer(phost, LINE_WIDTH(linewidth));
  App_WrDl_Buffer(phost, COLOR_RGB(r, g, b));
  
  App_WrDl_Buffer(phost, VERTEX2II(x1, y1, 0, 0));
  App_WrDl_Buffer(phost, VERTEX2II(x2, y2, 0, 0));  
  App_WrDl_Buffer(phost, END());
}


void draw_rect(Gpu_Hal_Context_t *phost, int x1, int y1, int x2, int y2, int r, int g, int b) {
  App_WrDl_Buffer(phost, BEGIN(RECTS));
  App_WrDl_Buffer(phost, COLOR_RGB(r, g, b));
  App_WrDl_Buffer(phost, LINE_WIDTH(100));
  
  /* Using * 16 becuase input to VERTEX2F is 1/16 pixel*/    
  App_WrDl_Buffer(phost, VERTEX2F(x1 * 16, y1 * 16));
  App_WrDl_Buffer(phost, VERTEX2F(x2 * 16, y2 * 16));

  App_WrDl_Buffer(phost, END());
}



