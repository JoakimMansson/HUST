#pragma once

#include <SPI.h>
#include "Platform.h"
#include "App_Common.h"


void Write_Text(Gpu_Hal_Context_t *phost,  int x, int y, int size, char text[]);
void Finish_Display(Gpu_Hal_Context_t *phost);
void Start_Set_Display(Gpu_Hal_Context_t *phost);
void Start_Set_Display(Gpu_Hal_Context_t *phost);
void insert_line(Gpu_Hal_Context_t *phost, int x1, int x2, int y1, int y2, int linewidth);
void insert_single_line(Gpu_Hal_Context_t *phost, int x1, int x2, int y1, int y2, int linewidth, int r, int g, int b);
void draw_rect(Gpu_Hal_Context_t *phost, int x1, int y1, int x2, int y2, int r, int g, int b);
