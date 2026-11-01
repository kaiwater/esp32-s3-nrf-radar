#pragma once

#include "radar_view.h"
#include "scan_view.h"

void st7735_init(void);
void st7735_draw_radar(const radar_view_t *v);
void st7735_draw_scan(const scan_view_t *v);
void st7735_next_theme(void);
int st7735_button(void);
