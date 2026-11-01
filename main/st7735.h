#pragma once

#include <stdbool.h>

#include "radar_view.h"

void st7735_init(void);
void st7735_draw_radar(const radar_view_link_t *links, unsigned nlinks,
                       unsigned modules, unsigned channel,
                       bool motion, bool paused, bool calibrating);
int st7735_button(void);
