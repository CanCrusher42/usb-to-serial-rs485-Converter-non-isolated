#pragma once
#ifndef SCAN_DISPLAY_H
#define SCAN_DISPLAY_H

void ConvertDisplayLineToRoom(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight, uint16_t x_scale , uint16_t y_scale);
void DisplayLineDistance(uint16_t startAngle, uint16_t endAngle, uint8_t minQuality, uint16_t maxHeight);

#endif
