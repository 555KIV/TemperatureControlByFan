/*
 * filter_sma.c
 *
 *  Created on: Dec 8, 2023
 *      Author: a555k
 */
#include "filter_sma.h"

uint16_t Filter_Buffer[FILTER_SMA_ORDER] = {0,};

uint16_t Filter_Sma(uint16_t For_Filtered)
{
	Filter_Buffer[FILTER_SMA_ORDER-1] = For_Filtered;

	uint32_t Output = 0;

	for (uint8_t i = 0; i<FILTER_SMA_ORDER;i++)
	{
		Output+=Filter_Buffer[i];
	}

	Output /= FILTER_SMA_ORDER;

	for (uint8_t i = 0; i<FILTER_SMA_ORDER;i++)
	{
		Filter_Buffer[i]=Filter_Buffer[i+1];
	}

	return (uint16_t)Output;
}

uint16_t y = 0;
const float k = 0.55;

uint16_t Filter_Exp(uint16_t For_filtered)
{
	y = (y*k)+For_filtered*(1-k);

	return y;
}

