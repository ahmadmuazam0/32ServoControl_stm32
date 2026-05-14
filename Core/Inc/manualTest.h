/*
 * manualTest.h
 *
 *  Created on: Jan 24, 2025
 *      Author: muazam-epazz
 */

#ifndef INC_MANUALTEST_H_
#define INC_MANUALTEST_H_

#include "main.h"

uint16_t B1press = 0;
bool B2press = false;
unsigned long debounceDelay = 100;

void readButton(void){
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET ){
		HAL_Delay(debounceDelay);
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET ){
		B1press++ ;}
		while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		};
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	}
	else if(HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET){
		HAL_Delay(debounceDelay);
		if(HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET){
		B2press = !B2press;}
		while(HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	else if(HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin) == GPIO_PIN_RESET){
			HAL_Delay(debounceDelay);
			if(HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin) == GPIO_PIN_RESET){
			B1press--;}
			while(HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin) == GPIO_PIN_RESET){
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			}
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}

}

inline void manual_testing(void){
	readButton();
		if (B2press) {
			switch(B1press){
			case 0:
				HAL_GPIO_WritePin(THUMB_1_GPIO_Port, THUMB_1_Pin, GPIO_PIN_RESET);
				break;
			case 1:
				HAL_GPIO_WritePin(THUMB_2_GPIO_Port, THUMB_2_Pin, GPIO_PIN_RESET);
				break;
			case 2:
				HAL_GPIO_WritePin(INDEX_1_GPIO_Port, INDEX_1_Pin, GPIO_PIN_RESET);
				break;
			case 3:
				HAL_GPIO_WritePin(INDEX_2_GPIO_Port, INDEX_2_Pin, GPIO_PIN_RESET);
				break;
			case 4:
				HAL_GPIO_WritePin(MIDDLE_1_GPIO_Port, MIDDLE_1_Pin, GPIO_PIN_RESET);
				break;
			case 5:
				HAL_GPIO_WritePin(MIDDLE_2_GPIO_Port, MIDDLE_2_Pin, GPIO_PIN_RESET);
				break;
			case 6:
				HAL_GPIO_WritePin(RING_1_GPIO_Port, RING_1_Pin, GPIO_PIN_RESET);
				break;
			case 7:
				HAL_GPIO_WritePin(RING_2_GPIO_Port, RING_2_Pin, GPIO_PIN_RESET);
				break;
			case 8:
				HAL_GPIO_WritePin(PINKY_1_GPIO_Port, PINKY_1_Pin, GPIO_PIN_RESET);
				break;
			case 9:
				HAL_GPIO_WritePin(PINKY_2_GPIO_Port, PINKY_2_Pin, GPIO_PIN_RESET);
				break;
			default:
				B1press = 0;
				break;
			}
		}
		else{
			HAL_GPIO_WritePin(THUMB_1_GPIO_Port, THUMB_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(THUMB_2_GPIO_Port, THUMB_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INDEX_1_GPIO_Port, INDEX_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(INDEX_2_GPIO_Port, INDEX_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MIDDLE_1_GPIO_Port, MIDDLE_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MIDDLE_2_GPIO_Port, MIDDLE_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RING_1_GPIO_Port, RING_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RING_2_GPIO_Port, RING_2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PINKY_1_GPIO_Port, PINKY_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PINKY_2_GPIO_Port, PINKY_2_Pin, GPIO_PIN_SET);
		}
}



#endif /* INC_MANUALTEST_H_ */
