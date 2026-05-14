/*
 * hand.cpp
 *
 *  Created on: Jan 20, 2025
 *      Author: muazam-epazz
 */

#include "hand.h"
#include "gpio.h"
#include "main.h"
/**
 * @ brief Get the current state of the Finger
 *
 */
 FingerState Finger::getFingerState() const{
	/// It has to use the Flex sensor to know the position of the finger
	// Till then, we're reading pin state of specific finger to get the state of the finger
	return fState;
}

 /**
  * @ brief Set the current state of the Finger
  *
  */
  void Finger::setFingerState(FingerState fs) {
 	fState = fs;
 }


 /**
  * Function declaration of the Hand Class
  */
/**
 * #breif: To open the specific finger
 * @param: The index of the finger to be operated
 */
 void Hand::openFinger(FingerIndex_t fingerIndex){
//	 if (fingers[fingerIndex].getFingerState() == FINGER_CLOSE) {
		 switch (fingerIndex) {
			case THUMB:
				HAL_GPIO_WritePin(THUMB_1_GPIO_Port, THUMB_1_Pin, GPIO_PIN_RESET);
				HAL_Delay(ARM_Delay);
				HAL_GPIO_WritePin(THUMB_1_GPIO_Port, THUMB_1_Pin, GPIO_PIN_SET);
				break;
			case INDEX:
				HAL_GPIO_WritePin(INDEX_1_GPIO_Port, INDEX_1_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(INDEX_1_GPIO_Port, INDEX_1_Pin, GPIO_PIN_SET);
				break;
			case MIDDLE:
				HAL_GPIO_WritePin(MIDDLE_1_GPIO_Port, MIDDLE_1_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(MIDDLE_1_GPIO_Port, MIDDLE_1_Pin, GPIO_PIN_SET);
				break;
			case RING:
				HAL_GPIO_WritePin(RING_1_GPIO_Port, RING_1_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(RING_1_GPIO_Port, RING_1_Pin, GPIO_PIN_SET);
				break;
			case PINKY:
				HAL_GPIO_WritePin(PINKY_1_GPIO_Port, PINKY_1_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(PINKY_1_GPIO_Port, PINKY_1_Pin, GPIO_PIN_SET);
				break;
			default:
				break;
//		}
	 }

//		fingers[fingerIndex].setFingerState(FINGER_OPEN);	// Update the FInger State to OPEN
 }

 /**
  * @breif: To close the specific finger
  * @pram: The index of the finger to be operated
  */
 void Hand::closeFinger(FingerIndex_t fingerIndex){
//	 if (fingers[fingerIndex].getFingerState() == FINGER_OPEN) {
		 switch (fingerIndex) {
			case THUMB:
				HAL_GPIO_WritePin(THUMB_2_GPIO_Port, THUMB_2_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(THUMB_2_GPIO_Port, THUMB_2_Pin, GPIO_PIN_SET);
				break;
			case INDEX:
				HAL_GPIO_WritePin(INDEX_2_GPIO_Port, INDEX_2_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(INDEX_2_GPIO_Port, INDEX_2_Pin, GPIO_PIN_SET);
				break;
			case MIDDLE:
				HAL_GPIO_WritePin(MIDDLE_2_GPIO_Port, MIDDLE_2_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(MIDDLE_2_GPIO_Port, MIDDLE_2_Pin, GPIO_PIN_SET);
				break;
			case RING:
				HAL_GPIO_WritePin(RING_2_GPIO_Port, RING_2_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(RING_2_GPIO_Port, RING_2_Pin, GPIO_PIN_SET);
				break;
			case PINKY:
				HAL_GPIO_WritePin(PINKY_2_GPIO_Port, PINKY_2_Pin, GPIO_PIN_RESET);
				HAL_Delay(FINGER_Delay);
				HAL_GPIO_WritePin(PINKY_2_GPIO_Port, PINKY_2_Pin, GPIO_PIN_SET);
				break;
			default:
				break;
//		}
	 }

//		fingers[fingerIndex].setFingerState(FINGER_CLOSE);
 }

 /**
  * @breif To open the HAND
  * @param NONE
  */
 void Hand::openAll(){
	 GPIOA->ODR &= 	 ~((1<<3)|(1<<5)|(1<<7));	// Set on Pin 3,5 & 7
	 GPIOB->ODR &=	 ~((1<<1)/*| (1<<11)*/);		// SET the pin 1 & 11.
	 HAL_Delay(FINGER_Delay);
	 GPIOA->ODR |=  ((1<<3)|(1<<5)|(1<<7));	//RESET the pin 3,5 & 7 from the BSRR register of Port A
	 GPIOB->ODR |=  ((1<<1) /*| (1<<11)*/);			//RESET the pin 1 & 11 from BSRR register of Port B

//	 for(int i = 0; i< 5; i++){
//		 fingers[i].setFingerState(FINGER_OPEN);		// Set the current state of HAND
//	 }

 }


 /**
  * @breif To close the HAND
  * @param NONE
  */
 void Hand::closeAll(){
	 GPIOA->ODR |= 	 ((1<<2)|(1<<4)|(1<<6));	// Set on Pin 2,4 & 7
	 GPIOB->ODR |=	 ((1<<0) /*| (1<<10)*/);		// SET the pin 0 & 10
	 HAL_Delay(FINGER_Delay);
	 GPIOA->ODR &=  ~((1<<2)|(1<<4)|(1<<6));	//RESET the pin 3,5 & 7 from the BSRR register of Port A
	 GPIOB->ODR &=  ~((1<<0) /*| (1<<10)*/);			//RESET the pin 1 & 11 from BSRR register of Port B

//	 for(int i = 0; i< 5; i++){
//		 fingers[i].setFingerState(FINGER_CLOSE);		// Set the current state of HAND
//	 }

 }
 /**
  * @breif To close the ARM
  * @param NONE
  */

 //***********************************THUMB PIN HAS TO BE CHANGED TO ARM PIN**************************//
 void Hand::closeArm(){
	 HAL_GPIO_WritePin(THUMB_1_GPIO_Port, THUMB_1_Pin, GPIO_PIN_RESET);
	 HAL_Delay(3000);
	 HAL_GPIO_WritePin(THUMB_1_GPIO_Port, THUMB_1_Pin, GPIO_PIN_SET);
 }
 /**
  * @breif To Open the ARM
  * @param NONE
  */

 //***********************************THUMB PIN HAS TO BE CHANGED TO ARM PIN**************************//
 void Hand::openArm(){
 	 HAL_GPIO_WritePin(THUMB_2_GPIO_Port, THUMB_2_Pin, GPIO_PIN_RESET);
 	 HAL_Delay(3000);
 	 HAL_GPIO_WritePin(THUMB_2_GPIO_Port, THUMB_2_Pin, GPIO_PIN_SET);
  }


