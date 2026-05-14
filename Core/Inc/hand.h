#ifndef __HAND_H__
#define __HAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"
#include "stm32f1xx_hal.h"

#define FINGER_Delay 1000
#define ARM_Delay 3000
#define NUMBER_OF_FINGERS 5
// Enum for finger states (open or close)
typedef enum {
    FINGER_OPEN = 0,
    FINGER_CLOSE
} FingerState;

typedef enum {
    THUMB,
    INDEX,
    MIDDLE,
    RING,
    PINKY
} FingerIndex_t;
/**
 * @brief Class representing a Finger
 */
class Finger {
private:
	FingerState fState;
    FingerIndex_t fIndex;
public:
    /**
     * @brief Get the current state of the finger
     * @return Current state of the finger (OPEN or CLOSE)
     */
    Finger(){
    	fState = FINGER_OPEN;
    }
    ~Finger();
    FingerState getFingerState() const;
    void setFingerState(FingerState fs) ;

};

/**
 * @brief Class representing a Hand with five fingers
 */
class Hand {
private:
    Finger fingers[NUMBER_OF_FINGERS]; ///< Array of fingers: index, middle, ring, pinky, thumb
public:
    /**
     * @brief Constructor to initialize a hand with all five fingers and put the hand in rest position
     * @param fingerPorts Array of GPIO port base addresses for the fingers
     * @param fingerPins Array of GPIO pin numbers for the fingers
     */
    Hand(){
    }

    /**
     * @brief Destructor to clean up resources
     */
    ~Hand();

    /**
     * @brief Open all fingers of the hand
     */
    void openAll();

    /**
     * @brief Close all fingers of the hand
     */
    void closeAll();

    /**
     * @brief Open a specific finger by index
     * @param fingerIndex Index of the finger to open (0-4)
     */
    void openFinger(FingerIndex_t fingerIndex);

    /**
     * @brief Close a specific finger by index
     * @param fingerIndex Index of the finger to close (0-4)
     */
    void closeFinger(FingerIndex_t fingerIndex);
    /**
     * @breif: Open the ARM - Biscep
     * @param: None
     */
    void openArm(void);
    /**
	 * @breif: Close the ARM - Biscep
	 * @param: None
	 */
	void closeArm(void);

//    /**
//     * @brief set the finger status
//     * @param fIndex Index of the FInger
//     * @param gpioPort Port of the GPIO
//     * @param gpiopin GPIO Pin
//     * @param fState State of finger to be set FINGER_OPEN/FINGER_CLOSE
//     */
//    void setFingerState(FingerIndex_t fIndex, GPIO_TypeDef *gpioPort, uint16_t gpioPin, FingerState fState);
};

#ifdef __cplusplus
}
#endif

#endif  // __HAND_H__
