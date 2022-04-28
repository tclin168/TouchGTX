

//gui---Screen1View.cpp
#include "stm32f7xx_hal.h" //include the HAL library to your board.

// LED Toggle PJ5 function
void Screen1View::ToggleLED()
{
	if(toggleButton1.getState()) HAL_GPIO_WritePin(GPIOJ,GPIO_PIN_5, GPIO_PIN_SET);
	//get the state of the toggle button. If the state is true, we will turn on LED.

	else HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_5, GPIO_PIN_RESET);
	//else the LED will be off.
}


//Screen1View.hpp
class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void ToggleLED(); //add the ToggleLED() function

protected:
};