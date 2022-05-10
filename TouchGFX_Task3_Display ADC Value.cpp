/* Step-1 
in the file Application-gui-Screen1view.hpp

#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void handleTickEvent(); //this function is basically called every frame, so its rate depends on display

protected:

};

#endif // SCREEN1VIEW_HPP



\*


/* Step-2 
in the file Application-gui-Screen1view.cpp

#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

extern uint16_t Angle;
extern uint16_t Angle1;
extern uint16_t Get_Value;

void Screen1View::handleTickEvent()
{
	//Value.setValue(Angle1);
	circleProgress1.setValue(Angle);
	boxProgress1.setValue(Angle);
	textProgress1.setValue(Angle);

	Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%u", Get_Value);
	textArea1.invalidate();
}

\*

/* Step-3
define the function and the variable
in the file (Application-User-TouchGFX)

/* USER CODE BEGIN 0 */

uint16_t Angle = 0; //create a variable to keep the track of this angle
uint16_t Angle1 = 0;
uint16_t Get_Value = 0;

/* USER CODE END 0 */


/* USER CODE END Header_StartadcTask */
void StartadcTask(void *argument)
{
  /* USER CODE BEGIN StartadcTask */

	uint16_t ADC_VAL; //define a variable to store the adc value

  /* Infinite loop */
  for(;;)
  {
	  //function to read the ADC value and convert it to angle
	  HAL_ADC_Start (&hadc1); // Start ADC
	  HAL_ADC_PollForConversion (&hadc1, 100);
	  ADC_VAL = HAL_ADC_GetValue (&hadc1);
	  //HAL_ADC_Stop (&hadc1);

	  Angle = ADC_VAL;
	  Get_Value = ADC_VAL ;
	  Angle = ADC_VAL*300/4095 ;

    osDelay(100);
  }
  /* USER CODE END StartadcTask */
}


\*