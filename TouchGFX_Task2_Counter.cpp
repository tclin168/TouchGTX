/* Step-1 
Set the button on the TouchGFX Designer, such as the button and the wildcard

Texts-Typographies-Wildcard--Characters: -,+

\*


/* Step-2 
write our function 

in the file (application-User-gui-Screen1View.cpp)

void Screen1View::downclicked()
{
	counter--;
	//if (counter <=0) counter = 0;
	//Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%u", counter);
	Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%d", counter);
	textArea1.invalidate();

}

void Screen1View::upclicked()
{
    counter++;
    //Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%u", counter);
    Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%d", counter);
    textArea1.invalidate();
}

\*

/* Step-3
define the function and the variable
in the file (Screen1View.hpp)

public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void downclicked();
    virtual void upclicked();

protected:

    int16_t counter = 0; //creat the variable to track the count

};
\*