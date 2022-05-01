#include <buttons.h>
#include <main.h>


void Key1Released(void){
    int localTest = GetTest();
    if (localTest > 0)
    {
        SetTest(5);
    }
    else
    {
        SetTest(1);
    }
        
}