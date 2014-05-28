
#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/motor.h>
#include <RASLib/inc/servo.h>


tBoolean led_on;
tServo *lmotor, *rmotor;
float left, right;

void blink(void) {
    SetPin(PIN_F2, led_on);

    led_on = !led_on;
}

float getval(void) {
    char ch;
    float val;
    ch = Getc();

    if (ch == '-') {
        val = -1;
        ch = Getc();
    } else { 
        val = 1;
    }

    if (ch == '1') {
        return ch;
    } else if (ch != '0') {
        return 0;
    }

    ch = Getc();

    if (ch != '.')
        return 0;

    ch = Getc();
    val *= (ch - '0');
    val /= 10.0f;

    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;

    return val/4;
}

int main(void) {  
    char ch;
    InitializeMCU();

    CallEvery(blink, 0, 0.25f);
    lmotor = InitializeServo(PIN_A6);
    rmotor = InitializeServo(PIN_A7);

go:
    if (Getc() != '{')
        goto go;

    Printf("l: ");
    left = getval();
    Printf("%f\n", left);    

    while (Getc() != ',')
        {Printf("no,\r");}

    Printf("r: ");
    right = getval();
    Printf("%f\n", right);    

    if ((ch = Getc()) != '}')
        {Printf("no} %c\r", ch);}

    
    SetServo(lmotor, (left+1.0f)/2.0f);
    SetServo(rmotor, (right+1.0f)/2.0f);
    goto go;
}
