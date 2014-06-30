
#include "led.h"
#include "motors.h"


int main(){

	ledInit();
	motorsInit();

	motorsTestTask(0);

  return 0;
}
