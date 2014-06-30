
#include "led.h"

int main(){

	ledInit();

	ledSet(LED_RED, 1);
	ledSet(LED_GREEN, 1);

  while(1);
  return 0;
}
