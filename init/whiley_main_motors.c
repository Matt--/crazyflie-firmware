#define LIBRARY_TESTING 0

#include "cCompiler/mattCompiler.h"
#include "cCompiler/mattCompiler_library.c"
#include "led.h"
#include "motors.h"


int main (){
  Any a1 = Bool(true);
  Any a0 = a1;
  ledInit (  );
  motorsInit (  );
  Any a2 = Int(0);
  motorsTestTask ( a2.i );
  loop_start_label0: ;
  Any a4 = Bool(true);
  if ( dataAsInt( a0 ) == dataAsInt( a4 ) ) { goto label1; };
  goto label0;
  label1: ;
  Any a5 = Bool(true);
  a0 = a5;
  goto loop_start_label0;
  label0: ;
  return 0;
}

