#define LIBRARY_TESTING 0

#include "cCompiler/mattCompiler.h"
#include "cCompiler/mattCompiler_library.c"
#include "led.h"
#include "motors.h"


int main (){
  Any a1 = Int(0);
  Any a0 = a1;
  Any a3 = Int(1);
  Any a2 = a3;
  Any a5 = Bool(true);
  Any a4 = a5;
  ledInit();
  Any a6 = a0;
  Any a7 = Int(1);
  ledSet( a6.i, a7.i );
  Any a8 = a2;
  Any a9 = Int(1);
  ledSet( a8.i, a9.i );
  loop_start_label0: ;
  Any a11 = Bool(true);
  if ( dataAsInt( a4 ) == dataAsInt( a11 ) ) { goto label1; };
  goto label0;
  label1: ;
  Any a12 = Bool(true);
  a4 = a12;
  goto loop_start_label0;
  label0: ;
  return 0;
}

