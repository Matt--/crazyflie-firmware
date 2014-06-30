#define LIBRARY_TESTING 1

#include "mattCompiler.h"
#include "mattCompiler_library.c"

//// Tests
int main()
{
  printf("\nTesting Whiley Intermediate to C Compiler Library.\n");
  printf("Author : Matt Stevens, Honours student, Victoria University 2014\n\n");

  printf("Tests starting................................\n");
	// Primitive data types
  // Whiley allows type conversion and union types. Structs allow for this.
	Any a;
	// test size of char array cannot be excepted when Char struct created. 
  printf("a = Char(\"A string, len 16\"); --> ok\n");
    a = Char("A string, len 16");
  printf("a = Char(\"A string, len 19 xx\"); --> ok\n");
    a = Char("A string, len 19 xx");
  printf("a = Char(\"A string, len 20 xxx\"); --> ok\n");
    a = Char("A string, len 20 xxx"); 
  printf("a = Char(\"A string, len 21 xxxx\"); --> error expected\n");
    a = Char("A string, len 21 xxxx"); 
  printf("a = Char(\"A string, len 24 xxxxxxx\"); --> error expected\n");
    a = Char("A string, len 24 xxxxxxx"); 
	
  printf("Tests finished................................\n\n");
}


