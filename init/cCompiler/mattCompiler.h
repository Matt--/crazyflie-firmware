#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define NULL_TYPE 0
#define INT_TYPE 1
#define CHAR_TYPE 2
#define BOOL_TYPE 3
#define STR_TYPE 4
#define REAL_TYPE 5
#define BYTE_TYPE 6
#define TUPLE_TYPE 7

#define MAXCHAR 40


typedef struct {
    int type;
    union{
    	int i;
    	double r;
    	bool b;
    	char byte[8+1];
    	char c;
    	char s[MAXCHAR+1];
    	void *ptr;
//   	struct Any **anyPtr;
    };
} Any;

void println(Any);
Any Int(int);
void error(int, char*);

// Math ops
Any add(Any, Any);
Any sub(Any, Any);
Any mul(Any, Any);
Any div_wyce(Any, Any);


