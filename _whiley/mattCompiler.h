#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#include <stdbool.h>
//#include <math.h>

#define NULL_TYPE 0
#define INT_TYPE 1
#define CHAR_TYPE 2
#define BOOL_TYPE 3
#define STR_TYPE 4
#define REAL_TYPE 5
#define BYTE_TYPE 6
#define TUPLE_TYPE 7
#define POINTER_TYPE 8
#define FPTR_TYPE 9

#define FUNCPARAMS_ZERO (Any(*)())
#define FUNCPARAMS_ONE (Any(*)(Any))
#define FUNCPARAMS_TWO (Any(*)(Any, Any))
#define FUNCPARAMS_THREE (Any(*)(Any, Any, Any))
#define FUNCPARAMS_FOUR (Any(*)(Any, Any, Any, Any))
#define FUNCPARAMS_FIVE (Any(*)(Any, Any, Any, Any, Any))
#define FUNCPARAMS_SIX (Any(*)(Any, Any, Any, Any, Any, Any))
#define FUNCPARAMS_SEVEN (Any(*)(Any, Any, Any, Any, Any, Any, Any))

#define MAXCHAR 40

//typedef struct {
//	void (*ptr)(void);
//	int  params;
//}wyce_Func;

typedef struct {
    int type;
    union {
    	long i;
    	double r;
    	bool b;
    	char byte[8+1];
    	char c;
    	char s[MAXCHAR+1];
    	void *ptr;
//    	wyce_Func f;
    };
} Any;

//void println(Any);
void error(int, char*);

/**** Math ops ****/
//Any wyce_add(Any, Any);
//Any wyce_sub(Any, Any);
//Any wyce_mul(Any, Any);
//Any wyce_div(Any, Any);
//
//Any toStr(Any);


