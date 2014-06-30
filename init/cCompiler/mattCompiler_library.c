// Library constructors & functions


//// Data struct types
// any new constructor, also put into destructor "dataAsInt"
Any Int(int i)
{
	Any a;
	a.i = i;
	a.type = INT_TYPE;
	return a;
}
Any Real(double r)
{
	Any a;
	a.r = r;
	a.type = REAL_TYPE;
	return a;
}
Any Char(char c)
{
	Any a;
	a.c = c;
	a.type = CHAR_TYPE;
	return a;
}
Any Str(char* s)
{
	if(strlen(s) > MAXCHAR) {
		char message [200];
		sprintf(message, "method failed, char string over %d chars : Any Str(char* c); The string is: %s", MAXCHAR, s);
		error(1, message);
	}
	Any a;
	memset(a.s, 0, sizeof(a.s)); // TODO shouldn't need this, remove later
	strcpy(a.s, s);
	a.type = STR_TYPE;
	return a;
}
Any Bool(bool b)
{
	Any a;
	a.b = b;
	a.type = BOOL_TYPE;
	return a;
}
//Any Byte(char* b)
//{
//	Any a;
//	a.byte = b;
//	a.type = BYTE_TYPE;
//	return a;
//}
Any Null()
{
	Any a;
	//a.b = b;
	a.type = NULL_TYPE;
	return a;
}
// TODO is there a way to take 2+ parameters in one Tuple constructor?
Any Tuple(Any a, Any b)
{
	/*
	 * kept for my reference only.
	 * static reserves memory in this function only
	 * malloc reserves memory for the programs duration or until free()
	 *	static Any x; x = a;
	 *	static Any y; y = b;
	 */

	Any *x = malloc(sizeof(*x));
	Any *y = malloc(sizeof(*y));
	*x = a;
	*y = b;

	Any d;
	d.type = TUPLE_TYPE;
	d.ptr = malloc( sizeof(d.ptr) * 2 );
	Any **any = (Any**)d.ptr;
	any[0] = x;
	any[1] = y;
	return d;
}

/**
 * Frees all memory reserved by this tuple. Including any
 * values held. ie: copy them before calling this.
 * TODO extend to handle recursion?
 */
bool freeTuple(Any a){
	if(a.type != TUPLE_TYPE) return true;
	// A pointer to an array of pointers
	Any **any = (Any**)a.ptr;
	int size = sizeof(a.ptr)/4;
	// free the pointers in the array
	int i = 0;
	while(i < size){
		free(any[i++]);
	}
	// free the array pointer
	free(a.ptr);
	return true;
}

//Any Tuple3(Any a, Any b, Any c)
//{
//	Any d;
//	d.type = TUPLE_TYPE;
//	d.ptr = malloc(3);
//	Any** any = (Any**)d.ptr;
//	any[0] = &a;
//	any[1] = &b;
//	any[2] = &c;
//	return d;
//}

//Any AnyCopy(Any a)
//{
//	Any d;
//	d.type = a.type;
//	d.data = a.data;
//	return d;
//}




Any toStr(Any a){
//	Any* x_address = ((Any**)a.ptr)[0];
//		printf ("x address is %d\n", x_address);
//		printf ("x is %d\n", x_address->i);
//	int y = ((Any**)a.ptr)[1]->i;
//		printf ("y is %d\n", y);
//	printf("type is : %d\n", a.type);
//	printf("ptr size is : %d\n", sizeof(a.ptr));

	if(a.type == NULL_TYPE){
		return Str("null");
	}
	if(a.type == INT_TYPE){
		char str [MAXCHAR+1];
		sprintf(str, "%d", a.i);
		Any b = Str(str);
		return b;
	}
	if(a.type == REAL_TYPE){
		char str [MAXCHAR+1];
		sprintf(str, "%f", a.r);
		Any b = Str(str);
		return b;
	}
	if(a.type == CHAR_TYPE){
		char str [MAXCHAR+1];
		sprintf(str, "%c", a.c);
		Any b = Str(str);
		return b;
	}
	if(a.type == STR_TYPE){
		return a;
	}
	if(a.type == BOOL_TYPE){
		if(a.b == 0)
			return Str("false");
		else
			return Str("true");
	}
	if(a.type == TUPLE_TYPE){
		// TODO only copes with 2 and 3 tuples.
		// TODO only prints int values, extend to other types.

		char str [MAXCHAR+1];

		if(sizeof(a.ptr)/4 == 2)
			sprintf(str, "(%d, %d)", ((Any**)a.ptr)[0]->i, ((Any**)a.ptr)[1]->i);
		if(sizeof(a.ptr)/4 == 3)
			sprintf(str, "(%d, %d, %d)", ((Any**)a.ptr)[0]->i, ((Any**)a.ptr)[1]->i, ((Any**)a.ptr)[2]->i);
		Any b = Str(str);
		return b;
	}
	error(1, "error, toStr(Any); type unknown");

	return Str("");
}

int dataAsInt(Any a){
	if(a.type == NULL_TYPE){ return 0; }
	if(a.type == INT_TYPE){ return a.i; }
	if(a.type == CHAR_TYPE){ return (int)a.c; }
//	if(a.type == STR_TYPE){ int r = (int)a.s; return r; } //NB pointer cast to int
	if(a.type == BOOL_TYPE){ return (int)a.b; }

	#if(LIBRARY_TESTING)
	error(1, "error, dataAsInt(Any); type unknown");
	#endif

	return 0;
}

//// Math operations
Any add(Any x, Any y)
{
    if(x.type != INT_TYPE || y.type != INT_TYPE){
    	error(1, "method failed, not adding these ints : Any add(Any, Any)");
    }
    return Int(x.i + y.i);
}
Any sub(Any x, Any y)
{
    if(x.type != INT_TYPE || y.type != INT_TYPE){
      error(1, "method failed, not subtracting ints : Any sub(Any, Any)");
    }
    return Int(x.i - y.i);
}
Any mul(Any x, Any y)
{
    if(x.type != INT_TYPE || y.type != INT_TYPE){
      error(1, "method failed, not multiplying ints : Any mul(Any, Any)");
    }
    return Int(x.i * y.i);
}
Any div_wyce(Any x, Any y)
{
    if(x.type != INT_TYPE || y.type != INT_TYPE){
      error(1, "method failed, not dividing ints : Any div(Any, Any)");
    }
    if(y.i == 0){
      error(1, "method failed, divisor == 0 : Any div(Any, Any)");
    }
    return Int(x.i / y.i);
}
Any Copy(Any a)
{
	return a;
}
// end Data struct types

//// Helpers
#if(LIBRARY_TESTING)
void println(Any a)
{
    if(a.type == INT_TYPE){
      printf("%d\n", a.i); }
    if(a.type == REAL_TYPE){
      printf("%f\n", a.r); }
    else if (a.type == CHAR_TYPE){
      printf("%c\n", a.c); }
    else if (a.type == STR_TYPE){
      printf("%s\n", a.s); }
    else if (a.type == BOOL_TYPE && a.b == 0){
      printf("false\n"); }
    else if (a.type == BOOL_TYPE && a.b == 1){
      printf("true\n"); }
}

void error(int error_no, char c [200])
{
  printf("    %s\n", c);
  if(!LIBRARY_TESTING) { exit(error_no); }
  //exit(error_no);
}
#endif
// end Helpers



