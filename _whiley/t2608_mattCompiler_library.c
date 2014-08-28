/*
 * Library constructors & functions
 */


/**** Data struct types ****/
/* any new constructor, also put into destructor "dataAsInt" */
Any Int(int i)
{
	Any a;
	a.type = INT_TYPE;
	a.i = i;
	return a;
}
Any Real(double r)
{
	Any a;
	a.type = REAL_TYPE;
	a.r = r;
	return a;
}
Any Char(char c)
{
	Any a;
	a.type = CHAR_TYPE;
	a.c = c;
	return a;
}
Any Ptr( void *c)
{
	Any a;
	a.type = POINTER_TYPE;
	a.ptr = c;
	return a;
}
Any Fptr( void* address, int count )
{
	Any a;
	a.type = FPTR_TYPE;
	a.f.ptr = address; /* generates a warning, void* to func*. Ignore */
	a.f.params = count;
	return a;
}
Any Str(char* s)
{
#if(LIBRARY_TESTING)
	if(strlen(s) > MAXCHAR) {
		char message [200];
		sprintf(message, "method failed, char string over %d chars : Any Str(char* c); The string is: %s", MAXCHAR, s);
		error(1, message);
	}
#endif
	Any a;
	a.type = STR_TYPE;
	memset(a.s, 0, sizeof(a.s)); /* TODO shouldn't need this, remove later */
	strcpy(a.s, s);
	return a;
}
Any Bool(bool b)
{
	Any a;
	a.type = BOOL_TYPE;
	a.b = b;
	return a;
}
Any Null()
{
	Any a;
	a.type = NULL_TYPE;
	//a.b = b;
	return a;
}

Any Tuple(Any a, Any b)
{
	/*
	 * kept for reference only.
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

Any toStr(Any a){
	if(a.type == NULL_TYPE){
		return Str("null");
	}
	if(a.type == INT_TYPE){
		char str [MAXCHAR+1];
		sprintf(str, "%ld", a.i);
		Any b = Str(str);
		return b;
	}
	if(a.type == REAL_TYPE){
		char str [MAXCHAR+1];
		sprintf(str, "%g", a.r); /* %g to remove trailing 0's */
		if(strchr(str, '.') == NULL){ strcat(str, ".0"); }
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
		char str[MAXCHAR+1];
		sprintf(str, "\"%s\"", a.s);
		Any b = Str(str);
		return b;
	}
	if(a.type == BOOL_TYPE){
		if(a.b == 0)
			return Str("false");
		else
			return Str("true");
	}
#if(LIBRARY_TESTING)
	error(1, "error, toStr(Any); type unknown");
#endif
	return Str("");
}

int dataAsInt(Any a){
	if(a.type == NULL_TYPE){ return 0; }
	if(a.type == INT_TYPE){ return a.i; }
	if(a.type == REAL_TYPE){ return a.r; }
	if(a.type == CHAR_TYPE){ return (int)a.c; }
	if(a.type == BOOL_TYPE){ return (int)a.b; }

	#if(LIBRARY_TESTING)
	error(1, "error, dataAsInt(Any); type unknown");
	#endif

	return 0;
}


Any recordToStr1(char* s1, Any a){
	Any as = toStr(a);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, "}");
	return Str(str);
}

Any recordToStr2(char* s1, Any a, char* s2, Any b){
	Any as = toStr(a);
	Any bs = toStr(b);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, ",");
	strcat(str, s2);
	strcat(str, ":");
	strcat(str, bs.s);
	strcat(str, "}");
	return Str(str);
}

Any recordToStr3(char* s1, Any a, char* s2, Any b, char* s3, Any c){
	Any as = toStr(a);
	Any bs = toStr(b);
	Any cs = toStr(c);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, ",");
	strcat(str, s2);
	strcat(str, ":");
	strcat(str, bs.s);
	strcat(str, ",");
	strcat(str, s3);
	strcat(str, ":");
	strcat(str, cs.s);
	strcat(str, "}");
	return Str(str);
}

Any recordToStr4(char* s1, Any a, char* s2, Any b, char* s3, Any c, char* s4, Any d){
	Any as = toStr(a);
	Any bs = toStr(b);
	Any cs = toStr(c);
	Any ds = toStr(d);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, ",");
	strcat(str, s2);
	strcat(str, ":");
	strcat(str, bs.s);
	strcat(str, ",");
	strcat(str, s3);
	strcat(str, ":");
	strcat(str, cs.s);
	strcat(str, ",");
	strcat(str, s4);
	strcat(str, ":");
	strcat(str, ds.s);
	strcat(str, "}");
	return Str(str);
}

Any recordToStr5(char* s1, Any a, char* s2, Any b, char* s3, Any c, char* s4, Any d, char* s5, Any e){
	Any as = toStr(a);
	Any bs = toStr(b);
	Any cs = toStr(c);
	Any ds = toStr(d);
	Any es = toStr(e);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, ",");
	strcat(str, s2);
	strcat(str, ":");
	strcat(str, bs.s);
	strcat(str, ",");
	strcat(str, s3);
	strcat(str, ":");
	strcat(str, cs.s);
	strcat(str, ",");
	strcat(str, s4);
	strcat(str, ":");
	strcat(str, ds.s);
	strcat(str, ",");
	strcat(str, s5);
	strcat(str, ":");
	strcat(str, es.s);
	strcat(str, "}");
	return Str(str);
}

Any recordToStr6(char* s1, Any a, char* s2, Any b, char* s3, Any c, char* s4, Any d, char* s5, Any e, char* s6, Any f){
	Any as = toStr(a);
	Any bs = toStr(b);
	Any cs = toStr(c);
	Any ds = toStr(d);
	Any es = toStr(e);
	Any fs = toStr(f);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, ",");
	strcat(str, s2);
	strcat(str, ":");
	strcat(str, bs.s);
	strcat(str, ",");
	strcat(str, s3);
	strcat(str, ":");
	strcat(str, cs.s);
	strcat(str, ",");
	strcat(str, s4);
	strcat(str, ":");
	strcat(str, ds.s);
	strcat(str, ",");
	strcat(str, s5);
	strcat(str, ":");
	strcat(str, es.s);
	strcat(str, ",");
	strcat(str, s6);
	strcat(str, ":");
	strcat(str, fs.s);
	strcat(str, "}");
	return Str(str);
}

Any recordToStr7(char* s1, Any a, char* s2, Any b, char* s3, Any c, char* s4, Any d, char* s5, Any e, char* s6, Any f, char* s7, Any g){
	Any as = toStr(a);
	Any bs = toStr(b);
	Any cs = toStr(c);
	Any ds = toStr(d);
	Any es = toStr(e);
	Any fs = toStr(f);
	Any gs = toStr(g);
	char str [MAXCHAR+1];
	sprintf(str, "%c", '{');
	strcat(str, s1);
	strcat(str, ":");
	strcat(str, as.s);
	strcat(str, ",");
	strcat(str, s2);
	strcat(str, ":");
	strcat(str, bs.s);
	strcat(str, ",");
	strcat(str, s3);
	strcat(str, ":");
	strcat(str, cs.s);
	strcat(str, ",");
	strcat(str, s4);
	strcat(str, ":");
	strcat(str, ds.s);
	strcat(str, ",");
	strcat(str, s5);
	strcat(str, ":");
	strcat(str, es.s);
	strcat(str, ",");
	strcat(str, s6);
	strcat(str, ":");
	strcat(str, fs.s);
	strcat(str, ",");
	strcat(str, s7);
	strcat(str, ":");
	strcat(str, gs.s);
	strcat(str, "}");
	return Str(str);
}


/**** Math operations ****/
Any wyce_add(Any x, Any y)
{
	if(x.type != y.type){
#if(LIBRARY_TESTING)
		error(1, "method error, adding two different types : Any add(Any, Any)");
#endif
	}
	switch(x.type){
	case INT_TYPE:	return Int(x.i + y.i);
	case REAL_TYPE:	return Real(x.r + y.r);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, cannot add this type : Any add(Any, Any)");
#endif
	}
	return Null();
}
Any wyce_sub(Any x, Any y)
{
	if(x.type != y.type){
#if(LIBRARY_TESTING)
		error(1, "method error, subtracting two different types : Any sub(Any, Any)");
#endif
	}
	switch(x.type){
	case INT_TYPE:	return Int(x.i - y.i);
	case REAL_TYPE:	return Real(x.r - y.r);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, cannot subtract this type : Any sub(Any, Any)");
#endif
	}
	return Null();
}
Any wyce_neg(Any x)
{
	switch(x.type){
	case INT_TYPE:	return Int(-x.i);
	case REAL_TYPE:	return Real(-x.r);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, cannot establish negative for this type : Any neg(Any)");
#endif
	}
	return Null();
}
Any wyce_mul(Any x, Any y)
{
	if(x.type != y.type){
#if(LIBRARY_TESTING)
		error(1, "method error, multiplyinging two different types : Any mul(Any, Any)");
#endif
	}
	switch(x.type){
	case INT_TYPE:	return Int(x.i * y.i);
	case REAL_TYPE:	return Real(x.r * y.r);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, cannot multiply this type : Any mul(Any, Any)");
#endif
	}
	return Null();
}
Any wyce_div(Any x, Any y)
{
	switch(x.type){
	case INT_TYPE:
		if(y.i == 0){
#if(LIBRARY_TESTING)
			error(1, "method failed, divisor == 0 : Any div(Any, Any)");
#endif
		}
		return Int(x.i / y.i);
	case REAL_TYPE:
		if(y.r == 0){
#if(LIBRARY_TESTING)
			error(1, "method failed, divisor == 0 : Any div(Any, Any)");
#endif
		}
		return Real(x.r / y.r);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, cannot divide this type : Any div(Any, Any)");
#endif
	}
	return Null();
}
Any wyce_mod(Any x, Any y)
{
	if(x.type != y.type){
#if(LIBRARY_TESTING)
		error(1, "method error, moding two different types : Any mod(Any, Any)");
#endif
	}

	switch(x.type){
	case INT_TYPE:
		if(y.i == 0){
#if(LIBRARY_TESTING)
			error(1, "method failed, divisor == 0 : Any mod(Any, Any)");
#endif
		}
		return Int(x.i % y.i);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, cannot mod this type : Any mod(Any, Any)");
#endif
	}
	return Null();
}

Any Copy(Any a)
{
	switch(a.type){
	case(NULL_TYPE):	return Null();
	case(INT_TYPE):		return Int(a.i);
	case(REAL_TYPE):	return Real(a.r);
	case(CHAR_TYPE):	return Char(a.c);
	case(BOOL_TYPE):	return Bool(a.b);
	case(STR_TYPE):		return Str(a.s);
	case(POINTER_TYPE):	return Ptr(a.ptr);
#if(LIBRARY_TESTING)
	default:
		error(1, "method failed, type not catered for : Any Copy(Any)");
#endif
	}
	Any copy;
	return copy;
}


/**** Helpers ****/
#if(LIBRARY_TESTING)
void print(Any);

void println(Any a){
	print(a);
	printf("\n");
}

void print(Any a)
{
	/*
	 * Printing chars. Daves test Char_Valid_2 requires a char to print with single quotes.
	 * The bytecode turns the char type to a string before this method is called.
	 * Making identifying a char at this point problematic.
	 * Changing the toStr for char, creates problems elsewhere...
	 */

	Any str;
	if(a.type == STR_TYPE){
		str = a;
	} else { // turn to string first
		str = toStr(a);
	}
	printf("%s", str.s);
}

void error(int error_no, char c [200])
{
  printf("    %s\n", c);
  if(!LIBRARY_TESTING) { exit(error_no); }
}
#endif




