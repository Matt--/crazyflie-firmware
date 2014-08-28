/*
 * Library constructors & functions
 */


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
//	memset(a.s, 0, sizeof(a.s)); /* TODO shouldn't need this, remove later */
//	strcpy(a.s, s);
	return a;
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



/**** Helpers ****/
#if(LIBRARY_TESTING)

void whileyPrecision(char * str){
  if(strchr(str, '.') == NULL){ return; }
  int i = strlen(str);
  if( str[i] != '.' ) {
    while( i > 0 ){
      if( str[i] == '.' ) { str[i+1] = '0'; break; }
      if( str[i] != '0' && str[i] != '\0') { break; }
      str[i--] = '\0';
    }
  };
}
#endif


