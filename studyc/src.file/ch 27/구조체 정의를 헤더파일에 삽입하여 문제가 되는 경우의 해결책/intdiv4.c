#include "stdiv2.h"

Div IntDiv(int num1, int num2)
{
	Div dval;
	dval.quotient=num1/num2;
	dval.remainder=num1%num2;
	return dval;
}