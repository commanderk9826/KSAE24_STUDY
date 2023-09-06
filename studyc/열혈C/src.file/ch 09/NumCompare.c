#include <stdio.h>
int NumberCompare(int num1, int num2); //매개변수 이름 생략 -> int NumberCompare(int, int);

int main(void)
{
	printf("3과 4중에서 큰 수는 %d 이다. \n", NumberCompare(3, 4));
	printf("7과 2중에서 큰 수는 %d 이다. \n", NumberCompare(7, 2));
	return 0;
}

int NumberCompare(int num1, int num2)
{
	if(num1>num2)
		return num1; //중간에 얼마든지 return문이 올 수 있다.
	else
		return num2;
}
/*실행결과
3과 4중에서 큰 수는 4 이다.
7과 2중에서 큰 수는 7 이다.
*/