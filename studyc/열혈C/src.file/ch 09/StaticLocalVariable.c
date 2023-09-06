#include <stdio.h>

void SimpleFunc(void)
{
	static int num1=0;	//초기화하지 않으면 0 초기화
	int num2=0;			//초기화하지 않으면 쓰레기 값 초기화
	num1++, num2++;
	printf("static: %d, local: %d \n",num1, num2);
}

int main(void)
{
	int i;
	for(i=0; i<3; i++)
		SimpleFunc();
	return 0;
}

/*실행결과
static: 1 local:1
static: 2 local:1
static: 3 local:1
*/