#include <stdio.h>

void Recursive(int num)
{
	if(num<=0)	//����� Ż������
		return; //����� Ż��!

	printf("Recursive call! %d \n", num);
	Recursive(num-1);
}

int main(void)
{
	Recursive(3);
	return 0;
}

/*������
Recursive call! 3
Recursive call! 2
Recursive call! 1
*/