#include <stdio.h>

void Recursive(int num)
{
	if(num<=0)	//Àç±ÍÀÇ Å»ÃâÁ¶°Ç
		return; //Àç±ÍÀÇ Å»Ãâ!

	printf("Recursive call! %d \n", num);
	Recursive(num-1);
}

int main(void)
{
	Recursive(3);
	return 0;
}

/*½ÇÇà°á°ú
Recursive call! 3
Recursive call! 2
Recursive call! 1
*/