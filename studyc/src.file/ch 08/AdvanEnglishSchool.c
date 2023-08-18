#include <stdio.h>

int main(void)
{
	char sel;
	printf("M 오전, A 오후, E 저녁 \n");
	printf("입력: ");		
	scanf("%c", &sel);
	
	switch(sel)
	{
	case 'M': // case 'M': case 'm': -> 다음과 같이 두 case 레이블을 한 줄에 같이 표시하기도 함.
	case 'm':
		printf("Morning \n");
		break;	
	case 'A': // case 'A': case 'a':
	case 'a':
		printf("Afternoon \n");
		break;
	case 'E': // case 'E': case 'e':
	case 'e':
		printf("Evening \n");
		break;  // 사실 불필요한 break문! 
	}
	return 0;
}
