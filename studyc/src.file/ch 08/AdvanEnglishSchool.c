#include <stdio.h>

int main(void)
{
	char sel;
	printf("M ����, A ����, E ���� \n");
	printf("�Է�: ");		
	scanf("%c", &sel);
	
	switch(sel)
	{
	case 'M': // case 'M': case 'm': -> ������ ���� �� case ���̺��� �� �ٿ� ���� ǥ���ϱ⵵ ��.
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
		break;  // ��� ���ʿ��� break��! 
	}
	return 0;
}
