#include <stdio.h>

int main(void)
{
	char str[50];
	int idx=0;

	printf("���ڿ� �Է�: ");
	scanf("%s", str); // ���ڿ��� �Է� �޾Ƽ� �迭 str�� ����!
	printf("�Է� ���� ���ڿ�: %s \n", str);

	printf("���� ���� ���: ");
	while(str[idx] != '\0')
	{
		printf("%c", str[idx]);
		idx++;
	}
	// scanf �Լ��� ȣ���� ���ؼ� �Է� ���� ���ڿ��� ������ �� ���ڰ� �������� Ȯ���ϱ� ���� ����
	printf("\n");	
	return 0;
}
/*������
���ڿ� �Է� : He is my friend
�Է� ���� ���ڿ� : He
���� ���� ��� : He
*/