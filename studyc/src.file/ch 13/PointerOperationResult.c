#include <stdio.h>

int main(void)
{
	int * ptr1=0x0010; //�޸� �ּ� 0x0010�� �ʱ� ������ ����
	double * ptr2=0x0010;

	printf("%p %p \n", ptr1+1, ptr1+2); //ptr1�� ����Ű�� �޸� �ּҿ� 1�� 2�� ���� �ּҸ� �����. ������ ������ �ش� ������ Ÿ���� ũ�⸸ŭ ������.
	printf("%p %p \n", ptr2+1, ptr2+2);

	printf("%p %p \n", ptr1, ptr2);
	ptr1++;
	ptr2++;
	printf("%p %p \n", ptr1, ptr2);
	return 0;
}
/*������
00000014 00000018
00000018 00000020
00000010 00000010
00000014 00000018
*/
