#include <stdio.h>

int main(void)
{
	int * ptr1=0x0010; //메모리 주소 0x0010을 초기 값으로 설정
	double * ptr2=0x0010;

	printf("%p %p \n", ptr1+1, ptr1+2); //ptr1이 가리키는 메모리 주소에 1과 2를 더한 주소를 출력함. 포인터 연산은 해당 데이터 타입의 크기만큼 증가됨.
	printf("%p %p \n", ptr2+1, ptr2+2);

	printf("%p %p \n", ptr1, ptr2);
	ptr1++;
	ptr2++;
	printf("%p %p \n", ptr1, ptr2);
	return 0;
}
/*실행결과
00000014 00000018
00000018 00000020
00000010 00000010
00000014 00000018
*/
