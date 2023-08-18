#include <stdio.h>

int main(void)
{
	int arr1[5]={1, 2, 3, 4, 5};
	int arr2[]={1, 2, 3, 4, 5, 6, 7};
	int arr3[5]={1, 2};
	int ar1Len, ar2Len, ar3Len, i;

	printf("배열 arr1의 크기: %d \n", sizeof(arr1)); //sizeof 연산의 결과로 배열의 바이트 크기정보 반환
	printf("배열 arr2의 크기: %d \n", sizeof(arr2));
	printf("배열 arr3의 크기: %d \n", sizeof(arr3));

	ar1Len = sizeof(arr1) / sizeof(int);     // 배열 arr1의 길이 계산
	ar2Len = sizeof(arr2) / sizeof(int);     // 배열 arr2의 길이 계산
	ar3Len = sizeof(arr3) / sizeof(int);     // 배열 arr3의 길이 계산->배열의 길이를 계산하는 방식
	
	for(i=0; i<ar1Len; i++)
		printf("%d ", arr1[i]); //배열이기에 for문을 통한 숫자적 접근이 가능하다.
	printf("\n");

	for(i=0; i<ar2Len; i++)
		printf("%d ", arr2[i]);
	printf("\n");

	for(i=0; i<ar3Len; i++) //다수의 변수라면 반복문을 통한 순차적 접근 불가능!
		printf("%d ", arr3[i]);
	printf("\n");
	return 0;
}
/*실행결과
배열 arr1의 크기 : 20
배열 arr2의 크기 : 28
배열 arr3의 크기 : 20
1 2 3 4 5
1 2 3 4 5 6 7
1 2 0 0 0
*/