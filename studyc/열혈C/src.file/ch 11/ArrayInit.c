#include <stdio.h>

int main(void)
{
	int arr1[5]={1, 2, 3, 4, 5};
	int arr2[]={1, 2, 3, 4, 5, 6, 7};
	int arr3[5]={1, 2};
	int ar1Len, ar2Len, ar3Len, i;

	printf("�迭 arr1�� ũ��: %d \n", sizeof(arr1)); //sizeof ������ ����� �迭�� ����Ʈ ũ������ ��ȯ
	printf("�迭 arr2�� ũ��: %d \n", sizeof(arr2));
	printf("�迭 arr3�� ũ��: %d \n", sizeof(arr3));

	ar1Len = sizeof(arr1) / sizeof(int);     // �迭 arr1�� ���� ���
	ar2Len = sizeof(arr2) / sizeof(int);     // �迭 arr2�� ���� ���
	ar3Len = sizeof(arr3) / sizeof(int);     // �迭 arr3�� ���� ���->�迭�� ���̸� ����ϴ� ���
	
	for(i=0; i<ar1Len; i++)
		printf("%d ", arr1[i]); //�迭�̱⿡ for���� ���� ������ ������ �����ϴ�.
	printf("\n");

	for(i=0; i<ar2Len; i++)
		printf("%d ", arr2[i]);
	printf("\n");

	for(i=0; i<ar3Len; i++) //�ټ��� ������� �ݺ����� ���� ������ ���� �Ұ���!
		printf("%d ", arr3[i]);
	printf("\n");
	return 0;
}
/*������
�迭 arr1�� ũ�� : 20
�迭 arr2�� ũ�� : 28
�迭 arr3�� ũ�� : 20
1 2 3 4 5
1 2 3 4 5 6 7
1 2 0 0 0
*/