#include <iostream>
#include <algorithm>
using namespace std;

void stuGrade(const double* pArr, int num, double& rSum, double& rAve, double& rMax);

//������ �̻��
/*int main(void) {
	int n;
	cout << "�Է��� �л�(����) ��: ";
	cin >> n;
	int score[50];
	int max = 0;
	for (int i = 0; i < n; i++) {
		cin >> score[i];
		if (max < score[i]) { max = score[i]; }
	}
	sort(score, score + n);
	int total = 0;
	
	for (int j = 0; j < n; j++) {
		total += score[j];
		}

	double Mean = total/n;

	cout << "##### ������� ��� #####" << endl;
	cout << "�л���: " << n << endl;
	cout << "����: " << total << endl;
	cout << "���: " << Mean << endl;
	cout << "�ִ밪: " << max << endl;
}*/


//������, ���� �޸� �Ҵ� �̻��
int main(void) {
	int stuSize;
	double* pArr = nullptr;
	double sum, ave, max;

	cout << "�Է��� �л�(����) ��: ";
	cin >> stuSize;

	//���� �޸� �Ҵ� : �л�(����)�� ��ŭ..
	pArr = new double[stuSize];
	for (int i = 0; i < stuSize; i++)
		cin >> *(pArr + i);
	
	stuGrade(pArr, stuSize, sum, ave, max);
	cout << "\n ##### ������� ��� ##### \n" << endl;
	cout << "�л� ��: " << stuSize << endl;
	cout << "�� ��: " << sum << endl;
	cout << "���: " << ave << endl;
	cout << "�ִ밪: " << max << endl;

	//���� �޸� �ݳ�
	delete[] pArr;
	return 0;
}

void stuGrade(const double* pArr, int num, double& rSum, double& rAve, double& rMax) {
	rSum = 0;
	rMax = *pArr;
	for (int i = 0; i < num; i++) {
		rSum += *(pArr + i);
		if (rMax < *(pArr + i))
			rMax = *(pArr + i);
	}
	rAve = rSum / num;
}




