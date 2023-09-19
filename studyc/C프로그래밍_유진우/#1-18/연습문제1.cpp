#include <iostream>
#include <algorithm>
using namespace std;

void stuGrade(const double* pArr, int num, double& rSum, double& rAve, double& rMax);

//포인터 미사용
/*int main(void) {
	int n;
	cout << "입력할 학생(성적) 수: ";
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

	cout << "##### 성적결과 출력 #####" << endl;
	cout << "학생수: " << n << endl;
	cout << "총점: " << total << endl;
	cout << "평균: " << Mean << endl;
	cout << "최대값: " << max << endl;
}*/


//포인터, 동적 메모리 할당 미사용
int main(void) {
	int stuSize;
	double* pArr = nullptr;
	double sum, ave, max;

	cout << "입력할 학생(성적) 수: ";
	cin >> stuSize;

	//동적 메모리 할당 : 학생(성적)수 만큼..
	pArr = new double[stuSize];
	for (int i = 0; i < stuSize; i++)
		cin >> *(pArr + i);
	
	stuGrade(pArr, stuSize, sum, ave, max);
	cout << "\n ##### 성적결과 출력 ##### \n" << endl;
	cout << "학생 수: " << stuSize << endl;
	cout << "총 점: " << sum << endl;
	cout << "평균: " << ave << endl;
	cout << "최대값: " << max << endl;

	//동적 메모리 반납
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




