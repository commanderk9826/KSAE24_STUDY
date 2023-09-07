/*
   OOhandling.cpp   
*/
#include <iostream>
using std::endl;
using std::cout;
using std::cin;

int main(void)
{
	int a, b;
	
	cout<<"두개의 숫자 입력 : ";
	cin>>a>>b;

	try{
		if(b==0)
			throw b;
		cout<<"a/b의 몫 : "<<a/b<<endl;
		cout<<"a/b의 나머지 : "<<a%b<<endl;
	}
	catch(int exception){
		cout<<exception<<" 입력."<<endl; 
		cout<<"입력오류! 다시 실행 하세요."<<endl;
	}

	return 0;
}