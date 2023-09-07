/*
   catch_over2.cpp   
*/
#include <iostream>
using std::endl;
using std::cout;
using std::cin;

char* account="1234-5678";  //계좌번호
int sid=1122;    //비밀번호
int balance=1000;  //잔액.

class AccountExpt
{
	char acc[10];
	int sid;
public:
	AccountExpt(char* str, int id){
		strcpy(acc, str);
		sid=id;
	}
	void What(){
		cout<<"계좌번호: "<<acc<<endl;
		cout<<"비밀번호: "<<sid<<endl;
	}
};

int main(void)
{
	char acc[10];
	int id;
	int money;

	try{
		cout<<"계좌번호 입력: ";
		cin>>acc;
		cout<<"비밀번호 4자리 입력: ";
		cin>>id;
		if(strcmp(account, acc) || sid!=id)
			throw AccountExpt(acc, id);
	}
	catch(AccountExpt& expt){
		cout<<"다음 입력을 다시 한번 확인 하세요"<<endl;
		expt.What();
	}

	try{
		cout<<"출금액 입력: ";
		cin>>money;
		if(balance<money)
			throw money;
		
		balance-=money;
		cout<<"잔액: "<<balance<<endl;
	}
	catch(int money){
		cout<<"부족 금액: "<<money-balance<<endl;
	}
	return 0;
}
