/*
   inheri_catch1.cpp   
*/
#include <iostream>
using std::endl;
using std::cout;
using std::cin;

class ExceptA
{
public:
	void What(){
		cout<<"ExceptA 예외"<<endl;
	}
};

class ExceptB : public ExceptA
{
public:
	void What(){
		cout<<"ExceptB 예외"<<endl;
	}
};

class ExceptC : public ExceptB
{
public:
	void What(){
		cout<<"ExceptC 예외"<<endl;
	}
};

void ExcepFunction(int ex)
{
	if(ex==1)
		throw ExceptA();
	else if(ex==2)
		throw ExceptB();
	else
		throw ExceptC();
}

int main(void)
{

	int exID;
	cout<<"발생시킬 예외의 숫자: ";
	cin>>exID;

	try{
		ExcepFunction(exID);
	}
	catch(ExceptA& ex){
		cout<<"catch(ExceptA& ex)에 의한 처리"<<endl;
		ex.What();
	}	
	catch(ExceptB& ex){
		cout<<"catch(ExceptB& ex)에 의한 처리"<<endl;
		ex.What();
	}
	catch(ExceptC& ex){
		cout<<"catch(ExceptC& ex)에 의한 처리"<<endl;
		ex.What();
	}

	return 0;
}