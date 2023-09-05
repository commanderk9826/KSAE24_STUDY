#include <iostream>
#include <cstring>
using namespace std;

class Person
{
private:
	int age;        // 나이
	char name[50];     // 이름
public:
	Person(int myage, char * myname) : age(myage)
	{
		strcpy(name, myname);
	}
	void WhatYourName() const 
	{
		cout<<"My name is "<<name<<endl;
	}
	void HowOldAreYou() const 
	{
		cout<<"I'm "<<age<<" years old"<<endl;
	}
};

class UnivStudent : public Person
{
private:
	char major[50];     // 전공과목
public:
	UnivStudent(char * myname, int myage, char * mymajor)
		: Person(myage, myname)
	{
		strcpy(major, mymajor);
	}
	void WhoAreYou() const 
	{
		WhatYourName();
		HowOldAreYou();
		cout<<"My major is "<<major<<endl<<endl;
	}
};

int main(void)
{
	UnivStudent ustd1("Lee", 22, "Computer eng.");
	ustd1.WhoAreYou();

	UnivStudent ustd2("Yoon", 21, "Electronic eng.");
	ustd2.WhoAreYou();
	return 0;
};