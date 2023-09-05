#include <iostream>
#include <cstring>
using namespace std;

class Gun
{
private:
	int bullet;    	// ÀåÀüµÈ ÃÑ¾ËÀÇ ¼ö
public:
	Gun(int bnum) : bullet(bnum)
	{ }
	void Shut()
	{
		cout<<"BBANG!"<<endl;
		bullet--;
	}
};

class Police : public Gun
{
private:
	int handcuffs;	    // ¼ÒÀ¯ÇÑ ¼ö°©ÀÇ ¼ö
public:
	Police(int bnum, int bcuff)
		: Gun(bnum), handcuffs(bcuff)
	{  }
	void PutHandcuff() 
	{
		cout<<"SNAP!"<<endl;
		handcuffs--;
	}
};

int main(void)
{
	Police pman(5, 3);	// ÃÑ¾Ë 5, ¼ö°© 3
	pman.Shut();
	pman.PutHandcuff();
	return 0;
}