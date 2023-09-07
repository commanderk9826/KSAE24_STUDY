#include <iostream>
#include "Point.h"
using namespace std;

Point::Point(int x, int y) : xpos(x), ypos(y) {  }

ostream& operator<<(ostream& os, const Point& pos)
{
	os<<'['<<pos.xpos<<", "<<pos.ypos<<']'<<endl;
	return os;
}