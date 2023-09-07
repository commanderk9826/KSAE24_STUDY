struct Car
{
	char gamerID[ID_LEN];	// 소유자ID
	int fuelGauge;		// 연료량
	int curSpeed;		// 현재속도

	void ShowCarState()
	{
		cout<<"소유자ID: "<<gamerID<<endl;
		cout<<"연료량: "<<fuelGauge<<"%"<<endl;
		cout<<"현재속도: "<<curSpeed<<"km/s"<<endl<<endl;
	}
	void Accel()
	{
		if(fuelGauge<=0)
			return;
		else
			fuelGauge-=FUEL_STEP;

		if(curSpeed+ACC_STEP>=MAX_SPD)
		{
			curSpeed=MAX_SPD;
			return;
		}
	
		curSpeed+=ACC_STEP;
	}
	void Break()
	{
		if(curSpeed<BRK_STEP)
		{
			curSpeed=0;
			return;
		}

		curSpeed-=BRK_STEP;
	}
};