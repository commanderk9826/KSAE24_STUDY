weather_forecast = [	{ "온도" :  31, "습도" : 73, "풍속" :  5} ,
			{ "온도" :  30, "습도" : 67, "풍속" :  3} ,
			{ "온도" :  29, "습도" : 76, "풍속" :  4} ,
			{ "온도" :  33, "습도" : 56, "풍속" :  6} ,
			{ "온도" :  28, "습도" : 68, "풍속" :  7} ,
			{ "온도" :  24, "습도" : 57, "풍속" :  2}  ]

for forecast in weather_forecast: 
     temperature = forecast["온도"]  
     print(temperature, end=" ")
