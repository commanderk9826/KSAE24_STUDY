##
#	이 프로그램은 중첩 반복문을 사용하여서 모든 사람/식당 조합을 출력한다. 
#
persons = [ "Kim" , "Park" , "Lee"] 
restaurants = [ "Korean" , "American" , "French"]

for person in persons:
    for restaurant in restaurants:
        print(person + "이 " + restaurant+" 식당을 방문")
