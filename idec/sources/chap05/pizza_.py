def main() : 
    print("20cm 피자 2개의 면적:", get_area(20)+get_area(20))
    print("30cm 피자 1개의 면적:", get_area(30))

## 원의 면적을 계산한다.
# @param radius 원의 반지름
# @return 원의 면적 
#
def get_area(radius) :
    if radius > 0 :
        area = 3.14*radius**2
    else :
        area = 0
    return area

main()
