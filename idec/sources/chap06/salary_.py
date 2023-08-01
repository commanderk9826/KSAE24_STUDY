salaries = [200, 250, 300, 280, 500]

def modify(values, factor) :
    for i in range(len(values)) :
        values[i] = values[i] * factor

print("인상전", salaries)
modify(salaries, 1.3)
print("인상후", salaries)
