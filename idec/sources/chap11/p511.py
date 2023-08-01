invitations = ["Kim", "Lee", "Park", "Choi"]
persons = [1, 3, 0, 6]
print(sum(persons))

print(any(persons))
print(all(persons))
print(max(persons))
for name, person in zip(invitations, persons):
     print(name, person)

