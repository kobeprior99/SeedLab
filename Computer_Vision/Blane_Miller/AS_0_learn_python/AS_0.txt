#Exercise 1
print("Hello, World!") 

#Exercise 2
mystring = "hello"
myfloat = 10.0
myint = 20

# testing code
if mystring == "hello":
    print("String: %s" % mystring)
if isinstance(myfloat, float) and myfloat == 10.0:
    print("Float: %f" % myfloat)
if isinstance(myint, int) and myint == 20:
    print("Integer: %d" % myint) 

#Exercise 3 
numbers = [1,2,3]
strings = ["hello","world"]
names = ["John", "Eric", "Jessica"]

# write your code here
second_name = names[1]


# this code should write out the filled arrays and the second name in the names list (Eric).
print(numbers)
print(strings)
print("The second name on the names list is %s" % second_name)

#Exercise 4
x = object()
y = object()

# TODO: change this code
x_list = [x] * 10
y_list = [y] * 10
big_list = x_list + y_list

print("x_list contains %d objects" % len(x_list))
print("y_list contains %d objects" % len(y_list))
print("big_list contains %d objects" % len(big_list))

# testing code
if x_list.count(x) == 10 and y_list.count(y) == 10:
    print("Almost there...")
if big_list.count(x) == 10 and big_list.count(y) == 10:
    print("Great!")

#Exercise 5
data = ("John", "Doe", 53.44)
format_string = "Hello %s %s. Your current balance is $%s."

print(format_string % data)

#Exercise 6
s = "Strings are awesome!"
# Length should be 20
print("Length of s = %d" % len(s))

# First occurrence of "a" should be at index 8
print("The first occurrence of the letter a = %d" % s.index("a"))

# Number of a's should be 2
print("a occurs %d times" % s.count("a"))

# Slicing the string into bits
print("The first five characters are '%s'" % s[:5]) # Start to 5
print("The next five characters are '%s'" % s[5:10]) # 5 to 10
print("The thirteenth character is '%s'" % s[12]) # Just number 12
print("The characters with odd index are '%s'" %s[1::2]) #(0-based indexing)
print("The last five characters are '%s'" % s[-5:]) # 5th-from-last to end

# Convert everything to uppercase
print("String in uppercase: %s" % s.upper())

# Convert everything to lowercase
print("String in lowercase: %s" % s.lower())

# Check how a string starts
if s.startswith("Str"):
    print("String starts with 'Str'. Good!")

# Check how a string ends
if s.endswith("ome!"):
    print("String ends with 'ome!'. Good!")

# Split the string into three separate strings,
# each containing only a word
print("Split the words of the string: %s" % s.split(" "))

#Exercise 7
# change this code
number = 16
second_number = 0
first_array = [1,2,3]
second_array = [1,2]

if number > 15:
    print("1")

if first_array:
    print("2")

if len(second_array) == 2:
    print("3")

if len(first_array) + len(second_array) == 5:
    print("4")

if first_array and first_array[0] == 1:
    print("5")

#Exercise 8
# your code goes here
for number in numbers:
    if number == 237:
        break
    elif number % 2 == 0:
        print(number)

#Exercise 9
# Modify this function to return a list of strings as defined above
def list_benefits():
    return ["More organized code", "More readable code", "Easier code reuse", "Allowing programmers to share and connect code together"]

# Modify this function to concatenate to each benefit - " is a benefit of functions!"
def build_sentence(info):
    return info + " is a benefit of functions!"

def name_the_benefits_of_functions():
    list_of_benefits = list_benefits()
    for benefit in list_of_benefits:
        print(build_sentence(benefit))

name_the_benefits_of_functions() 

#Exercise 10
# your code goes here
car1 = Vehicle()
car1.name = "Fer" 
car1.kind = "convertible"
car1.color = "red" 
car1.value = 60000.00 
car2 = Vehicle()
car2.name = "Jump"
car2.kind = "van"
car2.color = "blue" 
car2.value = 10000.00 

#Exercise 11 
phonebook = {  
    "John" : 938477566,
    "Jack" : 938377264,
    "Jill" : 947662781,
    "Jake" : 938273443
}  
# your code goes here
phonebook.pop("Jill") 

#Exercise 12
import re

# Your code goes here
find_members = []
for member in dir(re):
    if "find" in member:
        find_members.append(member)

print(sorted(find_members)) 

#Exercise 13 
# Taking the name input using raw_input()
name = raw_input("Enter your name: ")

# Taking the age input using input() and converting it to integer
age = int(input("Enter your age: "))

# Taking the country input using raw_input()
country = raw_input("Enter your country: ")

# Displaying the formatted sentence with name, age, and country
message = f"Hello, {name}! You are {age} years old and you are from {country}." 

print(message) 

#Note: This one wouldn't let me get it right no matter what, I think it's a site error 
