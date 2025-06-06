'''
*******************************************************************
* File Name         : task1.py
* Description       : Read data from a file and 
* perform operations on the data such as finding
* the maximum value, finding the minimum value, 
* finding the index of a specific value, finding the
* number repeated the most and the number of times it 
* is repeated, converting data to a numpy array and sorting, and
* finding all even numbers in the data.
*                    
* Revision History  :
* Date		Author 			Comments
* ------------------------------------------------------------------
* 01/12/2025	Kobe Prior	Created File
*
******************************************************************
Hardware Setup: Power on the Pi -> either connect peripherials or connect via PiConnect or other VNC service 
Example Excecution: -> open terminal -> navigate to the directory where the task1.py file is located using cd command ->
run the python file using 'python task1.py' command
'''

from collections import Counter #used in task 4
import numpy as np #used in task 5

#open the data file as a file object
with open('datafile.txt', 'r') as file:
    #extract the items in the file using eval function
    #eval function is used to convert the string representation of a list to a list
    data = eval(file.read())

#print(data) #debug line to see if we properly extracted the data from the file

#1. find the maximum value in the list
#using built in max function to find the maximum value in the list
max_value = max(data)
print(f"Task 1: The maximum value in the list is: {max_value}")

#2. find the minimum value in the list
#using built in min function to find the minimum value in the list
min_value = min(data)   
print(f"Task 2: The minimum value in the list is: {min_value}")

#3. Find the index of the number 38
#using the index method to find the index of the number 38
index_38 = data.index(38)
print(f"Task 3: The index of the number 38 is: {index_38}") 


#4. The number or numbers repeated the most and the number of times repeated
print("Task 4:")
counter = Counter(data)
#find the highest frequency in the list
#counter.values() returns a list of the counts for each element in the list
highest_frequency = max(counter.values())
#print(highest_frequency) #debug to see the highest frequency
sameFreqCount = 0 #initialize a variable to count the number of elements with the same frequency
for elements in counter:
    #print(elements, counter[elements])#debug line to see the elements and their frequencies 
    #note that the indexing behavior of the counter object is like a dictionary
    if counter[elements] == highest_frequency and sameFreqCount == 0:
        print(f"The number repeated the most is {elements} is repeated {highest_frequency} times")
        sameFreqCount+=1
    elif counter[elements] == highest_frequency:
        #if there are multiple elements with the same frequency
        print(f"{elements} is also repeated {highest_frequency} times") 

#5. Convert to numpy array and sort list in ascending order
numpy_array = np.array(data) #convert the list to a numpy array
sorted_numpy_array = np.sort(numpy_array) #sort the numpy array in ascending order
print(f"Task 5: The sorted list in ascending order is: {sorted_numpy_array}")

#6. All even numbers in order (using list comprehension)
#print(data) #debug line to see if the list is sorted
even_acending=[x for x in data if x % 2 == 0] #list comprehension to find all even numbers in the list
even_acending.sort() #sort the list in ascending order better to do this after list comprehension to save processing time
print(f"Task 6: The even numbers in the list in order are: {even_acending}")