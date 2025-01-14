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
* 01/14/2025	Blane Miller	Created File
*
******************************************************************
Hardware Setup: Power on the Pi -> either connect peripherials or connect via PiConnect or other VNC service 
Example Excecution: -> open terminal -> navigate to the directory where the task1.py file is located using cd command ->
run the python file using 'python task1.py' command
'''

#open datafile in read mode as a file object
with open('datafile.txt', 'r') as f:
    #read datafile, or put another way extract items in the file
    #Using eval to convert items from string to list
    b = eval(f.read()) 

#Task 1: Finding the maximum
#Using preset max finder function
maximum = b.max()
print(f"The maximum of this data set is {maximum}")

#Task 2: Finding the minimum
#Using preset min finder function
minimum = b.min()
print(f"The minimum of this data set is {minimum}")

#Task 3: Finding the index where the number 38 is located
#Using preset index finder function
idx = b.index(38)
print(f"The index of the number 38 in this data set is {idx}")

#Task 4: Finding the number or numbers that are repeated most, and how many times they are repeated
#Importing a counter so I can count repetitions of a word
from collections import Counter
#Counting the number of instances of each number in the list
ind_counts = Counter(b)
#Finding which values among the ind counts are highest using a preset max finder function
max_counts = max(ind_counts.values())
#Finds the most repeated number by looking for the number(s) in which the count from the tuples being returned by ind_counts.items() is equal to the max number of counts in the list
most_repeated_num = [num for num, count in ind_counts.items() if count == max_counts]
print(f"The most repeated number(s) in this data set is/are {most_repeated_num}, which repeats {max_counts} times")

#Task 5: Sorting the list
#Importing np stuff 
import numpy as np
#Converting list to numpy array
arr = np.array(b)
#Sorting numpy array using preset np function 
sorted_arr = np.sort(arr)
print(f"The sorted list is as shown: {sorted_arr}")

#Task 6: List of the even numbers as they appear in the list
#Using list comprehension to extract the even numbers 
even_nums = [num for num in b if b % 2 == 0]
print(f"The list of even numbers in the order they appear is as shown: {even_nums}")