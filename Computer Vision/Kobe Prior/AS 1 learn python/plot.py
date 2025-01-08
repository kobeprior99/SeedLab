from matplotlib import pyplot as plt
import numpy as np
import math
#create a function to calculate each y value for each x value
# def function(x):
#     return math.exp(-x)

#generate x values 
# x = np.linspace(0, 1, num=1000)

#generate y values
# y = []
# for i in x:
#     y.append(function(i))

#plot the graph
# plt.plot(x, y)
# plt.xlabel("X")
# plt.ylabel("Y")
# plt.title("Graph of y = e^-x from 0 to 1")
# plt.show()

def isNumpyArr(x):
    if type(x) == np.ndarray:
        print("This is a numpy array")
        return True
    else:
        print("This is not a numpy array")
        return False
    
#test the function
arr = np.array([[1,2,3,4,5], [1,2,3,4,5]])
if isNumpyArr(arr)==True: 
    print(f"The array is {arr.ndim} dimensional")