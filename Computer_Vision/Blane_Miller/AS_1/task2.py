'''
*******************************************************************
* File Name         : task2.py
* Description       : Create a Finite State Machine that checks
* if a string contains the 'abcd' sequence
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

# Function to gather user input
def beginning():
    user_str = input("Your string is:")
    # Adds a space at the end of the string so that the entire string, 
    # including the final element, will be parsed through
    return user_str + ' '

# Function call outside the loop so that the string is only gathered once
the_str = beginning()

# Initial state searching for an 'a'
def state0(new_char):
    if new_char == 'a':
        return state1 
    else:
        return state0
    
# Second state searching for a 'b', 
# or maintaining on an 'a', 
# or going back to initial state otherwise
def state1(new_char):
    if new_char == 'a':
        return state1
    elif new_char == 'b':
        return state2
    else:
        return state0


# Third state searching for a 'c', 
# or pushing back to the second state if an 'a', 
# or pushing back to initial state otherwise
def state2(new_char):
    if new_char == 'a':
        return state1
    elif new_char == 'c':
        return state3
    else: 
        return state0

# Fourth state searching for a 'd', 
# or pushing back to the second state if an 'a', 
# or pushing back to initial state otherwise
def state3(new_char):
    if new_char == 'a':
        return state1
    elif new_char == 'd':
        return state4 
    else: 
        return state0

# Final state printing that abcd is contained in the string,
# and then going back to the initial state to check for another instance of it
def state4(new_char):
    print("abcd is contained in the string")
    return state0

# Setting state to be initial state
state = state0
# Setting check to be false in case abcd is not contained in the string
check = False
# Looping through every element of the string
for i in the_str: 
    # Putting the current state into a variable, 
    # and then setting that as the current state in the machine
    new_state = state(i)
    state = new_state
    # Setting check to True so that the not contained text doesn't output
    if state == state4:
        check = True
# Using the check variable in case abcd isn't contained within the string
if check == False:
    print("abcd is not contained in the string")