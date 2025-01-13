'''
*******************************************************************
* File Name         : task2.py
* Description       : Create a Finite State Machine that checks
* if a string contains the 'abcd' sequence
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
#define each state of the FSM
#if else branching is used to determine the next state based on the current character
def state_0(current_char):
    if current_char == 'a':
        return state_a
    else:
        return state_0
def state_a(current_char):
    if current_char == 'b':
        return state_b
    elif current_char == 'a':
        return state_a
    else: 
        return state_0
    
def state_b(current_char):
    if current_char == 'c':
        return state_c
    elif current_char == 'a':
        return state_a
    else: 
        return state_0
    
def state_c(current_char):
    if current_char == 'd':
        return state_d
    elif current_char == 'a':
        return state_a
    else: 
        return state_0
#note that state_d is the final state and saves additional 
#iterations in for loop since the task is to see if the 'abcd' sequence is present in the string
def state_d(current_char):
    print("The string contains 'abcd' sequence")
    return None
    
#create a function to FSM test on string
def finite_state_machine_test(string):
    string += ' ' #add a space to the end of the string to ensure the last character is processed
    state = state_0
    for char in string:
        #start state machine
        new_state = state(char)
        state = new_state
        if state == None:
            return 
    #if you made it out of the loop without returning, 
    #then the string does not contain 'abcd' sequence
    print("The string does not contain 'abcd' sequence")
    return 

#get user input string
user_input_string = input("please enter a string: ")
#run the test to see if the string contains 'abcd' sequence
finite_state_machine_test(user_input_string)