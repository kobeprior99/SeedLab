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
def beginning():
    user_str = input("Your string is:")
    return user_str + ' '

the_str = beginning()


def state0(new_char):
    if new_char == 'a':
        return state1 
    else:
        return state0
    
def state1(new_char):
    if new_char == 'a':
        return state1
    elif new_char == 'b':
        return state2
    else:
        return state0

def state2(new_char):
    if new_char == 'a':
        return state1
    elif new_char == 'c':
        return state3
    else: 
        return state0

def state3(new_char):
    if new_char == 'a':
        return state1
    elif new_char == 'd':
        return state4 
    else: 
        return state0
    
def state4(new_char):
    print("abcd is contained in the string")
    return None


state = state0
check = False
for i in the_str: 
    new_state = state(i)
    state = new_state
    if state == state4:
        check = True
if check == False:
    print("abcd is not contained in the string")