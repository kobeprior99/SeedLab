# define state variable
state = 0
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
    
def state_d(current_char):
    print("The string contains 'abcd' sequence")
    return None
    
#create a function to perform finite state diagram described on string
def finite_state_machine_test(string):
    state = state_0
    for char in string:
        #start state machine
        new_state = state(char)
        state = new_state
        if state == None:
            return 
    #if you made it out of the loop without returning, then the string does not contain 'abcd' sequence
    print("The string does not contain 'abcd' sequence")
    return 

#get user input string
user_input_string = input("please enter a string: ")

finite_state_machine_test(user_input_string)