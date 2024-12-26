# ----------------------------------------------------------------------------- #
#                                                                               #              
#    Project:        Using Threads                                              #
#    Module:         main.py                                                    #
#    Author:         VEX                                                        #
#    Created:        Fri Aug 05 2022                                            #
#    Description:    This example will show how to run multiple threads (tasks) # 
#                    in a project at the same time                              #
#                                                                               #                                                                          
#    Configuration:  None                                                       #
#                                                                               #                                                                          
# ----------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

# Robot configuration code


# Create a function that will be used as a thread
def thread_1():
    thread_loop_count = 0
    while True:
        brain.screen.set_cursor(2, 1)
        brain.screen.print("Thread #1 Iterations (250ms loop):", thread_loop_count)
        thread_loop_count = thread_loop_count + 1
        wait(250, MSEC)

# Functions to be used as threads can also accept parameters
def thread_2(time_delay):
    thread_loop_count = 0
    while True:
        brain.screen.set_cursor(3, 1)
        brain.screen.print("Thread #2 Iterations (500ms loop):", thread_loop_count)
        thread_loop_count = thread_loop_count + 1
        wait(time_delay, MSEC)

# Creating instances of threads will start the threads immediately
my_thread1 = Thread(thread_1)
# Creating a thread with parameters must use a tuple for passing parameters
my_thread2 = Thread(thread_2, (500,))

# Print from the main thread to show that it is running at the same time as other threads
main_count = 0
while True:
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Main Thread Iterations (100ms loop):", main_count)
    main_count = main_count + 1
    wait(100, MSEC)
