# FreeRTOS

Two tasks are created with priority 2 and 1, task1 with highest priority and task 2 with lowest priority then task 1. 
1. Task1 runs its application in 13 micro second
2. Task1 is blocked for 2 seconds
3. Task2 runs its application within 10 micro seconds and this task will be in ready state for every 1 second of tick time after that task2 is in blocked state idle task is    
   scheduled, because none of the other tasks are in ready state.
4. idle task runs for 1 second after this task1 and task2 both are ready for running
5. scheduler schedules task1 first and after 16 micro second task 2 exutes
6. This process repeats,
  
* for further reference check LED21.SVdat file in segger system view
