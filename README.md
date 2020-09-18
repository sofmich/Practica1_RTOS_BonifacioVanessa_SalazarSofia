# Practica1_RTOS_BonifacioVanessa_SalazarSofia

In this practice we had to implement a miniRTOS system through the algorithm using calls to functions and creating new tasks. 

Changes made on functions were:

rtos start scheduler(void)    global clock set to 0 and create tasks here
rtos create task(task body, priority, autostart flag, task hanlde) create the task checking if there is still space on task list
dispatcher(origen) wich had an error at calling context switch. Here we make the corresponding changes on r0 and r7 to save stack frame and also a loop to find the highest priority
context switch(origen) changes the actual task for the next to keep execution checking priorities
PendSV(void) makes the corresponding changes on r0 and r7 to save stack
systick(void) keeps the counter on global timer




