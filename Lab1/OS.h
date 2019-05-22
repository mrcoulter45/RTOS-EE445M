int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority); 

void OS_ClearPeriodicTime(void);

uint32_t OS_ReadPeriodicTime(void);
