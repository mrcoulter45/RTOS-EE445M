//opens one channel of specified ADC
// channelNum:
//	  0: PE3
//	  1: PE2
//	  2: PE1
//	  3: PE0
void ADC_Open(uint8_t channelNum);

//Reads a single value of the currently open ADC
uint16_t ADC_In(void);

//opens specified ADC
//will store "numberOfSamples" ADC values to "buffer" at a rate of "period"
int ADC_Collect(uint32_t channelNum, uint32_t period, uint16_t buffer[], uint32_t numberOfSamples);
