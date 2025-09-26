#pragma once

// If debug is define, the code will be compiled with debug information
#define MUTE_UART

void usartSetup(void);

//Send information on uart2
//1000 char max
void usartprintf(const char* format,...);