#include <it_sdk/config.h>
#include <it_sdk/wrappers.h>

// wait for a given number of Ms
void itsdk_delayMs(uint32_t ms) {
	delay(ms);
}

// get the time in ms since start with 32b millis overflow management
uint64_t __last_millis = 0;
uint64_t __loops_millis = 0;
uint64_t itsdk_time_get_ms() {
	uint32_t m = millis();
	if ( m < __last_millis ) __loops_millis++;
	__last_millis = m;
	return __loops_millis << 32 + m;
}