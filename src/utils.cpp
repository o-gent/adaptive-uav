#include <Arduino.h>
#include <RemoteDebug.h>
/**
 * Process Debug library messages just before delay
 * Had some problems where the debug library would disconnect without this
 */
void delayy(unsigned long ms, RemoteDebug &Debug)
{
    uint32_t start = micros();
    debugA("waiting");
    Debug.handle();
    while (ms > 0)
    {
        while (ms > 0 && (micros() - start) >= 1000)
        {
            ms--;
            start += 1000;
        }
    }
}
