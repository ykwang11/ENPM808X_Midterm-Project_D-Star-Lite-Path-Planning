#include <iostream>
#include <AnalogSensor.hpp>

int main()
{
    AnalogSensor lightSensor(5);
    std::cout << "Averaged sensor reading: " << lightSensor.Read() << std::endl;
    bool terminator;
    if( terminator )
    {
        std::cout << "DONE" << std::endl;
    }
    return 0;
}
