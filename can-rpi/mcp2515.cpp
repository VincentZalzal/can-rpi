#include "precomp.h"
#include "mcp2515.h"

using namespace std;

namespace mcp2515
{
    const int     SPI_CHANNEL      = 0;
    const int     SPI_SPEED_HZ     = 7372800 / 4;
    const int     MAX_COMMAND_SIZE = 32;
    const uint8_t DONT_CARE        = 0;

    // maybe not here... maybe with wiringPiSetup()? but would need to put SPI_SPEED_HZ in .h
#if 0
    void initialize()
    {
        // TODO
        if ( wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED_HZ) < 0 )
            // what?

        // Enable pull-up resistors on interrupt input pin.
        // SET_BIT(PORT_INT, PIN_INT_IRQ);
    }
#endif

    void readCommand(uint8_t instruction, uint8_t address, uint8_t* readValues, int nBytes)
    {
        uint8_t commandBuffer[MAX_COMMAND_SIZE];
        int headerSize = 1;
        commandBuffer[0] = instruction;
        if (address != NO_ADDRESS)
        {
            commandBuffer[1] = address;
            ++headerSize;
        }

        assert(headerSize + nBytes < MAX_COMMAND_SIZE);
        for (int i = 0; i < nBytes; ++i)
            commandBuffer[headerSize + i] = DONT_CARE;
        // RPI if ( wiringPiSPIDataRW(SPI_CHANNEL, commandBuffer, headerSize + nBytes) < 0 )
            cout << "wiringPiSPIDataRW failed." << endl;
        for (int i = 0; i < nBytes; ++i)
            readValues[i] = commandBuffer[headerSize + i];
    }

    void writeCommand(uint8_t instruction, uint8_t address, const uint8_t* writeValues, int nBytes)
    {
        uint8_t commandBuffer[MAX_COMMAND_SIZE];
        int headerSize = 1;
        commandBuffer[0] = instruction;
        if (address != NO_ADDRESS)
        {
            commandBuffer[1] = address;
            ++headerSize;
        }

        assert(headerSize + nBytes < MAX_COMMAND_SIZE);
        for (int i = 0; i < nBytes; ++i)
            commandBuffer[headerSize + i] = writeValues[i];
        // RPI if ( wiringPiSPIDataRW(SPI_CHANNEL, commandBuffer, headerSize + nBytes) < 0 )
            cout << "wiringPiSPIDataRW failed." << endl;
    }

} // namespace mcp2515

#if 0
    const uint8_t NO_TIMEOUT = -1;

    Status waitForInterrupt(uint8_t timeoutSlotNo)
    {
        if (timeoutSlotNo != NO_TIMEOUT)
        {
            while (true)
            {
                if (bit_is_clear(PIN_INT, PIN_INT_IRQ))
                    return INTERRUPT;
                if (timer::getRemainingTicks(timeoutSlotNo) == 0)
                    return TIMEOUT;
            }
        }
        else
        {
            loop_until_bit_is_clear(PIN_INT, PIN_INT_IRQ);
            return INTERRUPT;
        }
    }

} // namespace mcp2515

#endif
