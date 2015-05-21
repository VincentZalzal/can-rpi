#pragma once

namespace mcp2515
{

    enum Address
    {
        RXF0SIDH = 0x00,
        RXF0SIDL = 0x01,
        RXF0EID8 = 0x02,
        RXF0EID0 = 0x03,
        RXF1SIDH = 0x04,
        RXF1SIDL = 0x05,
        RXF1EID8 = 0x06,
        RXF1EID0 = 0x07,
        RXF2SIDH = 0x08,
        RXF2SIDL = 0x09,
        RXF2EID8 = 0x0A,
        RXF2EID0 = 0x0B,
        BFPCTRL  = 0x0C, // bit modify enabled
        TXRTSCTRL= 0x0D, // bit modify enabled
        CANSTAT  = 0x0E,
        CANCTRL  = 0x0F, // bit modify enabled

        RXF3SIDH = 0x10,
        RXF3SIDL = 0x11,
        RXF3EID8 = 0x12,
        RXF3EID0 = 0x13,
        RXF4SIDH = 0x14,
        RXF4SIDL = 0x15,
        RXF4EID8 = 0x16,
        RXF4EID0 = 0x17,
        RXF5SIDH = 0x18,
        RXF5SIDL = 0x19,
        RXF5EID8 = 0x1A,
        RXF5EID0 = 0x1B,
        TEC      = 0x1C,
        REC      = 0x1D,
        // CANSTAT = 0x1E, repeated for consecutive access
        // CANCTRL = 0x1F, repeated for consecutive access

        RXM0SIDH = 0x20,
        RXM0SIDL = 0x21,
        RXM0EID8 = 0x22,
        RXM0EID0 = 0x23,
        RXM1SIDH = 0x24,
        RXM1SIDL = 0x25,
        RXM1EID8 = 0x26,
        RXM1EID0 = 0x27,
        CNF3     = 0x28, // bit modify enabled
        CNF2     = 0x29, // bit modify enabled
        CNF1     = 0x2A, // bit modify enabled
        CANINTE  = 0x2B, // bit modify enabled
        CANINTF  = 0x2C, // bit modify enabled
        EFLG     = 0x2D, // bit modify enabled
        // CANSTAT = 0x2E, repeated for consecutive access
        // CANCTRL = 0x2F, repeated for consecutive access

        TXB0CTRL = 0x30, // bit modify enabled
        TXB0SIDH = 0x31,
        TXB0SIDL = 0x32,
        TXB0EID8 = 0x33,
        TXB0EID0 = 0x34,
        TXB0DLC  = 0x35,
        TXB0D0   = 0x36,
        TXB0D1   = 0x37,
        TXB0D2   = 0x38,
        TXB0D3   = 0x39,
        TXB0D4   = 0x3A,
        TXB0D5   = 0x3B,
        TXB0D6   = 0x3C,
        TXB0D7   = 0x3D,
        // CANSTAT = 0x3E, repeated for consecutive access
        // CANCTRL = 0x3F, repeated for consecutive access

        TXB1CTRL = 0x40, // bit modify enabled
        TXB1SIDH = 0x41,
        TXB1SIDL = 0x42,
        TXB1EID8 = 0x43,
        TXB1EID0 = 0x44,
        TXB1DLC  = 0x45,
        TXB1D0   = 0x46,
        TXB1D1   = 0x47,
        TXB1D2   = 0x48,
        TXB1D3   = 0x49,
        TXB1D4   = 0x4A,
        TXB1D5   = 0x4B,
        TXB1D6   = 0x4C,
        TXB1D7   = 0x4D,
        // CANSTAT = 0x4E, repeated for consecutive access
        // CANCTRL = 0x4F, repeated for consecutive access

        TXB2CTRL = 0x50, // bit modify enabled
        TXB2SIDH = 0x51,
        TXB2SIDL = 0x52,
        TXB2EID8 = 0x53,
        TXB2EID0 = 0x54,
        TXB2DLC  = 0x55,
        TXB2D0   = 0x56,
        TXB2D1   = 0x57,
        TXB2D2   = 0x58,
        TXB2D3   = 0x59,
        TXB2D4   = 0x5A,
        TXB2D5   = 0x5B,
        TXB2D6   = 0x5C,
        TXB2D7   = 0x5D,
        // CANSTAT = 0x5E, repeated for consecutive access
        // CANCTRL = 0x5F, repeated for consecutive access

        RXB0CTRL = 0x60, // bit modify enabled
        RXB0SIDH = 0x61,
        RXB0SIDL = 0x62,
        RXB0EID8 = 0x63,
        RXB0EID0 = 0x64,
        RXB0DLC  = 0x65,
        RXB0D0   = 0x66,
        RXB0D1   = 0x67,
        RXB0D2   = 0x68,
        RXB0D3   = 0x69,
        RXB0D4   = 0x6A,
        RXB0D5   = 0x6B,
        RXB0D6   = 0x6C,
        RXB0D7   = 0x6D,
        // CANSTAT = 0x6E, repeated for consecutive access
        // CANCTRL = 0x6F, repeated for consecutive access

        RXB1CTRL = 0x70, // bit modify enabled
        RXB1SIDH = 0x71,
        RXB1SIDL = 0x72,
        RXB1EID8 = 0x73,
        RXB1EID0 = 0x74,
        RXB1DLC  = 0x75,
        RXB1D0   = 0x76,
        RXB1D1   = 0x77,
        RXB1D2   = 0x78,
        RXB1D3   = 0x79,
        RXB1D4   = 0x7A,
        RXB1D5   = 0x7B,
        RXB1D6   = 0x7C,
        RXB1D7   = 0x7D,
        // CANSTAT = 0x7E, repeated for consecutive access
        // CANCTRL = 0x7F,  repeated for consecutive access

        NO_ADDRESS = 0xFF // implementation detail
    };

    const uint8_t INSTRUCTION_RESET           = 0xC0;
    const uint8_t INSTRUCTION_READ            = 0x03;
    const uint8_t INSTRUCTION_READ_RX_BUFFER  = 0x90;
    const uint8_t INSTRUCTION_WRITE           = 0x02;
    const uint8_t INSTRUCTION_LOAD_TX_BUFFER  = 0x40;
    const uint8_t INSTRUCTION_REQUEST_TO_SEND = 0x80;
    const uint8_t INSTRUCTION_READ_STATUS     = 0xA0;
    const uint8_t INSTRUCTION_RX_STATUS       = 0xB0;
    const uint8_t INSTRUCTION_BIT_MODIFY      = 0x05;

    const uint8_t SEND_TXB0 = INSTRUCTION_REQUEST_TO_SEND | 0x1;
    const uint8_t SEND_TXB1 = INSTRUCTION_REQUEST_TO_SEND | 0x2;
    const uint8_t SEND_TXB2 = INSTRUCTION_REQUEST_TO_SEND | 0x4;

    enum ReadFrom
    {
        FROM_IDENTIFIER = INSTRUCTION_READ_RX_BUFFER,
        FROM_DATA       = INSTRUCTION_READ_RX_BUFFER | 0x2,
    };

    enum LoadTo
    {
        TO_IDENTIFIER = INSTRUCTION_LOAD_TX_BUFFER,
        TO_DATA       = INSTRUCTION_LOAD_TX_BUFFER | 0x1,
    };

    enum Status
    {
        INTERRUPT,
        TIMEOUT
    };

#if 0

    void   initialize();
    inline void   reset();
    Status waitForInterrupt(uint8_t timeoutSlotNo);
    inline bool   hasPendingInterrupt();

    // MCP2515 commands
    inline uint8_t read(Address address);
    inline void    readSequence(Address address, uint8_t* readValues, uint8_t nBytes);
    inline void    readRxBuffer(uint8_t bufferNo, ReadFrom readFrom, uint8_t* readValues, uint8_t nBytes);     // bufferNo is 0 or 1
    inline void    write(Address address, uint8_t writeValue);
    inline void    writeSequence(Address address, const uint8_t* writeValues, uint8_t nBytes);
    inline void    loadTxBuffer(uint8_t bufferNo, LoadTo loadTo, const uint8_t* writeValues, uint8_t nBytes);  // bufferNo is 0, 1 or 2
    inline void    requestToSend(uint8_t instruction); // ORed combination of SEND_TXBn
    inline void    requestToSendOneBuffer(uint8_t bufferNo); // bufferNo is 0, 1 or 2
    inline void    bitModify(Address address, uint8_t mask, uint8_t data); // not all addresses allow bit modify; see enum above

                                                                           ////////////////////////////////////////////////////////////////////////////////
                                                                           // Private functions

    void readCommand(uint8_t instruction, uint8_t address, uint8_t* readValues, uint8_t nBytes);
    void writeCommand(uint8_t instruction, uint8_t address, const uint8_t* writeValues, uint8_t nBytes);

    ////////////////////////////////////////////////////////////////////////////////
    // Inline functions

    inline void reset()
    {
        writeCommand(INSTRUCTION_RESET, NO_ADDRESS, 0, 0);
    }

    inline bool hasPendingInterrupt()
    {
        return bit_is_clear(PIN_INT, PIN_INT_IRQ);
    }

    inline uint8_t read(Address address)
    {
        uint8_t readValue;
        readCommand(INSTRUCTION_READ, static_cast<uint8_t>(address), &readValue, 1);
        return readValue;
    }

    inline void readSequence(Address address, uint8_t* readValues, uint8_t nBytes)
    {
        readCommand(INSTRUCTION_READ, static_cast<uint8_t>(address), readValues, nBytes);
    }

    inline void readRxBuffer(uint8_t bufferNo, ReadFrom readFrom, uint8_t* readValues, uint8_t nBytes)     // bufferNo is 0 or 1
    {
        uint8_t instruction = (uint8_t)readFrom | logicalShiftLeft(bufferNo, 2);
        readCommand(instruction, NO_ADDRESS, readValues, nBytes);
    }

    inline void write(Address address, uint8_t writeValue)
    {
        writeCommand(INSTRUCTION_WRITE, address, &writeValue, 1);
    }

    inline void writeSequence(Address address, const uint8_t* writeValues, uint8_t nBytes)
    {
        writeCommand(INSTRUCTION_WRITE, address, writeValues, nBytes);
    }

    inline void loadTxBuffer(uint8_t bufferNo, LoadTo loadTo, const uint8_t* writeValues, uint8_t nBytes)  // bufferNo is 0, 1 or 2
    {
        uint8_t instruction = (uint8_t)loadTo | logicalShiftLeft(bufferNo, 1);
        writeCommand(instruction, NO_ADDRESS, writeValues, nBytes);
    }

    inline void requestToSend(uint8_t instruction)
    {
        writeCommand(instruction, NO_ADDRESS, 0, 0);
    }

    inline void requestToSendOneBuffer(uint8_t bufferNo) // bufferNo is 0, 1 or 2
    {
        requestToSend(mcp2515::INSTRUCTION_REQUEST_TO_SEND | logicalShiftLeft(1, bufferNo));
    }

    inline uint8_t readStatus()
    {
        uint8_t readValue;
        readCommand(INSTRUCTION_READ_STATUS, NO_ADDRESS, &readValue, 1);
        return readValue;
    }

    inline uint8_t rxStatus()
    {
        uint8_t readValue;
        readCommand(INSTRUCTION_RX_STATUS, NO_ADDRESS, &readValue, 1);
        return readValue;
    }

    inline void bitModify(Address address, uint8_t mask, uint8_t data)
    {
        uint8_t writeValues[2] = { mask, data };
        writeCommand(INSTRUCTION_BIT_MODIFY, address, writeValues, 2);
    }

#endif

} // namespace mcp2515
