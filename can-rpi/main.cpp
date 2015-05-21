#include "precomp.h"
#include "mcp2515.h"

using namespace std;

struct Packet
{
    uint8_t properties[2];  // [ PRI 1:0, DIR, OP 4:0 ]  [ OP 7:5, 010, MPM 1:0 ]
    uint8_t nodeAddress;  // OP is always defined in the description of this node; if DIR=1, then it is also the sender
    uint8_t callerAddress;  // the node that initiated the communication with the node above; BROADCAST_ADDRESS for events
    uint8_t size;  // number of bytes used in data; can be used to differentiate overloads of OP
    uint8_t data[8];  // payload; the first byte of the first packet of a multi-packet message contains the number of packets
};

// PRI: priority (2 bits)
enum Priority
{
    LOW_PRIORITY = 0x00, //0b0000 0000,
    NORMAL_PRIORITY = 0x40, //0b0100 0000,
    HIGH_PRIORITY = 0x80, //0b1000 0000,
    CRITICAL_PRIORITY = 0xC0  //0b1100 0000;
};

// DIR: direction bit
enum Direction
{
    SENDER = 0x20,
    RECEIVER = 0x00
};
const uint8_t DIR_MASK = SENDER;

// MPM: multi-packet messages
enum MultiPacket
{
    SINGLE_PACKET = 0x00,
    START_MULTI_PACKET = 0x01,
    IN_MULTI_PACKET = 0x02,
    END_MULTI_PACKET = 0x03,
};
const uint8_t MPM_MASK = END_MULTI_PACKET;

const uint8_t BROADCAST_ADDRESS = 0xFF; // this is also used for a node without address yet
const uint8_t OP_MASK[2] = { 0x1F, 0xE0 };
const uint8_t EXTENDED_ID = 0x08;

const uint8_t OP_MAC = 0x00;
const uint8_t OP_LED = 0x01;

// This is supposed to emulate object=oriented programming.
// Let's say we have a proximity sensor:
//   class ProximitySensor
//   {
//   public:
//       // properties
//       uint8_t threshold() const;                  // OP=THRESHOLD size=0
//       void    threshold(uint8_t newThreshold);    // OP=THRESHOLD size=1
//       
//       uint8_t distance() const;                   // OP=DISTANCE size=0
//       // some properties can be read-only
//       
//       // method: same thing as property, with only the "set" function
//       
//       // events
//       void inRange(uint8_t distance);          // OP=IN_RANGE size=1
//   };
//
// The master can call proximityThreshold.threshold(42) by sending this packet:
//   properties[0] = (THRESHOLD & OP_MASK[0]) | NORMAL_PRIORITY | RECEIVER;
//   properties[1] = (THRESHOLD & OP_MASK[1]) | EXTENDED_ID;
//   nodeAddress   = sensor.address
//   callerAddress = master.address
//   size          = 1
//   data[0]       = 42
//
// The master can call distance = proximityThreshold.distance() by sending this:
//   properties[0] = (DISTANCE & OP_MASK[0]) | NORMAL_PRIORITY | RECEIVER;
//   properties[1] = (DISTANCE & OP_MASK[1]) | EXTENDED_ID;
//   nodeAddress   = sensor.address
//   callerAddress = master.address
//   size          = 0
// and the sensor will answer back with this:
//   properties[0] = (DISTANCE & OP_MASK[0]) | NORMAL_PRIORITY | SENDER;
//   properties[1] = (DISTANCE & OP_MASK[1]) | EXTENDED_ID;
//   nodeAddress   = sensor.address
//   callerAddress = master.address
//   size          = 1
//   data[0]       = 42
// Note that the only difference, except for the payload, is the DIR bit.
//
// There are a few OPs that are supported by all devices. For example, MAC to
// get or set the MAC address of the device. The master can get all MAC
// addresses with a single broadcast call:
//   properties[0] = (MAC & OP_MASK[0]) | NORMAL_PRIORITY | RECEIVER;
//   properties[1] = (MAC & OP_MASK[1]) | EXTENDED_ID;
//   nodeAddress   = BROADCAST_ADDRESS
//   callerAddress = master.address
//   size          = 0
// Nodes would respond by changing the DIR bit and setting their nodeAddress.
//
// Finally, the node can trigger a master callback using this message:
//   properties[0] = (IN_RANGE & OP_MASK[0]) | NORMAL_PRIORITY | SENDER;
//   properties[1] = (IN_RANGE & OP_MASK[1]) | EXTENDED_ID;
//   nodeAddress   = sensor.address
//   callerAddress = BROADCAST_ADDRESS
//   size          = 1
//   data[0]       = 42
// Since there is no specific caller (there could be no one interested, or many
// subscribers), the callerAddress is set to BROADCAST_ADDRESS.

const uint8_t NODE_MAC[] = { 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78 };
const uint8_t MAC_ADDRESS[] = { 0x87, 0x76, 0x65, 0x54, 0x43, 0x32, 0x21 };
const uint8_t ADDRESS_MASK[] = { DIR_MASK, 0x00, 0x00, 0x00 };                        // Apply filters on the direction bit only
const uint8_t ADDRESS_FILTER[] = { SENDER, EXTENDED_ID, 0x00, 0x00 };                   // Accepts all extended messages, as long as a node sends it

const uint8_t ADDRESS_BYTE = 2;

// Timings. Fosc = 7.3728 MHz / 64 = 115.2 kbps = Fbit = CAN nominal bit rate (NBR)
// Prescaler BRP = 1 ==> Tq = 4/Fosc
// Nominal bit time = Tbit = 1/Fbit = 64 / Fosc = 16 Tq
// Tbit = Tsyncseg + Tpropseg + Tps1 + Tps2
// Tsynseg = 1 Tq (fixed)
// Constraints: Tpropseg + Tps1 >= Tps2, 1 <= Tps1 <= 8, 2 <= Tps2 <= 8, Sum(T) = 16 Tq
// Sample point should be at 60-70% according to MCP2515, 87.5% according to CANopen, DeviceNet and VSCP.
// Because we are sampling 3 times, let's set the sample point at 69% == 11 Tq ==> Tps2 = 5 Tq
// Tpropseg + Tps1 = 10 (>= Tps2)
// Tpropseg = 3 Tq (1-8, propagation delays)
// Tps1 = 7 Tq (1-8)
// CNF3 = (SOF) (WAKFIL) (0, 0, 0) (PHSEG2 2:0)    = 0 0 0 4  (Tps2 = 4+1 Tq)
// CNF2 = (BTLMODE) (SAM) (PHSEG1 2:0) (PRSEG 2:0) = 1 1 6 2  (Tps1 = 6+1 Tq, Tpropseg = 2+1 Tq)
// CNF1 = (SJW 1:0) (BRP 5:0)                      = 0 0      (SJW = 1 Tq, BRP = 0 ==> Tq = 4/Fosc)
const uint8_t TIMINGS[] = { 0x04, 0xF2, 0x01 }; // For 230.4 kbps, replace 0x01 by 0x00

                                                // Interrupts when receiving a message or when there is an error.
const uint8_t INTERRUPTS = 0x23; // MERRE, WAKIE, ERRIE, TX2IE, TX1IE, TX0IE, RX1IE, RX0IE

                                 // RMX = 10 ==> receive only extended frames, BUKT = 1 ==> rollover from RXB0 to RXB1
                                 // Rest is read-only. The same value can be used for both RXB0 and RXB1 because the
                                 // bit in RXB1CTRL corresponding to BUKT in RXB0CTRL is read-only.
const uint8_t RECV_CTRL = 0x44; // RXB0CTRL: (0) (RMX 1:0) (0) (RXRTR) (BUKT) (BUKT1) (FILHIT0)
                                // RXB1CTRL: (0) (RMX 1:0) (0) (RXRTR) (FILHIT2:0)

void fatalError(const char* message)
{
    cout << message << endl;

    // Wait indefinitely.
    while (true) {}
}

int main()
{
	return 0;
}



#if 0

uint8_t waitForInterruptAndHandleErrors(/*uint8_t unexpectedInterruptFlags*/)
{
    mcp2515::Status status = mcp2515::waitForInterrupt(TIMEOUT_TIMER_SLOT_NO);
    if (status == mcp2515::TIMEOUT)
        return 0;

    uint8_t interruptFlags = mcp2515::read(mcp2515::CANINTF); // read interrupt flags

                                                              // Remove transmit interrupt flags
    interruptFlags &= 0xE3;

    if ((interruptFlags & 0xDC) != 0)                         // if an unexpected interrupt occurred, error.
    {
        // unexpected interrupts 0xDC: MERRF, WAKIF, TX2IF, TX1IF, TX0IF
        lcd::print(SET_CUR_POS_START_LINE_1 "CANINTF: ");
        lcd::print(interruptFlags);
        lcd::print("    ");
        fatalError(SET_CUR_POS_START_LINE_2 "** unexpect intr");
    }
    if ((interruptFlags & 0x20) != 0)                         // error interrupt!
    {
        uint8_t errorFlags = mcp2515::read(mcp2515::EFLG);
        lcd::print(SET_CUR_POS_START_LINE_1 "EFLG: ");
        lcd::print(errorFlags);
        lcd::print("       ");
        fatalError(SET_CUR_POS_START_LINE_2 "** error intr   ");
    }
    return interruptFlags;
}

inline uint8_t getRxBufferNo(uint8_t interruptFlags) // interruptFlags can be 1, 2 or 3
{
    // map 0b11 and 0b10 to 1, 0b00 to 0
    return logicalShiftRight(interruptFlags, 1);
}

const uint8_t TX_FULL = 0xFF;

inline uint8_t getTxBufferNo(uint8_t status)
{
    if (BIT_IS_CLEAR(status, 2))
        return 0;
    else if (BIT_IS_CLEAR(status, 4))
        return 1;
    else if (BIT_IS_CLEAR(status, 6))
        return 2;
    else
        return TX_FULL;
}

const timer::TickType ASK_ADDRESS_TIMEOUT = 2000; // 2s

Packet g_txPacket;
Packet g_rxPacket;

inline void setTxProperties(uint8_t Op, Priority priority, Direction direction)
{
    g_txPacket.properties[0] = (Op & OP_MASK[0]) | static_cast<uint8_t>(priority) | static_cast<uint8_t>(direction);
    g_txPacket.properties[1] = (Op & OP_MASK[1]) | EXTENDED_ID;
}

// TODO should this be inline?
uint8_t getRxOp()
{
    return ((g_rxPacket.properties[0] & OP_MASK[0]) | (g_rxPacket.properties[1] & OP_MASK[1]));
}

void rxSanityCheck()
{
    // Ensure mcp2515 filtered the message correctly.
    if ((g_rxPacket.properties[0] & DIR_MASK) != SENDER)
    {
        fatalError(SET_CUR_POS_START_LINE_2 "** RX check fail");
    }
}

void comparePackets(uint8_t errorMsgInt)
{
    const uint8_t* p1 = reinterpret_cast<uint8_t*>(&g_rxPacket);
    const uint8_t* p2 = reinterpret_cast<uint8_t*>(&g_txPacket);
    for (uint8_t i = 0; i < g_txPacket.size + 5; ++i)
        if (p1[i] != p2[i])
        {
            lcd::print(SET_CUR_POS_START_LINE_1 "FAIL: "); lcd::print(i); lcd::print("    "); lcd::print(p1[i]);
            fatalError(SET_CUR_POS_START_LINE_2 "** FAIL         ");
        }
};

int main(void)
{
    // Just in case, wait for everything to boot (LCD, MCP2515)
    _delay_ms(1000.0);

    // Initialize AVR.
    // TODO power management: turn everything off, tri-state all pins
    leds::initialize();
    buttons::initialize();
    mcp2515::initialize();
    lcd::initialize();

    // Ready to start, enable interrupts.
    sei();
    timer::start();

    leds::set(PIN_LED_ORANGE, leds::SOLID);
    lcd::print(CLEAR_LCD SET_CUR_POS_START_LINE_1 "Initializing... ");

    // Reset the MCP2515 as required by the spec.
    mcp2515::reset();
    _delay_ms(100.0);

    // Check that SPI communication is working.
    uint8_t canStatus = mcp2515::read(mcp2515::CANSTAT);
    if (canStatus != 0x80 || mcp2515::hasPendingInterrupt()) // 0x80: configuration mode, no interrupt yet
    {
        fatalError(SET_CUR_POS_START_LINE_2 "** No MCP2515   ");
    }

    // Configure the MCP2515.
    mcp2515::writeSequence(mcp2515::RXM0SIDH, ADDRESS_MASK, ARRAY_SIZE(ADDRESS_MASK));
    mcp2515::writeSequence(mcp2515::RXM1SIDH, ADDRESS_MASK, ARRAY_SIZE(ADDRESS_MASK));

    mcp2515::writeSequence(mcp2515::RXF0SIDH, ADDRESS_FILTER, ARRAY_SIZE(ADDRESS_FILTER));
    mcp2515::writeSequence(mcp2515::RXF1SIDH, ADDRESS_FILTER, ARRAY_SIZE(ADDRESS_FILTER));
    mcp2515::writeSequence(mcp2515::RXF2SIDH, ADDRESS_FILTER, ARRAY_SIZE(ADDRESS_FILTER));
    mcp2515::writeSequence(mcp2515::RXF3SIDH, ADDRESS_FILTER, ARRAY_SIZE(ADDRESS_FILTER));
    mcp2515::writeSequence(mcp2515::RXF4SIDH, ADDRESS_FILTER, ARRAY_SIZE(ADDRESS_FILTER));
    mcp2515::writeSequence(mcp2515::RXF5SIDH, ADDRESS_FILTER, ARRAY_SIZE(ADDRESS_FILTER));

    mcp2515::writeSequence(mcp2515::CNF3, TIMINGS, ARRAY_SIZE(TIMINGS));

    mcp2515::write(mcp2515::CANINTE, INTERRUPTS);
    mcp2515::write(mcp2515::RXB0CTRL, RECV_CTRL);
    mcp2515::write(mcp2515::RXB1CTRL, RECV_CTRL);

    // Switch MCP2515 ON (normal mode).
    mcp2515::bitModify(mcp2515::CANCTRL, 0xE0, 0x00);

    leds::set(PIN_LED_GREEN, leds::SOLID);
    lcd::print(SET_CUR_POS_START_LINE_1 "Ready! Press btn");
    buttons::waitForButtonPress();

    // Wait for the node to ask address
    leds::set(PIN_LED_GREEN, leds::SOLID);
    lcd::print(SET_CUR_POS_START_LINE_1 "Wait MAC        ");

WAIT_FOR_ADDRESS:
    uint8_t interruptFlags = waitForInterruptAndHandleErrors(); // return 0, 1, 2 or 3
    if (interruptFlags == 0) // not yet, ask again
        goto WAIT_FOR_ADDRESS;

    // Read the answer.
    uint8_t rxBufferNo = getRxBufferNo(interruptFlags);
    mcp2515::readRxBuffer(rxBufferNo, mcp2515::FROM_IDENTIFIER, reinterpret_cast<uint8_t*>(&g_rxPacket), sizeof(g_rxPacket));
    rxSanityCheck();

    // fill g_txPacket with expected packet
    setTxProperties(OP_MAC, HIGH_PRIORITY, SENDER);
    //g_txPacket.properties[1] &= ~EXTENDED_ID;
    g_txPacket.nodeAddress = BROADCAST_ADDRESS;
    g_txPacket.callerAddress = BROADCAST_ADDRESS;
    g_txPacket.size = 7;
    for (uint8_t i = 0; i < 7; ++i)
        g_txPacket.data[i] = NODE_MAC[i];
    comparePackets(0);

    // send address packet
    _delay_ms(500.0);
    uint8_t status = mcp2515::readStatus();
    uint8_t txBufferNo = getTxBufferNo(status);

    if (txBufferNo != TX_FULL)
    {
        setTxProperties(OP_MAC, HIGH_PRIORITY, RECEIVER);
        g_txPacket.nodeAddress = BROADCAST_ADDRESS;
        g_txPacket.callerAddress = 0;
        g_txPacket.size = 8;
        for (uint8_t i = 0; i < 7; ++i)
            g_txPacket.data[i] = NODE_MAC[i];
        g_txPacket.data[7] = 42;

        mcp2515::loadTxBuffer(txBufferNo, mcp2515::TO_IDENTIFIER, reinterpret_cast<uint8_t*>(&g_txPacket), g_txPacket.size + 5);
        mcp2515::requestToSendOneBuffer(txBufferNo);
        lcd::print(SET_CUR_POS_START_LINE_1 "Set address 42  ");
    }
    else
    {
        fatalError(SET_CUR_POS_START_LINE_2 "** TX full      ");
    }

    // send command
    _delay_ms(500.0);
    status = mcp2515::readStatus();
    txBufferNo = getTxBufferNo(status);

    if (txBufferNo != TX_FULL)
    {
        setTxProperties(OP_LED, NORMAL_PRIORITY, RECEIVER);
        g_txPacket.nodeAddress = 42;
        g_txPacket.callerAddress = 0;
        g_txPacket.size = 2;
        g_txPacket.data[0] = PIN_LED_ORANGE;
        g_txPacket.data[1] = static_cast<uint8_t>(leds::SOLID);

        mcp2515::loadTxBuffer(txBufferNo, mcp2515::TO_IDENTIFIER, reinterpret_cast<uint8_t*>(&g_txPacket), g_txPacket.size + 5);
        mcp2515::requestToSendOneBuffer(txBufferNo);
        lcd::print(SET_CUR_POS_START_LINE_1 "Set orange LED  ");
    }
    else
    {
        fatalError(SET_CUR_POS_START_LINE_2 "** TX full      ");
    }

    while (true) {}
}

#endif