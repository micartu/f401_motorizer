#include "CppUTest/TestHarness.h"
#include "remote.h"
#include <string.h>

#define NOT_USED 0xAA
#define BUF_SZ 32
#define SAMPLE_DATA PACK_BEGIN, 7, CMD_TYPE_SET_K, 1, 1, 0, 0, 0x7, PACK_END
#define SAMPLE_PACKET { SAMPLE_DATA }

static uint32_t _kgain = 0;
static size_t _kgain_called = 0;

// needed headers + and implementations which control fake's behavior:
extern "C"
{
    void _install_data_for_UART_receive(const uint8_t * data, uint16_t size);
    void _install_ret_type_for_UART_receive(HAL_StatusTypeDef ret);
    void _deinstall_data_for_UART_receive();
    uint16_t _return_last_read_size();
}

static void k_callback(uint32_t k)
{
    _kgain = k;
    ++_kgain_called;
}

TEST_GROUP(RemoteTestGroup)
{
    struct UART_Descr com;
    UART_HandleTypeDef huart;
    uint8_t buf[BUF_SZ];

    void setup()
    {
        memset(buf, NOT_USED, sizeof(buf));
        init_communication_uart(&com, &huart, buf, sizeof(buf));
        _kgain = 0;
        _kgain_called = 0;
    }

    void teardown()
    {
        deinit_communication_uart(&com);
        _deinstall_data_for_UART_receive();
    }
};

TEST(RemoteTestGroup, InitializationTest)
{
    POINTERS_EQUAL(&huart, com.huart);
    POINTERS_EQUAL(buf, com.uart_buffer);
    LONGS_EQUAL(WFIRST_BYTE, com.rstate);
}

TEST(RemoteTestGroup, ChangeState1stPacket)
{
    // given
    const uint8_t data[] = SAMPLE_PACKET;

    // when
    _install_data_for_UART_receive(data, sizeof(data));

    // then
    // we've received first byte
    // and waiting the 2nd: length of the rest of packet
    LONGS_EQUAL(WLEN_BYTE, com.rstate);
    LONGS_EQUAL(PACK_BEGIN, com.uart_buffer[0]);
}

TEST(RemoteTestGroup, ChangeState2ndPacket)
{
    // given
    const uint8_t data[] = SAMPLE_PACKET;

    // when
    _install_data_for_UART_receive(data, sizeof(data));
    // we received data over UART for the 2nd time:
    uart_irq_handler(&huart);

    // then
    LONGS_EQUAL(WREST_BYTES, com.rstate);
}

TEST(RemoteTestGroup, ChangeStateAmountOfDataToBeRead)
{
    // given
    const uint8_t len = 4;
    const uint8_t data[] = { PACK_BEGIN, len, CMD_TYPE_SET_K, 0, 0, PACK_END };

    // when
    _install_data_for_UART_receive(data, sizeof(data));
    // we received data over UART (2 + 1) times
    for (size_t i = 0; i < 2; ++i)
        uart_irq_handler(&huart);

    // then
    LONGS_EQUAL(len, _return_last_read_size());
}

TEST(RemoteTestGroup, CorrectReadBuffer)
{
    // given
    const uint8_t amount = 4;
    const uint8_t data[] = { PACK_BEGIN, amount, CMD_TYPE_SET_K, 0, 0, PACK_END };

    // when
    _install_data_for_UART_receive(data, sizeof(data));
    // we received data over UART (2 + 1) times
    for (size_t i = 0; i < 2; ++i)
        uart_irq_handler(&huart);

    // then
    LONGS_EQUAL(amount, _return_last_read_size());
    for (size_t i = 0; i < sizeof(data); ++i)
        LONGS_EQUAL(data[i], com.uart_buffer[i]);
}

TEST(RemoteTestGroup, ReceiveTooBigPacket)
{
     // given
     // two packets worth of data
     const uint8_t data[] = {PACK_BEGIN, BUF_SZ * 2, CMD_TYPE_SET_K, 0, 0, PACK_END };

     // when
     _install_data_for_UART_receive(data, sizeof(data));
     // we received data over UART (2 + 1) times
     for (size_t i = 0; i < 2; ++i)
         uart_irq_handler(&huart);
     // call runloop handler in order to parse
     // the received data:
     uart_runloop_handler();
     // call irq once more in order to receive first byte
     // of next packet
     uart_irq_handler(&huart);

     // then
     LONGS_EQUAL(0, com.uart_ptr);
     LONGS_EQUAL(WFIRST_BYTE, com.rstate);
}

TEST(RemoteTestGroup, ReceiveTooManyPacketsDropOthers)
{
     // given
     const uint8_t chunk[] = SAMPLE_PACKET;
     const size_t csz = sizeof(chunk);
     uint8_t data[BUF_SZ * 2];
     size_t pkt_cnt = 0;
     for (size_t i = 0; i < sizeof(data);)
     {
         if (i + csz < sizeof(data))
         {
             memcpy(data + i, chunk, csz);
             i += csz;
             ++pkt_cnt;
         }
         else
             break;
     }

     // when
     _install_data_for_UART_receive(data, sizeof(data));
     install_k_callback(&com, &k_callback);
     // we have to call irq rec. handler UART 3 * pkt_cnt - 1 times
     for (size_t i = 0; i < 3 * pkt_cnt - 1; ++i)
         uart_irq_handler(&huart);
     uart_runloop_handler();

     // then
     LONGS_EQUAL(pkt_cnt, _kgain_called);
}

TEST(RemoteTestGroup, ReceiveTooManyPacketsDropOthersParsingReceived)
{
     // given
     const uint8_t chunk[] = SAMPLE_PACKET;
     const size_t csz = sizeof(chunk);
     uint8_t data[BUF_SZ * 2];
     size_t pkt_cnt = 0;
     for (size_t i = 0; i < sizeof(data); )
     {
         if (i + csz < sizeof(data))
         {
             memcpy(data + i, chunk, csz);
             i += csz;
             ++pkt_cnt;
         } else
             break;
     }

     // when
     _install_data_for_UART_receive(data, sizeof(data));
     install_k_callback(&com, &k_callback);
     // we have to call irq rec. handler UART 3 * pkt_cnt - 1 times
     for (size_t i = 0; i < 3 * pkt_cnt - 1; ++i)
         uart_irq_handler(&huart);
     // call runloop handler in order to parse
     // the received data:
     uart_runloop_handler();

     // then
     LONGS_EQUAL(pkt_cnt, _kgain_called);
}

TEST(RemoteTestGroup, TimeOutHandling)
{
    // given
    const uint8_t data[] = { PACK_BEGIN, 0, PACK_END };

    // when
    _install_data_for_UART_receive(data, sizeof(data));
    // wait max amount of time in order to reset the state
    for (size_t i = 0; i < SYS_TIMEOUT_TICKS + 1; ++i)
        uart_systick_handler();
    uart_runloop_handler();

    // then
    // packet's kind of invalid, so everything must be reseted
    LONGS_EQUAL(0, com.uart_ptr);
    LONGS_EQUAL(0, com.not_parsed_sz);
}

TEST(RemoteTestGroup, RunLoopHaveDataToBeParsed)
{
     // given
    const uint8_t data[] = SAMPLE_PACKET;

    // when
    _install_data_for_UART_receive(data, sizeof(data));
    install_k_callback(&com, &k_callback);
    // we received data over UART (2 + 1) times
    for (size_t i = 0; i < 2; ++i)
        uart_irq_handler(&huart);

    // then
    LONGS_EQUAL(1, _kgain_called);
    // we have data to be parsed, since 
    // uart_runloop_handler wasn't called
    LONGS_EQUAL(com.not_parsed_sz, com.uart_ptr);
}

TEST(RemoteTestGroup, RunLoopParsedReceivedPacket)
{
     // given
    const uint8_t data[] = SAMPLE_PACKET;

    // when
    _install_data_for_UART_receive(data, sizeof(data));
    // we received data over UART (2 + 1) times
    for (size_t i = 0; i < 2; ++i)
        uart_irq_handler(&huart);
    // call runloop handler in order to parse
    // the received data:
    uart_runloop_handler();

    // then
    LONGS_EQUAL(0, com.not_parsed_sz);
}

TEST(RemoteTestGroup, RunLoopParsedReceivedPacketAndWaitNewOne)
{
     // given
     // seq. of two packets
     const uint8_t data[] = {
         PACK_BEGIN, 4, CMD_TYPE_SET_K, 0, 0, PACK_END,
         PACK_BEGIN, 1, PACK_END};

     // when
     _install_data_for_UART_receive(data, sizeof(data));
     // we received data over UART (2 + 1) times
     for (size_t i = 0; i < 2; ++i)
         uart_irq_handler(&huart);
     // call runloop handler in order to parse
     // the received data:
     uart_runloop_handler();
     // call irq once more in order to receive first byte of
     // the next packet
     uart_irq_handler(&huart);

     // then
     LONGS_EQUAL(1, com.uart_ptr);
}

// **** tests for getting/setting the data ****

TEST(RemoteTestGroup, SetKGain)
{
     // given
     // seq. of two packets
     const uint8_t data[] = SAMPLE_PACKET;

     // when
     _install_data_for_UART_receive(data, sizeof(data));
     install_k_callback(&com, &k_callback);
     // we received data over UART (2 + 1) times
     for (size_t i = 0; i < 2; ++i)
         uart_irq_handler(&huart);
     // call runloop handler in order to parse
     // the received data:
     uart_runloop_handler();

     // then
     LONGS_EQUAL(0x101, _kgain);
}

TEST(RemoteTestGroup, SetKGain2Times)
{
     // given
     // seq. of two packets
     const uint8_t data[] = { SAMPLE_DATA, SAMPLE_DATA };

     // when
     _install_data_for_UART_receive(data, sizeof(data));
     install_k_callback(&com, &k_callback);
     // we received data over UART (5 + 1) times
     for (size_t i = 0; i < 5; ++i)
         uart_irq_handler(&huart);
     // call runloop handler in order to parse
     // the received data:
     uart_runloop_handler();

     // then
     LONGS_EQUAL(0x101, _kgain);
     LONGS_EQUAL(2, _kgain_called);
}

TEST(RemoteTestGroup, OverflowReceiveRemoteBuffer)
{
    // given
    // receive buffer is overloaded with garbage data
    com.not_parsed_sz = sizeof(buf) - 1;
    com.uart_ptr = com.not_parsed_sz;
    com.rticks_elapsed = SYS_TIMEOUT_TICKS + 1;
    com.rstate = WREST_BYTES;
    const uint8_t data[] = SAMPLE_PACKET;
    install_k_callback(&com, &k_callback);
    const size_t kiters = 2;
    _deinstall_data_for_UART_receive();

    // when
    for (size_t i = 0; i < kiters; ++i)
    {
        _install_data_for_UART_receive(data, sizeof(data));
        for (size_t i = 0; i < 7; ++i)
            uart_irq_handler(&huart);
        // call runloop handler in order to parse
        // the received data:
        uart_runloop_handler();
    }

    // then
    // kgain should be set
    LONGS_EQUAL(0x101, _kgain);
    LONGS_EQUAL(kiters, _kgain_called);
    // reset the received state, wait for a new packet
    LONGS_EQUAL(WFIRST_BYTE, com.rstate);
    LONGS_EQUAL(0, com.uart_ptr);
    LONGS_EQUAL(0, com.not_parsed_sz);
}