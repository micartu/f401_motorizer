#include "CppUTest/TestHarness.h"
#include "rpm_measurer.h"

TEST_GROUP(CalculateRPMTestGroup)
{
    struct speed_measure_t rpm;

    void setup()
    {
        rpm_measurer_init(&rpm);
    }

    void teardown()
    {
    }
};

TEST(CalculateRPMTestGroup, CalculateRPMNormalCase)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = 0;

    // when
    position_changed(&rpm, dp);

    // then
    DOUBLES_EQUAL(dp, rpm.speed, 0.1);
    LONGS_EQUAL(dp, rpm.pos);
}

TEST(CalculateRPMTestGroup, CalculateRPMOverflow)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = MAX_WHEEL_POS - dp;

    // when
    rpm_measurer_positive_overflow(&rpm);
    position_changed(&rpm, dp);

    // then
    DOUBLES_EQUAL(2 * dp, rpm.speed, 0.1);
    LONGS_EQUAL(dp, rpm.pos);
    LONGS_EQUAL(0, rpm.overlow);
}

TEST(CalculateRPMTestGroup, CalculateRPMAfterOverflow)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = MAX_WHEEL_POS - dp;

    // when
    rpm_measurer_positive_overflow(&rpm);
    position_changed(&rpm, dp);
    // check if we correctly move over the end of the wheel
    position_changed(&rpm, 2 * dp);

    // then
    DOUBLES_EQUAL(dp, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMNegativeSpeed)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = dp;

    // when
    position_changed(&rpm, 0);

    // then
    DOUBLES_EQUAL(-dp, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMNegativeSpeedOverflow)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = dp;

    // when
    rpm_measurer_negative_overflow(&rpm);
    position_changed(&rpm, MAX_WHEEL_POS - dp);

    // then
    DOUBLES_EQUAL(-2 * dp, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMAfterNegativeSpeedOverflow)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = dp;

    // when
    rpm_measurer_negative_overflow(&rpm);
    position_changed(&rpm, MAX_WHEEL_POS - dp);
    position_changed(&rpm, MAX_WHEEL_POS - 2 * dp);

    // then
    DOUBLES_EQUAL(-dp, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMWithCoef)
{
    // given
    const uint16_t dp = 10;
    const float koef = 2;
    rpm_measurer_set_speed_coef(&rpm, koef);

    // when
    position_changed(&rpm, dp);

    // then
    DOUBLES_EQUAL(koef * dp, current_speed(&rpm), 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMDetectPositiveOverflow)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = MAX_WHEEL_POS - dp;
    rpm.speed = 1; // positive speed

    // when
    position_changed_detect_overflow(&rpm, dp);

    // then
    DOUBLES_EQUAL(2 * dp, rpm.speed, 0.1);
    LONGS_EQUAL(dp, rpm.pos);
    LONGS_EQUAL(0, rpm.overlow);
}

TEST(CalculateRPMTestGroup, CalculateRPMDetectUnderflow)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = dp;
    rpm.speed = -1; // negative speed

    // when
    position_changed_detect_overflow(&rpm, MAX_WHEEL_POS - dp);

    // then
    DOUBLES_EQUAL(-2 * dp, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMDetectUnderflowInit)
{
    // given
    const uint16_t dp = 10;
    rpm.pos = dp;
    // prev. speed is zero - it's right after the device is powered

    // when
    position_changed_detect_overflow(&rpm, MAX_WHEEL_POS - dp);

    // then
    DOUBLES_EQUAL(-2 * dp, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMDetectUnderflowExp0)
{
    // given
    const uint16_t pos = 1164;
    const uint16_t npos = 1065;
    rpm.speed = 10;
    rpm.pos = pos;
    rpm.speed_coef = -1;

    // when
    position_changed_detect_overflow(&rpm, npos);

    // then
    DOUBLES_EQUAL((npos - pos) * rpm.speed_coef, rpm.speed, 0.1);
}

TEST(CalculateRPMTestGroup, CalculateRPMDetectUnderflowExp1)
{
    // given
    const uint16_t pos = 70;
    const uint16_t npos = 65504;
    rpm.speed = 10;
    rpm.pos = pos;
    rpm.speed_coef = -1;

    // when
    position_changed_detect_overflow(&rpm, npos);

    // then
    DOUBLES_EQUAL((-MAX_WHEEL_POS + npos - pos) * rpm.speed_coef, rpm.speed, 0.1);
}