#include <gtest/gtest.h>
#include <string>
#include "meca500_hardware/meca500_system.hpp"

namespace meca500_hardware
{

class TestMeca500SystemHardware : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }
};

class Meca500SystemHardwareTestable : public Meca500SystemHardware
{
public:
  using Meca500SystemHardware::parse_response;
  using Meca500SystemHardware::parse_joint_values;
};

TEST_F(TestMeca500SystemHardware, ParseResponseValid)
{
  int code = 0;
  std::string data;

  EXPECT_TRUE(
    Meca500SystemHardwareTestable::parse_response(
      "[2007][12.3,0.0,-5.1,0.0,90.0,0.0]", code, data));
  EXPECT_EQ(code, 2007);
  EXPECT_EQ(data, "12.3,0.0,-5.1,0.0,90.0,0.0");

  EXPECT_TRUE(Meca500SystemHardwareTestable::parse_response("[3000]", code, data));
  EXPECT_EQ(code, 3000);
  EXPECT_TRUE(data.empty());
}

TEST_F(TestMeca500SystemHardware, ParseResponseInvalid)
{
  int code = 0;
  std::string data;

  EXPECT_FALSE(Meca500SystemHardwareTestable::parse_response("2007", code, data));
  EXPECT_FALSE(Meca500SystemHardwareTestable::parse_response("[ABC]", code, data));
  EXPECT_FALSE(Meca500SystemHardwareTestable::parse_response("[2007", code, data));
}

TEST_F(TestMeca500SystemHardware, ParseJointValuesValid)
{
  double values[6];

  // 6 values
  EXPECT_TRUE(
    Meca500SystemHardwareTestable::parse_joint_values(
      "12.3,0.0,-5.1,0.0,90.0,0.5", values));
  EXPECT_NEAR(values[0], 12.3, 1e-5);
  EXPECT_NEAR(values[5], 0.5, 1e-5);

  // 7 values (timestamp)
  EXPECT_TRUE(
    Meca500SystemHardwareTestable::parse_joint_values(
      "123456,12.3,0.0,-5.1,0.0,90.0,0.5", values));
  EXPECT_NEAR(values[0], 12.3, 1e-5);
  EXPECT_NEAR(values[5], 0.5, 1e-5);
}

TEST_F(TestMeca500SystemHardware, ParseJointValuesInvalid)
{
  double values[6];

  EXPECT_FALSE(Meca500SystemHardwareTestable::parse_joint_values("1,2,3,4,5", values));
  EXPECT_FALSE(Meca500SystemHardwareTestable::parse_joint_values("1,2,3,4,5,6,7,8", values));
  EXPECT_FALSE(Meca500SystemHardwareTestable::parse_joint_values("1,2,3,abc,5,6", values));
}

}  // namespace meca500_hardware
