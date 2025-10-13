#include <gtest/gtest.h>                              // for Test, TestInfo, ASSERT_...
#include <exception>                                  // for exception
#include <iostream>                                   // for operator<<, bas...
#include <string>                                     // for allocator, char...
#include "hardware_interface/resource_manager.hpp"    // for ResourceManager
#include "ros2_control_test_assets/descriptions.hpp"  // for urdf_head, urdf...

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  TestableResourceManager()
  : hardware_interface::ResourceManager() {}

  TestableResourceManager(
    const std::string & urdf, bool validate_interfaces = true, bool activate_all = false)
  : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  {
  }
};

class TestGenericSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_2dof_ =
      R"(
  <ros2_control name="ReseqHardwareSystem" type="system">
    <hardware>
      <plugin>reseq_hardware/ReseqHardware</plugin>
      <param name="num_modules">2</param>
      <param name="can_interface">vcan0</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.7854</param>
      </state_interface>
    </joint>
  </ros2_control>
)";
  }
  void TearDown() override
  {
  }
  std::string hardware_system_2dof_;
};

TEST_F(TestGenericSystem, load_generic_system_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_2dof_ +
    ros2_control_test_assets::urdf_tail;
  try {
    TestableResourceManager rm(urdf);
  } catch (const std::exception & e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    FAIL() << "Exception during test: " << e.what();
  }
}


