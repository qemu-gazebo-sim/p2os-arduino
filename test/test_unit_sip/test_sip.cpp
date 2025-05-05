#include <unity.h>

#include "sip.hpp"
#include "p2os_msgs.hpp"

byte data[] = {
  0xfa, 0xfb, 0x21, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x90, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04,
  0x00, 0x7d, 0x07, 0x00, 0x7d, 0x05, 0x00, 0x59, 0x30, 0xf0, 0x11, 0xb4
};

SIP* sip_packet;
nav_msgs::ros_p2os_data_t p2os_data;

void setUp(void) {}

void tearDown(void) {}

void test_sip_position() {
  sip_packet = new SIP(0);
  sip_packet->ParseStandard(&data[3]);
  sip_packet->FillStandard(&p2os_data);
  
  // p2os_data.position.pose
  TEST_ASSERT_EQUAL(p2os_data.position.pose.covariance[0], 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.pose.pose.position.x, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.pose.pose.position.y, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.pose.pose.position.z, 0.00);

  // p2os_data.position.twist
  TEST_ASSERT_EQUAL(p2os_data.position.twist.covariance[0], 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.twist.twist.angular.x, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.twist.twist.angular.y, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.twist.twist.angular.z, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.twist.twist.linear.x, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.twist.twist.linear.y, 0.00);
  TEST_ASSERT_EQUAL(p2os_data.position.twist.twist.linear.z, 0.0);
}

void test_sip_grip() {
  sip_packet = new SIP(0);
  sip_packet->ParseStandard(&data[3]);
  sip_packet->FillStandard(&p2os_data);
  
  // p2os_data.gripper.grip
  TEST_ASSERT_EQUAL(p2os_data.gripper.grip.state, 3);
  TEST_ASSERT_EQUAL(p2os_data.gripper.grip.dir, 1);
  TEST_ASSERT_EQUAL(p2os_data.gripper.grip.inner_beam, 0);
  TEST_ASSERT_EQUAL(p2os_data.gripper.grip.outer_beam, 0);
  TEST_ASSERT_EQUAL(p2os_data.gripper.grip.left_contact, 0);
  TEST_ASSERT_EQUAL(p2os_data.gripper.grip.right_contact, 0);

  // p2os_data.gripper.lift
  TEST_ASSERT_EQUAL(p2os_data.gripper.lift.state, 3);
  TEST_ASSERT_EQUAL(p2os_data.gripper.lift.dir, 0);
  TEST_ASSERT_EQUAL(p2os_data.gripper.lift.position, 0.0);
}

void test_sip_odom() {
  sip_packet = new SIP(0);
  sip_packet->ParseStandard(&data[3]);
  sip_packet->FillStandard(&p2os_data);
  
  // p2os_data.odom_trans
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.rotation.w, 0.0);
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.rotation.x, 0.0);
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.rotation.y, 0.0);
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.rotation.z, 0.0);
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.translation.x, 0.0);
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.translation.y, 0.0);
  TEST_ASSERT_EQUAL(p2os_data.odom_trans.transform.translation.z, 0.0);
}

void test_sip(void) {
    sip_packet = new SIP(0);
    sip_packet->ParseStandard(&data[3]);
    sip_packet->FillStandard(&p2os_data);
    
    // p2os_data.batt
    TEST_ASSERT_EQUAL(p2os_data.batt.voltage, 14.40);

    // p2os_data.motor
    TEST_ASSERT_EQUAL(p2os_data.motors.state, 0);

    // p2os_data.sonar.pose
    TEST_ASSERT_EQUAL(p2os_data.sonar.ranges_count, 8);
    // TEST_ASSERT_EQUAL(p2os_data.sonar.ranges[0], 28.06);

    // p2os_data.dio
    TEST_ASSERT_EQUAL(p2os_data.dio.count, 8);
    TEST_ASSERT_EQUAL(p2os_data.dio.bits, 48);

    // p2os_data.aio
    TEST_ASSERT_EQUAL(p2os_data.aio.voltages_count, 1);
    TEST_ASSERT_EQUAL(p2os_data.aio.voltages[0], 1.75);
}

void setup() {
  delay(2000);

  UNITY_BEGIN();

  RUN_TEST(test_sip_position);
  RUN_TEST(test_sip_grip);
  RUN_TEST(test_sip_odom);
  RUN_TEST(test_sip);

  UNITY_END();
}

void loop() {}