
#include "fsm/state_machine.hpp"
#include "fsm/transitions.hpp"
#include "robot/MyRobot.hpp"
#include "test/test.hpp"

static void test_main();

// #define TEST

int main(int argc, char* argv[]) {
#ifdef TEST
  test_main();
#else
  auto my_robot = std::make_unique<MyRobot>();

  Controller<MyRobot, state_variant> controller {std::move(my_robot), Localization()};
  controller.run();
#endif

  return 0;
}

static void test_main() {
  // *** Test angle class ***
  auto const angle1 = math::Angle {10};
  auto const angle2 = math::Angle {350};
  auto const angle3 = math::Angle {-10};
  auto const angle4 = math::Angle {20};
  auto const angle5 = math::Angle {0};

  CHECK_EQUALS(angle2, angle3);
  CHECK_EQUALS(angle1.diff(angle4), 10);
  CHECK_EQUALS(angle1.diff(angle2), 20);
  CHECK_EQUALS(angle2.diff(angle1), 20);
  CHECK_EQUALS(angle1.signedDiff(angle5), -10);
  CHECK_EQUALS(angle4.signedDiff(angle1), -10);

  auto pid = Pid({200, -200, 0.1, 0.01, 0.5});

  double val = 135;
  for (int i = 0; i < 1000; i++) {
    double const inc = pid.calculate(0, val);
    std::printf("val:% 7.3f inc:% 7.3f\n", val, inc);
    val += inc;
  }
}
