//#include "globals.hpp"
#include "main.h"
#include "pros/misc.h"

void intakeInitialize() {}

void intakePeriodic() {
    //All intake
    if (controller.get_digital(DIGITAL_L2)) {
        intake.move(127);
        motor19.move(127);
        motor11.move(127);
        motor20.move(127);
        // lift.move(127);
        // low_sorter.move(-127);
        // ejector.move(127);
        // high_sorter.move(-127);
    }
    //middlescore
    else if (controller.get_digital(DIGITAL_R2)) {
        middleScore.move(38.1);
        motor19.move(-127);
        motor11.move(-127);
        motor20.move(38.1);
    }
    //topscore
    //16 back 17 back 18 for 19 back
    else if (controller.get_digital(DIGITAL_R1)) {
        middleScore.move(-127);
        motor19.move(-127);
        motor11.move(-127);
        topScore.move(127);
        motor20.move(127);
    }
    //outtake
    else if (controller.get_digital(DIGITAL_X)) {
        intake.move(-127);
        motor19.move(-127);
        motor11.move(-127);
        motor20.move(-127);
    }
    else {
        intake.move(0);
        motor19.move(0);
        motor11.move(0);
        middleScore.move(0);
        topScore.move(0);
        motor20.move(0);
        // lift.move(0);
        // low_sorter.move(0);
        // ejector.move(0);
        // high_sorter.move(0);
    }
}
