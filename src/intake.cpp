#include "main.h"

void intakeInitialize() {}

void intakePeriodic() {
    if (controller.get_digital(DIGITAL_L1)) {
        intake.move(127);
        lift.move(127);
        low_sorter.move(-127);
        ejector.move(127);
        high_sorter.move(-127);
    }
    else if (controller.get_digital(DIGITAL_A)) {
        intake.move(127);
        lift.move(127);
        low_sorter.move(127);
        ejector.move(127);
        high_sorter.move(127); m
    }
    else {
        intake.move(0);
        lift.move(0);
        low_sorter.move(0);
        ejector.move(0);
        high_sorter.move(0);
    }
}
