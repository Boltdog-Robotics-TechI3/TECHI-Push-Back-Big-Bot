//#include "globals.hpp"
#include "main.h"
#include "pros/misc.h"

void intakeInitialize() {}

//Intake functions 

void intakeOnAll() {
    intake.move(127);
    motor19.move(127);
    motor11.move(127);
    motor20.move(127);
}

//unused for now
void intakeOnAllAlternative() {
    intake.move(127);
    //motor19.move(-127);
    motor11.move(127);
    motor20.move(127);
    //18 normal
    middleScore.move(127);
    //17 inverted
    lowerTopScore.move(-127);
    //16 normal
    topScore.move(127);
}

void intakeOffAll() {
    intake.move(0);
    motor19.move(0);
    motor11.move(0);
    middleScore.move(0);
    topScore.move(0);
    lowerTopScore.move(0);
    motor20.move(0);
}

void intakeMiddleScore() {
    middleScore.move(38.1);
    motor19.move(-127);
    motor11.move(-127);
    motor20.move(38.1);
    lowerTopScore.move(127);
}

void intakeTopScore() {
    middleScore.move(-127);
    motor19.move(-127);
    motor11.move(-127);
    topScore.move(127);
    lowerTopScore.move(127);
    motor20.move(127);
}

void outtake() {
    intake.move(-67);
    motor19.move(-67);
    motor11.move(-67);
    motor20.move(-67);
}

void intakePeriodic() {
    //All intake
    if (controller.get_digital(DIGITAL_L2)) {
        intakeOnAll();
    }
    //middlescore
    else if (controller.get_digital(DIGITAL_R2)) {
        intakeMiddleScore();
    }
    //topscore
    //16 back 17 back 18 for 19 back
    else if (controller.get_digital(DIGITAL_R1)) {
        intakeTopScore();
    }
    //outtake
    else if (controller.get_digital(DIGITAL_X)) {
        outtake();
    }
    else {
        intakeOffAll();
    }
}
