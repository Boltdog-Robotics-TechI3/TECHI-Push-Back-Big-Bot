//#include "globals.hpp"
#include "main.h"
#include "pros/misc.h"

void intakeInitialize() {}

//Intake functions 

void intakeOnAll() {
    intake.move(127);
}

void intakeOffAll() {
    intake.move(0);
    middleScore.move(0);
    topScore.move(0);

}

void intakeMiddleScore() {
    middleScore.move(127);
    intake.move(127);

}

void intakeTopScore() {
    middleScore.move(-127);
    topScore.move(90);
    intake.move(217);
}

void outtake() {
    intake.move(-127);
}

void loadingUp(){ 
    loaderMech.move(127);
}

void loadingDown(){
    loaderMech.move(-127);
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

    // This is the loader mech for the intake, keep in mind this can break
    // If it is run in one direction for too long
    if(controller.get_digital(DIGITAL_UP)){
        loadingUp();
    } else if(controller.get_digital(DIGITAL_DOWN)){
        loadingDown();
    }
}
