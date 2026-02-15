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
    topScore.move(60);
    intake.move(217);
}

void outtake() {
    intake.move(-127);
}

void loadingUp(){ 
    loaderMech.set_brake_mode(MOTOR_BRAKE_HOLD);
    loaderMech.move(127);
}

void loadingDown(){
    loaderMech.set_brake_mode(MOTOR_BRAKE_COAST);
    loaderMech.move(-127);
    
}


void intakePeriodic() {
    //All intake
    if (controller.get_digital(DIGITAL_L2)) {
        intakeOnAll();
    }
    //middlescore
    else if (controller.get_digital(DIGITAL_R2)) {
        intakeTopScore();
    }
    //topscore
    //16 back 17 back 18 for 19 back
    else if (controller.get_digital(DIGITAL_R1)) {
    
        intakeMiddleScore();
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
    } else{
        loaderMech.move(0);
    }

}
