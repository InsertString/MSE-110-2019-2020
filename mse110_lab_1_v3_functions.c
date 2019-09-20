//#include "mse110_lab_1_v3.c"

//**************************************//
//   THIS FILE CONTAINS THE FUNCTIONS   //
//      AND CODE FOR MSE 110 LAB 1      //
//**************************************//


//macros for blue and green
#define BLUE 1
#define GREEN 2


//***********//
// VARIABLES //
//***********//


//PID variables
const float kp = 2.1;
const float ki = 0.3;
const float kd = 35;

int error = 0;
int past_error = 0;
int derivative = 0;
int integral = 0;
int integral_limit = 5;

int blue_target = 14;
int green_target = 14;

int const_speed = 15;
int max_speed = 30;

//Colour Values
long red = 0;
long blue = 0;
long green = 0;
int starting_colour = 0;

//other sensor vars
int final_turn_value = 180;
int green_turn_value = 50;
int blue_turn_value = 60;
int object_detection_range = 15;




//****************//
// CORE FUNCTIONS //
//****************//


void do_debugging() {
	displayCenteredBigTextLine(1, "sonic: %d   ", SensorValue[sonic]);
	displayCenteredBigTextLine(3, "gyro: %d  ", SensorValue[gyro]);
	displayCenteredBigTextLine(5, "red: %d   ", red);
	displayCenteredBigTextLine(7, "blue: %d   ", blue);
	displayCenteredBigTextLine(9, "green: %d   ", green);



}





//caps the value  of the input
int motorCap(int input, int max) {
	int out;
	if (input > max) {
		out = max;
	}
	else if (input < -max) {
		out = -max;
	}
	else {
		out = input;
	}
	return out;
}




//sets the speed of the drive motors
void setDrive(int left, int right) {
	setMotorSpeed(drive_left, left);
	setMotorSpeed(drive_right, right);
}




void setDriveCapped(int left, int left_cap, int right, int right_cap) {
	setMotorSpeed(drive_left, motorCap(left, left_cap));
	setMotorSpeed(drive_right, motorCap(right, right_cap));
}



//stops the drive motors
void stopDrive() {
	setDrive(0,0);
}




//sets line tracking variables
void setLineVars() {
	getColorRGB(line_tracker, red, blue, green);
}



//resets the gyro
void resetG() {
	resetGyro(gyro);
}





//*detects the starting colour of the line*//
//*******RUN AT START OF task main!!*******//
//****NEEDS TO BE TUNED FOR TAPE COLORS****//
void detectStartingColor() {
	setLineVars();
	if (blue > 20 && green < 20) {
		starting_colour = GREEN;
	}
	else if (blue < 20 && green < 20) {
		starting_colour = BLUE;
	}
}



//*********************************//
// TRACKS THE EDGE OF THE COLOURED //
//  TAPE USING A PID CONTROL LOOP  //
//*********************************//
void trackLine(int target) {
	do_debugging();
	setLineVars();
	int motor_power = 0;
	past_error = error;
	error = target - red;
	derivative = error - past_error;

	if (abs(error) > integral_limit && time1[T1] > 0) {
		integral += error;
		clearTimer(T1);
	}
	else if (abs(error) > integral_limit && time1[T1] < 0) {
		integral = integral;
	}
	else {
		integral = 0;
		clearTimer(T1);
	}

		if (red > 10) {
			clearTimer(T2);
		}

	//when the robot does a left turn it removes the constant speed so that it only spins in place
	if (time1[T2] > 30) {
		motor_power = (error * 1);
		setDrive(-motor_power, motor_power);
	}
	//other driving is handled by the PID with constant speed to auto correct for large right turns
	else {
		motor_power = (error * kp) + (integral*ki) + (derivative * kd);
		setDriveCapped(-motor_power + const_speed, max_speed, motor_power + const_speed, max_speed);
	}
}






//drive for a distance
void drive_cm_forward(int target) {
	setLineVars();
	resetMotorEncoder(drive_left);
	resetMotorEncoder(drive_right);

	bool right_done = false;
	bool left_done = false;

	int left_power = 10;
	int right_power = 10;

	while (right_done != true && left_done != true) {

		setDrive(left_power, right_power);

		if (fabs(getMotorEncoder(drive_left)) >= target) {
			left_done = true;
			left_power = -1;
		}

		if (fabs(getMotorEncoder(drive_right)) >= target) {
			right_done = true;
			right_power = -1;
		}
	}

	stopDrive();
}





//drive for a distance
void drive_cm_backward(int target) {
	setLineVars();
	resetMotorEncoder(drive_left);
	resetMotorEncoder(drive_right);

	bool right_done = false;
	bool left_done = false;

	int left_power = -10;
	int right_power = -10;

	while (right_done != true && left_done != true) {

		setDrive(left_power, right_power);

		if (fabs(getMotorEncoder(drive_left)) >= target) {
			left_done = true;
			left_power = 1;
		}

		if (fabs(getMotorEncoder(drive_right)) >= target) {
			right_done = true;
			right_power = 1;
		}
	}

	stopDrive();
}




//*********************//
// CODE  FOR DEMO RUNS //
//*********************//

//******TIMER T1 IS USED BY LINE TRACKING*********//


//blue run
void blueLine() {
	setLineVars();
	//track line until the obstacle
	while (SensorValue[sonic] > object_detection_range) {
		trackLine(blue_target);
	}

	//stop and beep twice

	stopDrive();
	playSound(soundBeepBeep);
	resetG();
	sleep(1000);

	//drive forward about 5cm
	drive_cm_forward(160);
	resetG();

	//turn left fast 60 degrees
	while (abs(SensorValue[gyro]) < blue_turn_value) {
		setDrive(50,-50);
	}
	stopDrive();
	resetG();

	sleep(1000);

	//turn back
	while (abs(SensorValue[gyro]) < blue_turn_value) {
		setDrive(-30,30);
	}
	stopDrive();
	resetG();

	sleep(1000);

	drive_cm_backward(160);
	resetG();

	sleep(1000);

	while (abs(SensorValue[gyro]) < 30) {
		setDrive(-20,20);
	}
	stopDrive();
	resetG();

	sleep(1000);

	//track the line, stop at the end
	while (abs(SensorValue[gyro]) < final_turn_value) {
		trackLine(blue_target);

		if (abs(error) < 2) {
			resetG();
		}
	}
	stopDrive();
}




//green run
void greenLine() {
	setLineVars();
	//track line until the obstacle
	while (SensorValue[sonic] > object_detection_range) {
		trackLine(green_target);
	}

	//stop and beep twice

	stopDrive();
	playSound(soundBeepBeep);
	resetG();
	sleep(1000);

	//turn 25 degrees to the right
	while (abs(SensorValue[gyro]) < green_turn_value) {
		setDrive(30, -30);
	}
	stopDrive();
	resetG();

	//track the line, stop at the end
	while (abs(SensorValue[gyro]) < final_turn_value) {
		trackLine(green_target);

		if (abs(error) < 2) {
			resetG();
		}
	}
	stopDrive();
}
