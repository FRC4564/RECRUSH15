package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

public class Lift {
	// define hardware
    private DigitalInput lowerLimit = new DigitalInput(Constants.DIO_LIFT_BOTTOM);
    private DigitalInput upperLimit = new DigitalInput(Constants.DIO_LIFT_TOP);
    private Victor liftMotor = new Victor(Constants.PWM_LIFT_MOTOR);
    private Encoder encoder = new Encoder(Constants.DIO_LIFT_ENCODER_A, Constants.DIO_LIFT_ENCODER_B,
    									  false, EncodingType.k4X);
	// encoder values
	private static final int COUNTS_PER_INCH = 100;
	private int targetPosition;  // Encoder count
	// motor constants
	private static final boolean MOTOR_INVERT = false; // inverted means positive motor values move down
	private static final double MOTOR_HOME_SPEED = -0.25;  //speed to move lift with finding home position
	// limit switch constants
	private static final boolean UPPER_LIMIT_PRESSED = true;
	private static final boolean LOWER_LIMIT_PRESSED = true;
	
	
	
	
	// Intialize lift
	public void init() {
		home();
	}
	
	// Find home - which is at lowest position
	public void home() {
		// Move to lower limit switch
		while (lowerLimit.get() != LOWER_LIMIT_PRESSED) {
			moveLift(MOTOR_HOME_SPEED);
			SmartDashboard.putNumber("encoder", encoder.get());
		}
		// Set encoder count to 0, which is considered 'home'
		encoder.reset();
	}
	
	
	// Move lift at specified speed.  Positive values move lift up.
	// Lift will not move beyond limit switches.
	private void moveLift(double motorSpeed) {
		// determine if we are moving upwards
		boolean directionUp = true;
		if (motorSpeed < 0) {
				directionUp = false;
			}
		// adjust direction of motorSpeed if it needs to be inverted
		if (MOTOR_INVERT) {
			motorSpeed = -motorSpeed;
		}
		// based on the direction, determine which limit switch to test
		// and move at the speed provided.
		if (directionUp) {
			if (upperLimit.get() == UPPER_LIMIT_PRESSED) {
				liftMotor.set(0);
			} else {
				liftMotor.set(motorSpeed);
			}
		} else {
			if (lowerLimit.get() == LOWER_LIMIT_PRESSED) {
				liftMotor.set(0);
			} else {
				liftMotor.set(motorSpeed);
			}
		}
	}
	
	public void update() {
		SmartDashboard.putNumber("encoder", encoder.get());
	}

}
