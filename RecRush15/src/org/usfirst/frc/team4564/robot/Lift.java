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
    
    //State Constants
    private static final int LIFT_STOPPED = 0;
	private static final int LIFT_INIT = 1;
	private static final int LIFT_IDLE = 2;
	private static final int LIFT_MOVING = 3;
    //Lift variables
    private int liftState = LIFT_STOPPED;
    private double liftSpeed = 0;
    private int liftTarget = 0;
    //PID constants
    private static final double Kp = 0.3;
    private static final double MAX_SPEED = 1.0;
	// encoder values
	private static final double COUNTS_PER_INCH = 10;
    private static final int TOLERANCE = 2;   //allowable tolerance between target and encoder
	// motor constants
	private static final boolean MOTOR_INVERT = false; // inverted means positive motor values move down
	private static final double MOTOR_INIT_SPEED = -0.25;  //speed to move lift when finding home position
	// limit switch constants
	private static final boolean UPPER_LIMIT_PRESSED = true;
	private static final boolean LOWER_LIMIT_PRESSED = false;

	
	public void init() {
		liftState = LIFT_INIT;
	}
	
	// Find home - which is at lowest position
	private void updateInit() {
		// Move to lower limit switch
		if (lowerLimit.get() != LOWER_LIMIT_PRESSED) {
			moveLift(MOTOR_INIT_SPEED);
			SmartDashboard.putNumber("encoder", encoder.get());
		} else {
			// Set encoder count to 0, which is considered 'home'
			moveLift(0);
			encoder.reset();
			liftState = LIFT_IDLE;
		} 
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
				liftSpeed = 0;
			} else {
				liftMotor.set(motorSpeed);
			}
		} else {
			if (lowerLimit.get() == LOWER_LIMIT_PRESSED) {
				liftMotor.set(0);
				liftSpeed = 0;
			} else {
				liftMotor.set(motorSpeed);
			}
		}
	}
	
	private void PIDMove() {
	    double error = 0;
	    double P = 0;
		// Calculate PID
		error = liftTarget - encoder.get();
		
		P = error * Kp;
		liftSpeed = P;
		if (liftSpeed > MAX_SPEED) {
			liftSpeed = MAX_SPEED;
		} else if (liftSpeed < -MAX_SPEED) {
			liftSpeed = -MAX_SPEED;
		}
		moveLift(liftSpeed);
	}
	
	public void move(double height) {
		if (liftState == LIFT_IDLE || liftState == LIFT_MOVING) {
			liftTarget = (int) (COUNTS_PER_INCH * height);
			liftState = LIFT_MOVING;
		}
	}
	
	private void updateMove() {
		if (Math.abs(encoder.get() - liftTarget) <= TOLERANCE) {
			liftState = LIFT_IDLE;
		} else {
			PIDMove();
		}
	}
	public void update() {
		
		if (liftState == LIFT_STOPPED) {
		} else if (liftState == LIFT_INIT) {
			updateInit();
		} else if (liftState == LIFT_IDLE) {
						
		} else if (liftState == LIFT_MOVING) {
			updateMove();
		}
		SmartDashboard.putNumber("encoder", encoder.get());
		SmartDashboard.putNumber("lift state", liftState);
		SmartDashboard.putNumber("lift target", liftTarget);
			
		
	}
	
	public boolean isIdle() {
		return liftState == LIFT_IDLE;
		
	}
}