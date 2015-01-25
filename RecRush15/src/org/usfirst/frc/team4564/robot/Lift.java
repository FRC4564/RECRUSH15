package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
//import edu.wpi.first.wpilibj.Joystick;

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
    private int level = 0;
    private double speed = 0;
    private static final double FREE_MOVE_IPS = 10;
    private static final double LIFT_MAX = 70;
    private static final double LIFT_MIN = 4;   // Mininum inches from bottom of lift arm to floor.
    private static final double PLATFORM = 2;   // Height of the field scoring platform
    private static final double LIFT_CLEARANCE = PLATFORM + 1;
    private static final double ARM_HEIGHT = 5;
    private static final double TOTE_TOTAL = 12.1;
    private static final double TOTE_STACKED = 11.875;
    private static final double TOTE_LIP = 0.5;
    private static final double LIFT_LEVEL1 = TOTE_TOTAL - ARM_HEIGHT + LIFT_CLEARANCE;
    private static final double LIFT_LEVEL2 = TOTE_TOTAL + LIFT_LEVEL1;

    //private double liftLevel3 = liftLevel2 + toteStacked - toteLip;
    //PID constants
    private static final double Kp = 0.014;
    private static final double MAX_SPEED = 1.0;
	// encoder values
	private static final double COUNTS_PER_INCH = 10;
    private static final int TOLERANCE = 2;   //allowable tolerance between target and encoder
	// motor constants
	private static final boolean MOTOR_INVERT = false; // inverted means positive motor values move down
	private static final double MOTOR_INIT_SPEED = -0.25;  //speed to move lift when finding home position
	// limit switch constants
	private static final boolean UPPER_LIMIT_PRESSED = false;
	private static final boolean LOWER_LIMIT_PRESSED = false;

	
	public void init() {
		liftState = LIFT_INIT;
	}
	
	// Find home - which is at lowest position
	private void updateInit() {
		// Move to lower limit switch
		if (lowerLimit.get() != LOWER_LIMIT_PRESSED) {
			moveLift(MOTOR_INIT_SPEED);
		} else {
			// Set encoder count to 0, which is considered 'home'
			moveLift(0);
			encoder.reset();
			liftState = LIFT_IDLE;
		} 
	}

	
	// Return lift height in inches
	public double getHeight() {
		return encoder.get() / COUNTS_PER_INCH;	
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
	
	// Move the lift at the given speed, by setting a target relative to current position
	// Speed can be +1.0 to -1.0 where +1.0 is up at 100% of FREE_MOVE_IPS.
	// Update cycle is assumed to be 100 cycles per second.
	public void moveFree(double speed)  {
		double distance = FREE_MOVE_IPS * speed / 100;  //Inches to move in 1/100th of a second
		move(getHeight() + distance);  //Move relative to our current lift height
	}

	
	// Returns lift height in inches for the given level number
	private double levelCalc(int level){
		if (level == 1){
			return LIFT_LEVEL1;
		} else if (level == 2){
			return LIFT_LEVEL2;
		} else if (level > 2){
			return LIFT_LEVEL2 + TOTE_STACKED * level;
		}
		return 0;
	}
	
	// Move lift to a specifc level
	public void levelGo(int level){
		move(levelCalc(level));
	}
	
	// Move lift up one level
	public void levelUp(){
		level += 1;
		if (level > 6){
			level = 6;
		}
		move(levelCalc(level));
	}
	
	// Move lift down one level
	public void levelDown(){
		level -= 1;
		if (level < 0){
			level = 0;
		}
		move(levelCalc(level));
	}
	
	
	// Called every cycle, when in Move state
	private void updateMove() {
		if (Math.abs(encoder.get() - liftTarget) <= TOLERANCE) {
			liftState = LIFT_IDLE;
		} else {
			PIDMove();
		}
	}
	
	// Must call this update procedure at a cycle of 100 times per second.
	public void update() {
		
		if (liftState == LIFT_STOPPED) {
		} else if (liftState == LIFT_INIT) {
			updateInit();
		} else if (liftState == LIFT_IDLE) {
			moveLift(0);			
		} else if (liftState == LIFT_MOVING) {
			updateMove();
		}
		SmartDashboard.putNumber("encoder", encoder.get());
		SmartDashboard.putNumber("lift state", liftState);
		SmartDashboard.putNumber("lift target", liftTarget);
		SmartDashboard.putBoolean("Lower Limit", lowerLimit.get());
		SmartDashboard.putBoolean("Upper Limit", upperLimit.get());
		SmartDashboard.putNumber("encoder", encoder.get());	
		SmartDashboard.putNumber("Height", getHeight());
		
	}
	
	
	// Is lift idle - Idle means lift is initialized and ready for a move request.
	public boolean isIdle() {
		return liftState == LIFT_IDLE;
	}
	
	
	// Is lift moving? 
	public boolean isMoving() {
		return liftState == LIFT_IDLE;
	}

}