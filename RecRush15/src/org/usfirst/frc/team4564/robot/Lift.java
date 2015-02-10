package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
//import edu.wpi.first.wpilibj.Joystick;

public class Lift {

    //State Constants
    private static final int LIFT_STOPPED = 0;  //Not ready...needs to be initialized
	private static final int LIFT_INIT = 1;     //Initialization in process
	private static final int LIFT_IDLE = 2;     //Not moving, initialized and ready to move
	private static final int LIFT_MOVING = 3;   //Moving
	//Physical dimensions
    private static final double LIFT_MIN_HEIGHT = 10.3;   // Inches from top of lift arm to floor when lift is at its lower limit.
    private static final double LIFT_MAX = 70;  // Inches from bottom of lift arm to floor when lift is at its upper limit.
    private static final double PLATFORM = 2;   // Height of the field scoring platform
    private static final double TOTE_HEIGHT = 12.1; // Dimensional height of a tote
    private static final double TOTE_STACKED = 11.875;  // When interlocked, additional height of a stacked tote
    private static final double TOTE_LIP = 1.0;    // Dimensional height of the totes lip (ie. upper edge)
    private static final double ARM_HEIGHT = 6;	// Dimensional height of the arm
    //Reference measurements
    private static final double LIFT_CLEARANCE = PLATFORM + 1;  // Desired clearance between bottom of tote and floor (Lift Level 1)
    private static final double RELEASE_CLEARANCE = 4; // Distance to move down from top of tote to release it 
    private static final double LIFT_LEVEL0 = TOTE_HEIGHT - TOTE_LIP; // Drag totes at ground level
    private static final double LIFT_LEVEL1 = TOTE_HEIGHT + LIFT_CLEARANCE;  // Lift height for carrying a tote with clearance
    private static final double LIFT_LEVEL2 = TOTE_HEIGHT + LIFT_LEVEL1;  // Lift height to get above one tote with clearance
    private static final int MAX_LEVEL = 6; // Highest level lift can reach.

    //PID constants and variables
    private static final double VEL_Kp = 0.002;
    private static final double VEL_Kd = 0.000;
    private static final double MOTOR_MAX_POWER = 1.0;  //Maximum allowed motor power (constrained by Velocity PID) 
    private static final double HEIGHT_Kp = 3.0;
    private static final double HEIGHT_Kd = 0.1;
    private static final double VEL_MIN_IPS = 2.0;
    private static final double VEL_MAX_IPS = 15.0;     //Fastest allowed movement of lift (constrained by Height PID).
    private double targetPIDVelocity = 0.0;	            //Target Velocity, controlled via PID
    private double prevPIDVelocityError = 0.0;          //Used for PID derivative
    private double prevPIDVelocityPower = 0.0;          //Motor power is adjusted by PID
    private double targetPIDHeight = LIFT_LEVEL0;       //Target Height (inches from floor to bottom of arm), controlled via PID
    private double prevPIDHeightError = 0.0;            //Used for PID derivative
    // Encoder, Motor, Limit switch constants
	private static final double COUNTS_PER_INCH = 56;  //encoder counts per inch of lift movement
    private static final double TOLERANCE = 0.25;        //allowable inch tolerance between target and encoder for positional alignment
	private static final boolean MOTOR_INVERT = true;  // inverted means positive motor values move down
	private static final double MOTOR_INIT_SPEED = -0.25;  //speed to move lift when finding home position
	private static final boolean LOWER_LIMIT_PRESSED = false;  //Value of limit switch when presse
	// define hardware
    private DigitalInput lowerLimit = new DigitalInput(Constants.DIO_LIFT_BOTTOM);
    private Talon liftMotor = new Talon(Constants.PWM_LIFT_MOTOR);
    private Encoder encoder = new Encoder(Constants.DIO_LIFT_ENCODER_A, Constants.DIO_LIFT_ENCODER_B,
    									  true, EncodingType.k4X);
    // Lift variables
    private int liftState = LIFT_STOPPED;          //State determines what action is pending and update() manages it.
    private int targetLevel = 0;	               //Level targeted for move.  Used by LevelUp and LevelDown
    
	public void init() {
		liftState = LIFT_INIT;
		encoder.setDistancePerPulse(1.0/COUNTS_PER_INCH);  // Calibrate encoder so that getRate() measures in inches/sec
		Common.debug("Starting: Lift init");
	}

	// Move lift at specified power level.  Positive values move lift up.
	// Power will not be applied if a limit switch is hit.
	private void setMotor(double power) {

		//Don't go lower than limit switch
		if (power < 0) {
			if (lowerLimit.get() == LOWER_LIMIT_PRESSED) {
				power = 0;
			}
		}
		// adjust direction of motorSpeed if it needs to be inverted
		if (MOTOR_INVERT) {
			power = -power;
		}
		// Move at the power provided.
		liftMotor.set(power);
	}
	
	// Return lift height in inches above floor as measured from top of the lifting point
	public double getHeight() {
		return encoder.getDistance() + LIFT_MIN_HEIGHT;	
	}

	// Governs the Velocity of the lift using a PID and encoder getRate().
	// Follows the value of targetPIDVelocity, which is in inches per second.
	// Controls motor power.
	private void PIDVelocity() {
		double error, power, P, D;
		// Calculate PID
		error = targetPIDVelocity - encoder.getRate();
		P = error * VEL_Kp;
		D = (error - prevPIDVelocityError) / (1.0 / Constants.REFRESH_RATE) * VEL_Kd;
		prevPIDVelocityError = error;
		//
		power = P+D+prevPIDVelocityPower;
		power = Common.constrain(power, 0, MOTOR_MAX_POWER);
		setMotor(power);
		prevPIDVelocityPower = power;
	}
	
	// Governs the position of the lift using a PID and encoder getDistance().
	// Follows the value of targetPIDHeight, which is inches from the floor to bottom of lift arm
	private void PIDHeight() {
	    double error, velocity, P, D;
		// Calculate PID
		error = targetPIDHeight - getHeight();  // Height from floor targeted and current encoder  
		P = error * HEIGHT_Kp;
		D = (error - prevPIDHeightError) / (1.0 / Constants.REFRESH_RATE) * HEIGHT_Kd;
		prevPIDHeightError = error;
		
		velocity = P + D;
		if (liftState == LIFT_MOVING) {
			velocity = Common.constrain(velocity, VEL_MIN_IPS, VEL_MAX_IPS);
		} else {
			velocity = 0;
		}
		
		targetPIDVelocity = velocity;
	}
	
	// Returns lift Level given a height in inches
	// Rounds up to nearest level.  Use if lift is moving upward
	private int upwardLevelCalc(double inches) {
		if (inches <= LIFT_LEVEL0) {
			return 0;
		} else if (inches <= LIFT_LEVEL1) {
			return 1;
		} else if (inches <= LIFT_LEVEL2) {
			return 2;
		} else {
			return (int) ((inches - LIFT_LEVEL2) / TOTE_STACKED) + 2;
		}
	}
	
	// Returns lift Level given a height in inches
	// Rounds down to nearest level.  Use if lift is moving upward
		
	private int downwardLevelCalc(double inches) {
		if (inches <= LIFT_LEVEL1 - RELEASE_CLEARANCE) {
			return 0;
		} else if (inches <= LIFT_LEVEL2 - RELEASE_CLEARANCE) {
				return 1;
		} else {
			return (int) ((inches - LIFT_LEVEL2 + RELEASE_CLEARANCE) / TOTE_STACKED) + 2;
		}
	}
		
	// Returns lift height in inches for the given level number
	private double levelToInches(int level) {
		// Determine height of level
		if (level == 1) {
			return LIFT_LEVEL1;
		} else if (level == 2) {
			return LIFT_LEVEL2;
		} else if (level > 2) {
			return LIFT_LEVEL2 + TOTE_STACKED * (level - 2);
		}
		return LIFT_LEVEL0;
	}
	
	// Request lift to move to height inches, as measured from floor to bottom of lift arm.
	public void gotoHeight(double inches) {
		if (liftState == LIFT_IDLE || liftState == LIFT_MOVING) {
			targetPIDHeight = inches;
			liftState = LIFT_MOVING;
		}
	}

	// Move the lift at the given speed, by setting a target relative to current position
	// Speed can be +1.0 to -1.0 where +1.0 is up at 100% of VEL_MAX_IPS.
	// Update cycle is typcially 100 cycles per second.
	public void moveFree(double speed)  {
		double distance = VEL_MAX_IPS * speed / Constants.REFRESH_RATE;  //Inches to move within a refresh cycle
		gotoHeight(getHeight() + distance);  //Move relative to current height
	}
	
	// Move lift to a specific level
	public void gotoLevel(int newLevel){
		targetLevel = newLevel;
		if (targetLevel >= MAX_LEVEL) {
			targetLevel = MAX_LEVEL;
		} else if (targetLevel < 0) {
			targetLevel = 0;
		}
		Common.debug("Target level = " + targetLevel);
		gotoHeight(levelToInches(targetLevel));
	}
	
	// Move lift up one level
	public void levelUp(){
		Common.debug("Level up");
		gotoLevel(targetLevel + 1);
	}
	
	// Move lift down one level
	public void levelDown(){
		Common.debug("Level down");
		gotoLevel(targetLevel - 1);
	}
		
	// Move down below nearest level to release the tote.
	public void releaseTote() {
		double releasePoint;
		releasePoint = levelToInches(downwardLevelCalc(getHeight())) - RELEASE_CLEARANCE;
		gotoHeight(releasePoint);
	}
	
	// Find home by moving down slowly until limit switch is hit
	private void updateInit() {
		// Move to lower limit switch
		if (lowerLimit.get() != LOWER_LIMIT_PRESSED) {
			setMotor(MOTOR_INIT_SPEED);
		} else {
			// Stop motor and set encoder count to 0, which is considered 'home'
			setMotor(0);
			encoder.reset();
			liftState = LIFT_IDLE;
			Common.debug("Ending: Lift init");
		} 
	}

	// Called every cycle, when in Move state
	private void updateMove() {
		if (Math.abs(getHeight() - targetPIDHeight) <= TOLERANCE) {
			liftState = LIFT_IDLE;
			//targetPIDHeight = getHeight();
//			setBrake();
		} else {
			liftState = LIFT_MOVING;
//			releaseBrake();
		}
		PIDHeight();
		PIDVelocity();
	}
	
	// Must call this update procedure at a cycle of 100 times per second.
	public void update() {
		if (liftState == LIFT_STOPPED) {
		} else if (liftState == LIFT_INIT) {
			updateInit();
		} else {
			updateMove();
		}
		SmartDashboard.putNumber("Lift state", liftState);
		SmartDashboard.putBoolean("Lower Limit", lowerLimit.get() == LOWER_LIMIT_PRESSED);
		SmartDashboard.putNumber("Lift encoder inches",encoder.getDistance());
		SmartDashboard.putNumber("Calculated height", getHeight());
		SmartDashboard.putNumber("Target height", targetPIDHeight);
		SmartDashboard.putNumber("Target velocity", targetPIDVelocity);		
		SmartDashboard.putNumber("Target Level", targetLevel);		

		
		
	}
	// Is lift idle - Idle means lift is initialized and ready for a move request.
	public boolean isIdle() {
		return liftState == LIFT_IDLE;
	}
	
	// Is lift moving? 
	public boolean isMoving() {
		return liftState == LIFT_MOVING;
	}
}
