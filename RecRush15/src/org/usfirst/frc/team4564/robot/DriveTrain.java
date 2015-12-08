package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;


/**
 *
 * @author Bin, Jacket, Sieve
 */
public class DriveTrain extends RobotDrive {
    
	//Hdrive drive motors
	static Talon frontL = new Talon(Constants.PWM_DRIVE_FL);
    static Talon rearL = new Talon(Constants.PWM_DRIVE_RL);
    static Talon frontR = new Talon(Constants.PWM_DRIVE_FR);
    static Talon rearR = new Talon(Constants.PWM_DRIVE_RR);
    Talon frontC = new Talon(Constants.PWM_DRIVE_FC);
    Talon rearC = new Talon(Constants.PWM_DRIVE_RC);  
    
    //Hdrive acceleration curve speeds
    double driveSpeed = 0;
    double turnSpeed = 0;
    double slideSpeed = 0;

    // Encoder definitions
    private Encoder encoderFB = new Encoder(Constants.DIO_DRIVE_FB_ENCODER_A, Constants.DIO_DRIVE_FB_ENCODER_B, 
    		true, EncodingType.k1X);
    private Encoder encoderLR = new Encoder(Constants.DIO_DRIVE_LR_ENCODER_A, Constants.DIO_DRIVE_LR_ENCODER_B,
    		true, EncodingType.k1X);
    private static final double COUNTS_PER_INCH_FB = 38.4;
    private static final double COUNTS_PER_INCH_LR = 7.12359;
    
    // Forward/Backward, Left/Right and Turn PID Parameters
    private static final double Kp_FB = .08;
    private static final double Kp_LR = .03;
    private final static double Kp_TURN = 0.04;
    private static final double MIN_SPEED_FB = 0.55; 	//Min motor power
    private static final double MIN_SPEED_LR = 0.6; 	//Min motor power
    private static final double MIN_SPEED_TURN = 0.5;	//Min motor power was .55
    private static final double MAX_SPEED_FB = 0.90;	//Max motor power
    private static final double MAX_SPEED_LR = 0.90;	//Max motor power
    private static final double MAX_SPEED_TURN = 1.0;	//Max motor power was 2
    private static final double TOLERANCE_LR = 0.5;		// allowable tolerance between target and encoder in inches
    private static final double TOLERANCE_FB = 0.5; 	// allowable tolerance between target and encoder in inches
    private static final double TOLERANCE_TURN = 1; 	// allowable tolerance between target and encoder in degrees
    private static final int STATE_IDLE = 0;		// Both FB and LR PIDs are disabled.  Heading hold is still active.
    private static final int STATE_MOVING = 1;		// Movement in progress to reach target 
    private double moveTargetFB = 0; 	// Forward/Backward target distance relative to current encoder distance
    private double moveTargetLR = 0; 	// Left/Right target distance relative to current encoder distance
    private double targetHeading = 0; 	// Targeted heading
    // private double heading = 0; 	  	// Current heading
    private int moveStateFB = STATE_IDLE;        // State is set through movement commands and cleared by PID
    private int moveStateLR = STATE_IDLE;   	    // State is set through movement commands and cleared by PID
    private int turnState = STATE_IDLE; 			// State is set through movement commands and cleared by PID
    private double govenor = 1.0; 		// Overall governor of Max power to drive motors for PIDs.  1.0=100% or full power 0.5=50% 
    
    // Gyro-based heading control and PID
    public static Gyro gyro = new Gyro(Constants.ANA_HEADING);
    //0.00669
    private static final double GYRO_CALIBRATION = 0.00660; // Volts/degree/sec
    private double prevTurn = 0;       // Used to monitor hDrive Trun input to control auto heading hold

    //Drivetrain constructor
    public DriveTrain() {
        super(frontL, rearL, frontR, rearR);
        setInvertedMotor(RobotDrive.MotorType.kFrontLeft,true);
        setInvertedMotor(RobotDrive.MotorType.kRearLeft,true);
        setInvertedMotor(RobotDrive.MotorType.kFrontRight,true);
        setInvertedMotor(RobotDrive.MotorType.kRearRight,true);
        // Set encoder for distance measuring so the getDistance() can be used.
		encoderFB.setDistancePerPulse(1.0/COUNTS_PER_INCH_FB);  // Calibrate encoder for measurement in inches
		encoderLR.setDistancePerPulse(1.0/COUNTS_PER_INCH_LR);  // Calibrate encoder for measurement in inches
    }
    
    // Initialize drive train gyro.  Robot must be still.
    public void init() {
    	gyro.reset();
    	gyro.setSensitivity(GYRO_CALIBRATION);
    	targetHeading = 0;
    	moveTargetFB = 0;
    	moveTargetLR = 0;
    	encoderFB.reset();
    	encoderLR.reset();
        moveStateFB = STATE_IDLE;
        moveStateLR = STATE_IDLE;
        turnState = STATE_IDLE;
    }
    
    // Set speed based on Y value from left joystick and a straight line acceleration curve
    // accelCurve to driveAccelCurve
    public double driveAccelCurve(double target,double driveAccel) {
        if (Math.abs(driveSpeed - target) > driveAccel) {
            if (driveSpeed > target) {
                driveSpeed = driveSpeed - driveAccel;
            } else {
                driveSpeed = driveSpeed + driveAccel;
            }
        } else {
            driveSpeed = target;
        }
        return driveSpeed;
    }
    
    // X value from right joystick to slide
    public double slideAccelCurve(double target, double slideAccel) {
    	if (Math.abs(slideSpeed - target) > slideAccel) {
    	    if (slideSpeed > target) {
                slideSpeed = slideSpeed - slideAccel;
            } else {
                slideSpeed = slideSpeed + slideAccel;
            }
        } else {
            slideSpeed = target;
        }
        return slideSpeed;
    }
    
    // X value from left joystick to turn
    public double turnAccelCurve(double target, double turnAccel) {
    	if (Math.abs(turnSpeed - target) > turnAccel) {
    		if (turnSpeed > target) {
    			turnSpeed = turnSpeed - turnAccel;
    		} else {
    			turnSpeed = turnSpeed + turnAccel;
    		}
    	} else {
    		turnSpeed = target;
    	}
    return turnSpeed;
	}
     
    private double deadzone(double input) {
    	if (Math.abs(input) < .2) {
    		return (0);
    	} else {
    		return (input);
    	}
    }
    
    // Set drive motors, with scaling and mixing for slide motors.
    private void setDrive(double drive, double turn, double slide) {
    	slide = slide * 0.6;
    	turn = turn * 0.85;
    	arcadeDrive(drive, turn);
    	frontC.set(-slide - turn * 0.1);
    	rearC.set(slide - turn * 0.1);
		SmartDashboard.putNumber("encoderFB inches", encoderFB.getDistance() );
		SmartDashboard.putNumber("encoderFB", encoderFB.get() );
		SmartDashboard.putNumber("gyroHeading", gyro.getAngle());
		SmartDashboard.putNumber("GyroRate", gyro.getRate());
		SmartDashboard.putNumber("TurnPID", turn);
		
    }
    
    public void stop() {
    	turnState = STATE_IDLE;
    	moveStateFB = STATE_IDLE;
    	moveStateLR = STATE_IDLE;
    	targetHeading = gyro.getAngle();
    	encoderFB.reset();
    	encoderLR.reset();
    	moveTargetFB = 0;
    	moveTargetLR = 0;
    	
    }
    
    public void arcadeDriver(double drive, double turn) {
    	hDrive(drive, turn, 0);
    }
    
    public void translateDrive(double drive, double slide) {
    	hDrive(drive, 0, slide);
    }
   
    public void hDrive(double drive, double turn, double slide){
    	drive = deadzone(drive);
    	turn = deadzone(turn);
    	slide = deadzone(slide);
    	drive = driveAccelCurve(drive, 0.06);
    	turn = turnAccelCurve(turn, 0.1);
    	slide = slideAccelCurve(slide, 0.1);
    	// If any input is given, snap out of auto moveState.
    	if (drive != 0 || turn != 0 || slide != 0) {
    		moveStateFB = STATE_IDLE;
    	}
    	// Heading hold logic
    	if (turn == 0) {     		// Auto heading hold
    	/*	if (prevTurn != 0) {
    			targetHeading = gyro.getAngle();
    			prevTurn = 0;
    		}
    		turn = PIDTurn(); */
    	} else {	     			// Otherwise, keep track of heading as turn input is given
    		prevTurn = turn;
    	}
    	// Drive 
		setDrive(drive,turn,slide);
		//SmartDashboard.putNumber("encoderFB raw ", encoderFB.get() );
		//SmartDashboard.putNumber("moveTargetLR", moveTargetLR );
//		//SmartDashboard.putNumber("encoderLR inches", encoderLR.getDistance() );
		//SmartDashboard.putNumber("encoderLR", encoderLR.get());

    }
    	
	// Normalizes a heading to be within 0 to 360 degrees.
    private double normalize(double heading){
    	double a;
    	a = heading/360;
    	a = a - (int)a;
    	if (a > 0) {
    		return a * 360;
    	} else {
    		return (1 + a) * 360;
    	}
    }
        
    public void rotateTo(double heading) {
    	double diff;
    	diff = heading - normalize(gyro.getAngle()); 
        diff = normalize(diff);
        if (diff < 180) {
        	targetHeading = gyro.getAngle() + diff;
        } else {
    		targetHeading = gyro.getAngle() - (360 - diff);
    	}
    	turnState = STATE_MOVING;
    }
    
    // Turn robot number degrees based on current heading. Positive values turn clockwise.
    public void rotate(double degrees) {
    	targetHeading = gyro.getAngle() + degrees;
    	turnState = STATE_MOVING;
    }
    	
    public void rotateLeft90() {
    	targetHeading = Math.floor((gyro.getAngle()-2) / 90)*90;
    	turnState = STATE_MOVING;
    }
    	
    public void rotateRight90() {
    	targetHeading = Math.floor((gyro.getAngle()+92) / 90)*90;
    	turnState = STATE_MOVING;
    }
    
    // Calculate a Turn rate to reach Target heading based on current gyro heading
    private double PIDTurn() {
    	double error,P,turn;
    	double heading = gyro.getAngle();
		error = targetHeading - heading;
		// Is turn complete?
		if (Math.abs(error) < TOLERANCE_TURN) {
			turnState = STATE_IDLE; 
			turn = 0;
		} else {
			// Turn left or right, whichever is shortest path
			if (error > 180) {
				error = heading - (360 - targetHeading);
			}
			P = error * Kp_TURN;
			turn = Common.constrain(P, MIN_SPEED_TURN, MAX_SPEED_TURN) * govenor;
		}
		return turn; 
    }
    
    // Calculate PID Movement for Forward/Backward distance moves
    private double PIDMoveFB() {
	    double error = 0;
	    double P = 0;
	    double moveSpeed = 0;
	    // If Idle, then force target to actual
	    if (moveStateFB == STATE_IDLE) {
	    	moveTargetFB = encoderFB.getDistance();
	    }
		// Calculate PID
		error = moveTargetFB - encoderFB.getDistance();
		// Are we there yet?
		if (Math.abs(error) > TOLERANCE_FB) {
			P = error * Kp_FB;
			moveSpeed = Common.constrain(P,MIN_SPEED_FB,MAX_SPEED_FB * govenor);
		} else {
			moveStateFB = STATE_IDLE;
			moveSpeed = 0;
		}
		// SmartDashboard.putNumber("moveSpeedFB", -moveSpeed );
		return -moveSpeed;  //Invert required since negative values drive forward.
	}
    
    // Calcualte PID movement for Left/Right distance moves
    private double PIDMoveLR() {
	    double error = 0;
	    double P = 0;
	    double moveSpeed = 0;
	    // If Idle, then force target to actual
	    if (moveStateLR == STATE_IDLE) {
	    	moveTargetLR = encoderLR.getDistance();
	    }
		// Calculate PID
		error = moveTargetLR - encoderLR.getDistance();
		// Are we there yet?
		if (Math.abs(error) > TOLERANCE_LR) {
			P = error * Kp_LR;
			moveSpeed = Common.constrain(P,MIN_SPEED_LR,MAX_SPEED_LR * govenor);
		} else {
			moveStateLR = STATE_IDLE;
			moveSpeed = 0;
		}

		return moveSpeed;
	}
 
    public boolean isIdle(){
    	return (turnState == STATE_IDLE && moveStateFB == STATE_IDLE && moveStateLR == STATE_IDLE);
    }
    
    // Set governor speed 100 = full speed, 50 = half speed
    public void setSpeed(int percent) {
    		govenor = percent / 100.0;
    }
    
    public void moveForward(double inches) {
    	moveTargetFB = encoderFB.getDistance() + inches;
    	moveStateFB = STATE_MOVING;
    }
    
    public void moveBackward(double inches) {
    	moveTargetFB = encoderFB.getDistance() - inches;
    	moveStateFB = STATE_MOVING;
    }
    
    public void moveRight(double inches) {
    	moveTargetLR = encoderLR.getDistance() + inches;
    	moveStateLR = STATE_MOVING;
    }
    
    public void moveLeft(double inches) {
    	moveTargetLR = encoderLR.getDistance() - inches;
    	moveStateLR = STATE_MOVING;
    }

    // If using distance movement and rotate commands, call update every robot Refresh cycle
    // to allow PIDs to perform moves.
    public void update() {
    	double drive = driveAccelCurve(PIDMoveFB(), 0.1);
    	double turn = turnAccelCurve(PIDTurn(), 1.0);
    	double slide = slideAccelCurve(PIDMoveLR(), 0.1);
   		setDrive(drive,turn/*-0.32*/,slide);
   		//SmartDashboard.putNumber("targetHeading", targetHeading );
		//SmartDashboard.putNumber("current heading", gyro.getAngle() );
		//SmartDashboard.putNumber("moveTargetLR", moveTargetLR );
		//SmartDashboard.putNumber("encoderLR inches", encoderLR.getDistance() );
		//SmartDashboard.putNumber("encoderLR", encoderLR.get());
   		
    }
    
    // Reset gyro to 0 degrees.
    public void gyroReset() {
    	gyro.reset();
    }

}
