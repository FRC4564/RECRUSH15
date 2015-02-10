package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
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
	Talon frontL = new Talon(Constants.PWM_DRIVE_FL);
    Talon rearL = new Talon(Constants.PWM_DRIVE_RL);
    Talon frontR = new Talon(Constants.PWM_DRIVE_FR);
    Talon rearR = new Talon(Constants.PWM_DRIVE_RR);
    Talon frontC = new Victor(Constants.PWM_DRIVE_FC);
    Talon rearC = new Victor(Constants.PWM_DRIVE_RC);  
    //Hdrive acceleration curve speeds
    double driveSpeed = 0;
    double turnSpeed = 0;
    double slideSpeed = 0;

    // Encoder definitions
    private Encoder encoderFB = new Encoder(Constants.DIO_DRIVE_FB_ENCODER_A, Constants.DIO_DRIVE_FB_ENCODER_B, 
    		false, EncodingType.k1X);
    private Encoder encoderLR = new Encoder(Constants.DIO_DRIVE_LR_ENCODER_A, Constants.DIO_DRIVE_LR_ENCODER_B,
    		false, EncodingType.k1X);
    private static final double COUNTS_PER_INCH_FB = 460 / 12.5663; // wheel circumference = 12.5663 /1 rev = 460
    private static final double COUNTS_PER_INCH_LR = 250 / 12.5663; //wheel circumference = 12.5663 / 1 rev = 250 counts 
    // Forward/Backward, Left/Right and Turn PID Parameters
    private static final double Kp_FB = .01;
    private static final double Kp_LR = .01;
    private final static double Kp_TURN = 0.03;
    private static final double MIN_SPEED_FB = 0.1; 	//Min motor power
    private static final double MIN_SPEED_LR = 0.1; 	//Min motor power
    private static final double MIN_SPEED_TURN = 0.1;	//Min motor power
    private static final double MAX_SPEED_FB = 1.0;		//Max motor power
    private static final double MAX_SPEED_LR = 1.0;		//Max motor power
    private static final double MAX_SPEED_TURN = 0.5;	//Max motor power
    private static final int TOLERANCE_LR = 0.5; // allowable tolerance between target and encoder in inches
    private static final int TOLERANCE_FB = 0.5; // allowable tolerance between target and encoder in inches
    private static final int TOLERANCE_TURN = 1; // allowable tolerance between target and encoder in degrees
    private static final int STATE_IDLE = 0;   // Both FB and LR PIDs are disabled.  Heading hold is still active.
    private static final int STATE_MOVING = 1; // Movement in progress to reach target 
    private double moveTargetFB = 0;  // Forward/Backward target distance relative to current encoder distance
    private double moveStateFB = 0;   // State is set through movement commands and cleared by PID
    private double moveTargetLR = 0;  // Left/Right target distance relative to current encoder distance
    private double moveStateLR = 0;   // State is set through movement commands and cleared by PID
    private double heading = 0; 		// Current heading
    private double targetHeading = 0; 	// Targeted heading
    private int turnState = 0; 			// State is set through movement commands and cleared by PID
    // Gyro-based heading control and PID
    Gyro gyro = new Gyro(0);
    private static final double GYRO_CALIBRATION = 0.00669; // Volts/sec/degree
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
    
    // Set drive motors, mixing turns into slide motors
    private void setDrive(double drive, double turn, double slide) {
    	arcadeDrive(drive, turn);
    	frontC.set(-slide - turn * 0.1);
    	rearC.set(slide - turn * 0.1);
    }
    
    public void arcadeDriver(double drive, double turn) {
    	hDrive(drive, turn, 0);
    }
    
    public void translateDrive(double drive, double slide) {
    	hDrive(drive, 0, slide);
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
			turn = Common.constrain(P, MIN_SPEED_TURN, MAX_SPEED_TURN); 
		}
		return turn; 
    }
    
    public void hDrive(double drive, double turn, double slide){
    	drive = deadzone(drive);
    	turn = deadzone(turn);
    	slide = deadzone(slide);
    	drive = driveAccelCurve(drive, 0.1);
    	turn = turnAccelCurve(turn, 0.1);
    	slide = slideAccelCurve(slide, 0.1);
    	// if any input is given, snap out of Move state
    	if (drive != 0 || turn != 0 || slide != 0) {
    		moveStateFB = STATE_IDLE;
    	}
    	// Heading hold logic
    	if (turn == 0) {     		// Auto heading hold
    		if (prevTurn != 0) {
    			targetHeading = gyro.getAngle();
    			prevTurn = 0;
    		}
    		turn = PIDTurn();
    	} else {	     			// Otherwise, keep track of heading as turn input is given
    		prevTurn = turn;
    	}
    	// Drive 
		setDrive(drive,turn,slide);
		
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putNumber("Target Heading", targetHeading);
    	SmartDashboard.putNumber("LR encoder", encoderLR.get());
    	SmartDashboard.putNumber("LR distance", encoderLR.getDistance());
    	SmartDashboard.putNumber("FB encoder", encoderFB.get());
    	SmartDashboard.putNumber("FB distance", encoderFB.getDistance());
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
    	
    public void rotateLeft90() {
    	targetHeading = Math.floor((gyro.getAngle()-2) / 90)*90;
    	turnState = STATE_MOVING;
    }
    	
    public void rotateRight90() {
    	targetHeading = Math.floor((gyro.getAngle()+92) / 90)*90;
    	turnState = STATE_MOVING;
    }
    
    // Calculate PID Movement for Forward/Backward distance moves
    private double PIDMoveFB() {
	    double error = 0;
	    double P = 0;
	    double moveSpeed = 0;
		// Calculate PID
		error = moveTargetFB - encoderFB.getDistance();
		// Are we there yet?
		if (Math.abs(error) > TOLERANCE_FB) {
			P = error * Kp_FB;
			moveSpeed = Common.constrain(P,MIN_SPEED_FB,MAX_SPEED_FB)
		} else {
			moveStateFB = STATE_IDLE;
			moveSpeed = 0;
		}
		SmartDashboard.putNumber("moveSpeedFB", moveSpeed );
		SmartDashboard.putNumber("moveTargetFB", moveTargetFB );
		SmartDashboard.putNumber("encoderFB inches", encoderFB.getDistance() );
		return -movespeed;  //Invert required since negative values drive forward.
	}
    
    // Calcualte PID movement for Left/Right distance moves
    private double PIDMoveLR() {
	    double error = 0;
	    double P = 0;
	    double moveSpeed = 0;
		// Calculate PID
		error = moveTargetLR - encoderLR.getDistance();
		// Are we there yet?
		if (Math.abs(error) > TOLERANCE_LR) {
			P = error * Kp_LR;
			moveSpeed = Common.constrain(P,MIN_SPEED_LR,MAX_SPEED_LR)
		} else {
			moveStateLR = STATE_IDLE;
			moveSpeed = 0;
		}
		SmartDashboard.putNumber("moveSpeedLR", moveSpeed );
		SmartDashboard.putNumber("moveTargetLR", moveTargetLR );
		SmartDashboard.putNumber("encoderLR inches", encoderLR.getDistance() );
		return moveSpeed
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
    
    public void moveRight(double inches) {
    	moveTargetLR = encoderLR.getDistance() - inches;
    	moveStateLR = STATE_MOVING;
    }

    // If using distnace movement and rotate commands, call updateMove every robot Refresh cycle
    // to allow PIDs to perform moves.
    public void updateMove() {
    		setDrive(PIDMoveFB(),PIDTurn(),PIDMoveLR());
    }
    
    public void gyroReset() {
    	gyro.reset();
    }
    
    public boolean isIdle(){
    	return (stateTurn == STATE_IDLE && stateMoveFB == STATE_IDLE && stateMoveLR == STATE_IDLE);
    }
}
