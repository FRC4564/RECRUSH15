package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 * @author Ben, Jacob, Steve
 */
public class DriveTrain extends RobotDrive {
    
    double speed = 0;
    double accel = 0.25;
    SpeedController frontC;
    SpeedController rearC;

    // Encoder definitions
    private Encoder encoderFB = new Encoder(Constants.DIO_DRIVE_FB_ENCODER_A, Constants.DIO_DRIVE_FB_ENCODER_B, 
    		false, EncodingType.k1X);
    private Encoder encoderLR = new Encoder(Constants.DIO_DRIVE_LR_ENCODER_A, Constants.DIO_DRIVE_LR_ENCODER_B,
    		false, EncodingType.k1X);
    private static final double COUNTS_PER_INCH_FB = 460 / 12.5663; // wheel circumference = 12.5663 /1 rev = 460
    private static final double COUNTS_PER_INCH_LR = 250 / 12.5663; //wheel circumference = 12.5663 / 1 rev = 250 counts 
    // PID Definitions
    private static final double Kp_FB = .005;
    private static final double Kp_LR = .1;
    private static final int MAX_SPEED_FB = 1;
    private static final int TOLERANCE_FB = 3; // allowable tolerance between target and encoder
    private static final int STATE_IDLE = 0;   // Both FB and LR PIDs are disabled.  Heading hold is still active.
    private static final int STATE_MOVING = 1; // PID control is active for FB and LR
    private int moveTargetFB = 0;  //
    private int moveStateFB = 0;  //Move state is managed by hDrive and set by rotate and move methods
    // Gyro-based heading control
    Gyro gyro = new Gyro(0);
    private double heading = 0;
    private double targetHeading = 0;
    private double prevTurn = 0;
    private double error = 0;
    private final static double Kp = 0.03;
    private double P = 0;
    
    
    public DriveTrain(SpeedController frontLeft, SpeedController rearLeft, SpeedController frontRight, SpeedController rearRight, 
    		          SpeedController frontCenter, SpeedController rearCenter) {
        super(frontLeft, rearLeft, frontRight, rearRight);
        setInvertedMotor(RobotDrive.MotorType.kFrontLeft,true);
        setInvertedMotor(RobotDrive.MotorType.kRearLeft,true);
        setInvertedMotor(RobotDrive.MotorType.kFrontRight,true);
        setInvertedMotor(RobotDrive.MotorType.kRearRight,false);
        frontC = frontCenter;
        rearC = rearCenter;
        // Set encoder for distance measuring so the getDistance() can be used.
		encoderFB.setDistancePerPulse(1.0/COUNTS_PER_INCH_FB);  // Calibrate encoder so that getRate() measures in inches/sec
		encoderLR.setDistancePerPulse(1.0/COUNTS_PER_INCH_LR);  // Calibrate encoder so that getRate() measures in inches/sec
    }
    
    // Set speed based on Y value from joystick and a straight line acceleration curve
    public double accelCurve(double Y) {
        if (Math.abs(speed - Y) > accel) {
            if (speed > Y) {
                speed = speed - accel;
            } else {
                speed = speed + accel;
            }
        } else {
            speed = Y;
        }
        return speed;
    }
    
    private double deadzone(double input) {
    	if (Math.abs(input) < .2) {
    		return (0);
    	} else {
    		return (input);
    	}
    }
    
    public void arcadeDriver(double drive, double turn) {
    	hDrive(drive, turn, 0);
    }
    
    public void translateDrive(double drive, double slide) {
    	hDrive(drive, 0, slide);
    }
    
    private void PIDRotate() {
    	
    }
    
    public void hDrive(double drive, double turn, double slide){
    	drive = deadzone(drive);
    	turn = deadzone(turn);
    	slide = deadzone(slide);
    	// if any input is given, snap out of Move state
    	if (drive != 0 || turn != 0 || slide != 0) {
    		moveStateFB = STATE_IDLE;
    	}
    	// Heading hold
    	double heading = gyro.getAngle();
    	SmartDashboard.putNumber("Gyro", gyro.getAngle());
    	SmartDashboard.putNumber("Target Heading", targetHeading);
    	SmartDashboard.putNumber("LR encoder", encoderLR.get());
    	SmartDashboard.putNumber("LR distance", encoderLR.getDistance());
    	SmartDashboard.putNumber("FB encoder", encoderFB.get());
    	SmartDashboard.putNumber("FB distance", encoderFB.getDistance());
    	if (turn == 0 ) {
    		if (prevTurn != 0) {
    			targetHeading = heading;
    			prevTurn = 0;
    		}
    		
    		
    		// Calculate PID
    		error = targetHeading - heading;
    		if (error > 180) {
    			error = heading - (360 - targetHeading);
    		}
    		
    		P = error * Kp;
    		turn = P;
    		if (turn > 1.0) {
    			turn = 1.0;
    		} else if (turn < -1.0) {
    			
    			turn = -1.0;
    		}
    		SmartDashboard.putNumber("P Turn", turn );
    	// Turn rate was given, don't hold heading
    	} else {
    		prevTurn = turn;
    	}
    	// Drive
    	arcadeDrive(drive, turn);
    	frontC.set(-slide - turn * 0.07);
    	rearC.set(slide - turn * 0.07);
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
    }
    	
    		
    public void rotateLeft90() {
    	targetHeading = Math.floor((gyro.getAngle()-2) / 90)*90;
    }
    	
    
    public void rotateRight90() {
    	targetHeading = Math.floor((gyro.getAngle()+92) / 90)*90;
    }
    
    // Initialize drive train components
    public void init()
    {
    	gyro.reset();
    	gyro.setSensitivity(.00669);
    	targetHeading = 0;
    }
    
    private void PIDMoveFB() {
	    double error = 0;
	    double P = 0;
	    double moveSpeed = 0;
		// Calculate PID
		error = moveTargetFB - encoderFB.get();
		// Are we there yet?
		if (Math.abs(error) > TOLERANCE_FB) {
			moveStateFB = STATE_MOVING;
			P = error * Kp_FB;
			moveSpeed = P;
			if (moveSpeed > MAX_SPEED_FB) {
				moveSpeed = MAX_SPEED_FB;
			} else if (moveSpeed < -MAX_SPEED_FB) {
				moveSpeed = -MAX_SPEED_FB;
			}
			hDrive(-moveSpeed, 0, 0);
    		SmartDashboard.putNumber("moveSpeed", moveSpeed );
    		SmartDashboard.putNumber("moveTarget", moveTargetFB );
    		SmartDashboard.putNumber("encoder", encoderFB.get() );

		} else {
			moveStateFB = STATE_IDLE;
			hDrive(0,0,0);
		}
	}
    
    public void moveForward(double inches) {
    	int distance = 0;
    	distance = (int) (inches * COUNTS_PER_INCH_FB);
    	moveTargetFB = encoderFB.get() + distance;
    	moveStateFB = STATE_MOVING;
    }
    
    public void updateMove() {
    		PIDMoveFB();
    	//	PIDMoveLR();
    }
    
    public void gyroReset() {
    	gyro.reset();
    }
}
