package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 * @author Ben
 */
public class DriveTrain extends RobotDrive {
    
    double speed = 0;
    double accel = 0.25;
    boolean init = true;
    SpeedController frontC;
    SpeedController rearC;
    
    // gyro-based heading control
    Gyro gyro = new Gyro(0);
    private double heading = 0;
    private double targetHeading = 0;
    private double prevTurn = 0;
    private double error = 0;
    private final static double Kp = 0.3;
    private double P = 0;
    
    
    public DriveTrain(SpeedController frontLeft, SpeedController rearLeft, SpeedController frontRight, SpeedController rearRight, SpeedController frontCenter, SpeedController rearCenter) {
        super(frontLeft, rearLeft, frontRight, rearRight);
        setInvertedMotor(RobotDrive.MotorType.kFrontLeft,true);
        setInvertedMotor(RobotDrive.MotorType.kRearLeft,true);
        setInvertedMotor(RobotDrive.MotorType.kFrontRight,false);
        setInvertedMotor(RobotDrive.MotorType.kRearRight,true);
        frontC = frontCenter;
        rearC = rearCenter;
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
    	if (Math.abs(input) < .05) {
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
    
    public void hDrive(double drive, double turn, double slide){
    	turn = deadzone(turn);
    	heading = gyro.getAngle();
    	SmartDashboard.putNumber("Gyro", Math.abs(gyro.getAngle()));
    	SmartDashboard.putNumber("Target Heading", targetHeading);
    	// Heading hold
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
    	frontC.set(-slide);
    	rearC.set(slide - turn * 0.5);
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
    	
    		
    public void rotateLeft() {
    	targetHeading = Math.floor((gyro.getAngle()-88) / 90)*90;
    }
    	
    
    public void rotateRight() {
    	targetHeading = Math.floor((gyro.getAngle()+90+2) / 90)*90;
    }
    
    
    
    // Initialize drive train components
    public void init()
    {
    	gyro.reset();
    	gyro.setSensitivity(.00669);
    	targetHeading = 0;
    }
}

