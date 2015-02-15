package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw {
	
	//Define Mast 
	Solenoid mastSol = new Solenoid(Constants.SOL_MAST);
	private static final boolean MAST_SOL_IN = true;  			//Solenoid setting to move mast in.
	private static final boolean MAST_SOL_OUT = ! MAST_SOL_IN;  //Solenoid setting to move mast out.
	private boolean mastPositionIn = true ;  					//Current mast position. True when mast is retracted.
	
	//Define Carriage 
	private Talon carriageMotor = new Talon(Constants.PWM_CARRIAGE_MOTOR);
	private DigitalInput carriageLimit = new DigitalInput(Constants.DIO_CARRIAGE_TOP);  //At top of mast
	private Encoder encoder = new Encoder(Constants.DIO_CARRIAGE_ENCODER_A, Constants.DIO_CARRIAGE_ENCODER_B,
				  true, EncodingType.k1X);  // Encoder should count positive values as carriage moves down
	private Solenoid brake = new Solenoid(Constants.SOL_BRAKE);
    Countdown idleTimer = new Countdown();  //Timer to wait after for PID to be in tolerance before setting to Idle.
    private static final boolean CARRIAGE_LIMIT_PRESSED = true;  //Value of limit switch when pressed.
	private static final double CARRIAGE_COUNTS_PER_INCH = 70.09174311;   //Encoder ticks per inch of carriage movement
	private static final double CARRIAGE_MAX = 54;
	
	// PID Carriage
	private static final double HEIGHT_Kp = 0.3;
    private static final double HEIGHT_Kd = 0;
    private static final double SPEED_Kp = 0.08;
    private static final double SPEED_Kd = 0.000;
    private static final double TOLERANCE = 0.25;  // Allowable error for PIDheight (Inches).
    private double targetPIDHeight = CARRIAGE_MAX;  //Carriage's home position.
	private double prevPIDHeightError = 0.0;
	private double prevPIDSpeedError = 0.0;
	private double prevPIDSpeedPower = 0.0;
	private static final double SPEED_MIN_IPS = 3.0;
	private static final double SPEED_MAX_IPS = 15.0;
	private double targetPIDSpeed = 0.0;
	private static final boolean MOTOR_INVERT = false;  // Inverted means positive motor values moves carriage down.
	private static final double MOTOR_MAX_POWER = 1.0; // Maximum power allowed for carriage motor.
	
	//Carriage States
	private final static int CARRIAGE_STOPPED = 0;
	private final static int CARRIAGE_INIT = 1;
	private final static int CARRIAGE_IDLE = 2;
	private final static int CARRIAGE_MOVING = 3;
	private final static int CARRIAGE_FREE = 4;
	private int carriageState = CARRIAGE_STOPPED;
	
	//Define Forebar
	private Solenoid forebarUpSol = new Solenoid(Constants.SOL_FOREBAR_UP);
	private Solenoid forebarDownSol = new Solenoid(Constants.SOL_FOREBAR_DOWN);
	private static final boolean FOREBAR_SOL_ENABLE = true;
	private static final boolean FOREBAR_SOL_DISABLE = ! FOREBAR_SOL_ENABLE;
	private boolean forebarPostionUp = true;
	private boolean forebarPostionStop = false;
	private static final double FOREBAR_HEIGHT_UP = 8;
	private static final double FOREBAR_HEIGHT_DOWN = 12;
	
	//Define Wrist 
	public static boolean VERTICAL_LIMIT_PRESSED = false; // This is the value of limit switch when pressed
	public static boolean HORIZONTAL_LIMIT_PRESSED = false; // ^
	public DigitalInput verticalLimit = new DigitalInput(Constants.DIO_VERTICAL_WRIST);
	public DigitalInput horizontalLimit = new DigitalInput(Constants.DIO_HORIZONTAL_WRIST);
    private Talon wristMotor = new Talon(Constants.PWM_WRIST_MOTOR);
    
    //Wrist States
    private static final int WRIST_IDLE = 0;
    private static final int WRIST_MOVING_HOR = 1;
    private static final int WRIST_MOVING_VER = 2;
    private int wristState = WRIST_IDLE;
    
	//Define Hand 
    Solenoid handSol = new Solenoid(Constants.SOL_HAND);
	private static final boolean HAND_SOL_OPEN = true;	//Solenoid value to open the hand.
	private static final boolean HAND_SOL_CLOSE = ! HAND_SOL_OPEN;  //Solenoid value to close the hand
	private boolean handPositionOpen = false ;  //Current hand position. True when hand is open.
	private static final double THUMB_HEIGHT = 9;
	private static final double FINGER_HEIGHT = 18;
	
	// Initiate carriage move to home position
	public void init() {
		carriageState = CARRIAGE_INIT;
		encoder.setDistancePerPulse(1.0/CARRIAGE_COUNTS_PER_INCH);  // Calibrate encoder to inches of travel
	}
	
	public void mastIn() {
	    mastSol.set(MAST_SOL_IN);
	    mastPositionIn = true;
	    	}	
	
	public void mastOut() {
		mastSol.set(MAST_SOL_OUT);
		mastPositionIn = false;
	}
	
	public void mastToggle() {
		if(mastPositionIn) {
			mastOut();
		} else {
			mastIn();
		}
	}
	
	public void forebarUp() {
		forebarUpSol.set(FOREBAR_SOL_ENABLE);
		forebarDownSol.set(FOREBAR_SOL_DISABLE);
		forebarPostionUp = true;
	}
	
	public void forbarDown() {
		forebarUpSol.set(FOREBAR_SOL_DISABLE);
		forebarDownSol.set(FOREBAR_SOL_ENABLE);
		forebarPostionUp = false;
	}

	
	public void forebarStop() {
		forebarDownSol.set(FOREBAR_SOL_DISABLE);
		forebarUpSol.set(FOREBAR_SOL_DISABLE);
	}
	
	// CARRIAGE
	
	// Calculate the height of the carriage along the mast. 
	// 0 = lowest point
	public double carriageHeight() {
		return CARRIAGE_MAX - encoder.getDistance();
	}
	
	public double safeMinHeight() {
		if (isVertical()) {
			return THUMB_HEIGHT;		
		} else {
			return 0;
		}
	}
	
	public double safeMaxHeight() { 
		if (isVertical()) {
			return (CARRIAGE_MAX - FOREBAR_HEIGHT_UP - FINGER_HEIGHT);		
		} else {
			return CARRIAGE_MAX;
		}
	}
	
	// Set the power of the carriage motor, but restricting its movement to stop at limit switch.
	// Positive power values move the lift up (set MOTOR_INVERT if motor must run backwards).
	
	private void setMotor(double power) {
		// based on the direction, determine how to test limits
		// and move at the power provided.
		if (power > 0) {		//moving up
			if (carriageLimit.get() == CARRIAGE_LIMIT_PRESSED) {
				power = 0;
			}
		}
		// Invert direction of motor, if necessary
		if (MOTOR_INVERT) {
			power = -power;
		}
		// Power the motor
		carriageMotor.set(power);
	}
		
	// Move carriage at target speed.  Brake is released/engaged based on speed.
	//Speed is constrained to Min/Max allowable.
	private void PIDSpeed() {
		double error, power, P, D;
		// Calculate PID
		if (carriageState != CARRIAGE_STOPPED) {
			if (targetPIDSpeed == 0) {
				power = 0;
				brake.set(false);	// Set brake.
			} else {
				targetPIDSpeed = Common.constrain(targetPIDSpeed, SPEED_MIN_IPS, SPEED_MAX_IPS) ;
				error = targetPIDSpeed - encoder.getRate();
				P = error * SPEED_Kp;
				D = (error - prevPIDSpeedError) / (1.0 / Constants.REFRESH_RATE) * SPEED_Kd;
				//prevPIDSpeedError = error;
				//
				power = P + D; //+ prevPIDSpeedPower;
				if (power > MOTOR_MAX_POWER) {
					power = MOTOR_MAX_POWER;
				} else if (power < -MOTOR_MAX_POWER) {
					power = -MOTOR_MAX_POWER;
				}
				brake.set(true); // Release brake.
			}
			setMotor(power);
			//prevPIDSpeedPower = power;
		}
	}
	
	//  Calculates speed to move carriage to reach target if moving, otherwise set speed to 0.
	//  Height is constrained to safe limits.
	private void PIDHeight() {
	    double error, speed, P, D;
	    if (carriageState == CARRIAGE_MOVING) { 
			// Calculate PID
	    	targetPIDHeight = Common.constrain(targetPIDHeight, safeMinHeight(), safeMaxHeight());
			error =  targetPIDHeight - carriageHeight();  // Targeted height from floor versus current carriage position  
			P = error * HEIGHT_Kp;
			D = (error - prevPIDHeightError) / (1.0 / Constants.REFRESH_RATE) * HEIGHT_Kd;
			prevPIDHeightError = error;
			speed = P + D;
	    } else {
			speed = 0;
		}
		targetPIDSpeed = speed;
	}

	// Set carriage to move to safe height from floor to have claw in vertical position
	//private void safeHeight() {
	//	if (HORIZONTAL_LIMIT_PRESSED = ! true && targetPIDHeight < SAFE_HEIGHT) {
	//		targetPIDHeight = SAFE_HEIGHT;
	//	}		
	//}
	
	// Move the carriage at the given speed, by setting a target relative to current position
	// Speed can be +1.0 to -1.0 where +1.0 is up at 100% of MAX_IPS.
	// Update cycle is typcially 100 cycles per second.
	public void moveFree(double speed)  {
		if (speed != 0 && carriageState != CARRIAGE_STOPPED) {
			if ((speed > 0 && carriageHeight() < safeMaxHeight() ) ||
				( speed < 0 && carriageHeight() > safeMinHeight()) ) {
				//double distance = SPEED_MAX_IPS * speed / Constants.REFRESH_RATE * 5;  //Inches to move within a refresh cycle
				//gotoHeight(getHeight() + distance);  //Move relative to current height
				targetPIDSpeed = SPEED_MAX_IPS * speed;
				carriageState = CARRIAGE_FREE;
			}
		}
	}
	
	// Is lift idle - Idle means lift is initialized and ready for a move request.
	public boolean isIdle() {
		return carriageState == CARRIAGE_IDLE;
	}
	
	// Is lift moving via PID for Free move? 
	public boolean isMoving() {
		return (carriageState == CARRIAGE_MOVING || carriageState == CARRIAGE_FREE);
	}
	
	// WRIST
	
	public boolean isHorizontal() {
		return true;
	}
	
	public boolean isVertical() {
		return false;
	}

	// Initiate moving the wrist to a horizontal position
	public void	horizontal() {
		wristState = WRIST_MOVING_HOR;  		
	}
	
	// Initiate moving the wrist to a vertical position
	public void	vertical() {
		wristState = WRIST_MOVING_VER;  		
	}

	//Update wrist movement based state.
	private void wristUpdate() {
		double power = 0;
		if (wristState == WRIST_MOVING_HOR) {
			// Power motor until limit switch is hit
			if (HORIZONTAL_LIMIT_PRESSED = true); {
				wristMotor.set(0);
			} if (HORIZONTAL_LIMIT_PRESSED = ! true); {
				wristMotor.set(MOTOR_MAX_POWER);
		} if (wristState == WRIST_MOVING_VER) {
			if (VERTICAL_LIMIT_PRESSED = true) {
				wristMotor.set(0);
			} if (VERTICAL_LIMIT_PRESSED = ! true); {
				wristMotor.set(-MOTOR_MAX_POWER);
			}
		}
	}
}
	
	public void handOpen() {
	    handSol.set(HAND_SOL_OPEN);
	    handPositionOpen = true;
	}
	
	public void handClose() {
	    handSol.set(HAND_SOL_CLOSE);
	    handPositionOpen = false;
	}
	
	public void handToggle() {
		if (handPositionOpen) {
			handClose();
		} else {
			handOpen();
		}
	}
	
	public void wristLeft() {   //rotate counterclockwise
		wristMotor.set(-.35);
		}
	
	public void wristRight() {
		wristMotor.set(.35);
	}
	
	public void wristStop() {
		wristMotor.set(0);
	}
	
	// Move carriage upward until limit switch and then set to Idle state
	private void updateInit() {
		if (carriageLimit.get() == CARRIAGE_LIMIT_PRESSED) {
			targetPIDSpeed = 0;
			carriageState = CARRIAGE_IDLE;
			encoder.reset();
		} else {
			targetPIDSpeed = 6;
		}
	}
	
	// Call update method every refresh cycle to update carriage and wrist movement
	public void update() {
		
		if (carriageState == CARRIAGE_INIT) {
			updateInit();
			PIDSpeed();
		} else if (carriageState == CARRIAGE_FREE) {
			PIDSpeed();
			carriageState = CARRIAGE_IDLE;
			targetPIDSpeed = 0;
			targetPIDHeight = carriageHeight();
		} else if (carriageState != CARRIAGE_STOPPED) {
			targetPIDHeight = Common.constrain(targetPIDHeight, safeMinHeight(), safeMaxHeight());
			if (Math.abs(carriageHeight() - targetPIDHeight) <= TOLERANCE) {
				if (idleTimer.done()) {
					carriageState = CARRIAGE_IDLE;
				} else {
					carriageState = CARRIAGE_MOVING;
				}
			} else {
				carriageState = CARRIAGE_MOVING;
				idleTimer.set(0.5);   // Reset timer for idle delay
			}
			PIDHeight();
			PIDSpeed();
		}
		wristUpdate();
		SmartDashboard.putNumber("Carriage Distance", encoder.getDistance());
		SmartDashboard.putNumber("Carriage Encoder", encoder.get());
		SmartDashboard.putBoolean("Carriage Limit?", carriageLimit.get() == CARRIAGE_LIMIT_PRESSED);
		SmartDashboard.putNumber("Carriage Height", carriageHeight());
		SmartDashboard.putNumber("Carriage tHeight", targetPIDHeight);
		SmartDashboard.putNumber("Carriage tSpeed", targetPIDSpeed);
		SmartDashboard.putNumber("Carriage State", carriageState);
		SmartDashboard.putNumber("Carriage Safe Min", safeMinHeight());
		
		//SmartDashboard.putBoolean("Wrist Limit Hor", horizontalLimit.get() == HORIZONTAL_LIMIT_PRESSED);
		//SmartDashboard.putBoolean("Wrist Limit Ver", verticalLimit.get() == VERTICAL_LIMIT_PRESSED);
		
	}
	
}
	