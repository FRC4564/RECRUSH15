package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw {
	
	//Define Mast 
	Solenoid mastSolIn = new Solenoid(Constants.SOL_MAST_IN);
	Solenoid mastSolOut = new Solenoid(Constants.SOL_MAST_OUT);
	private static final boolean MAST_SOL_IN = false;  			//Solenoid setting to move mast in.
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
	private static final double CARRIAGE_COUNTS_PER_INCH = 70.09174311;
	private static final double CARRIAGE_MAX = 54;
	
	// PID Carriage
	private static final double HEIGHT_Kp = 3.5;
    private static final double HEIGHT_Kd = 0;
    private static final double VEL_Kp = 0.02;
    private static final double VEL_Kd = 0.000;
    private static final double TOLERANCE = 0.35;  // Allowable error for PIDheight (Inches).
    private double targetPIDHeight = CARRIAGE_MAX;  //Carriage's home position.
	private double prevPIDHeightError = 0.0;
	private double prevPIDVelError = 0.0;
	private double prevPIDPower = 0.0;
	private double targetPIDVel = 0.0;
	private static final double VEL_MIN_IPS = 3.0;
	private static final double VEL_MAX_IPS = 10.0;
	private static final boolean MOTOR_INVERT = false;  // Inverted means positive motor values moves carriage down.
	private static final double MOTOR_MAX_POWER = 1.0; // Maximum power allowed for carriage motor.
	private static final double MOTOR_MIN_POWER = 0.2; // Minimum power for carriage motor.
	
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
    Solenoid handSolIn = new Solenoid(Constants.SOL_HAND_IN);
    Solenoid handSolOut = new Solenoid(Constants.SOL_HAND_OUT);
	private boolean handPositionOpen = false ;  //Current hand position. True when hand is open.
	private static final double THUMB_HEIGHT = 9;
	private static final double FINGER_HEIGHT = 18;
	
	// Initiate carriage move to home position
	public void init() {
		carriageState = CARRIAGE_INIT;
		targetPIDHeight = CARRIAGE_MAX;
		encoder.setDistancePerPulse(1.0/CARRIAGE_COUNTS_PER_INCH);  // Calibrate encoder to inches of travel
	}
	
	public void mastIn() {
	    mastSolIn.set(true);
	    mastSolOut.set(false);
	    mastPositionIn = true;
	    	}	
	
	public void mastOut() {
		mastSolIn.set(false);
		mastSolOut.set(true);
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
	
	public void forebarDown() {
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

	
	// Force motor stop, used to end autonomous
	public void stop() { 
		setMotor(0);
	}
		
	// Move carriage at target velocity.  Brake is released/engaged based on velocity.
	//Speed is constrained to Min/Max allowable.
	private void PIDVelocity() {
		double error, power, P, D;
		// Calculate PID
		if (carriageState != CARRIAGE_STOPPED) {
			if (targetPIDVel == 0) {
				power = 0;
				brake.set(false);	// Set brake.
			} else {
				targetPIDVel = Common.constrain(targetPIDVel, VEL_MIN_IPS, VEL_MAX_IPS) ;
				error = targetPIDVel - -encoder.getRate();
				P = error * VEL_Kp;
				//D = (error - prevPIDVelError) / (1.0 / Constants.REFRESH_RATE) * VEL_Kd;
				//prevPIDVelError = error;
				D = 0;
				//
				power = P + D + prevPIDPower;
				// Contstrain within Min/Max allowed
				power = Common.constrain(power, MOTOR_MIN_POWER, MOTOR_MAX_POWER);				
				brake.set(true); // Release brake.
			}
			setMotor(power);
			prevPIDPower = power;
		}
	}
	
	//  Calculates velocity to move carriage to reach target if moving, otherwise set velocity to 0.
	//  Height is constrained to safe limits.
	private void PIDHeight() {
	    double error, velocity, P, D;
	    if (carriageState == CARRIAGE_MOVING) { 
			// Calculate PID
	    	targetPIDHeight = Common.constrain(targetPIDHeight, safeMinHeight(), safeMaxHeight());
			error =  targetPIDHeight - carriageHeight();  // Targeted height from floor versus current carriage position  
			P = error * HEIGHT_Kp;
			//D = (error - prevPIDHeightError) / (1.0 / Constants.REFRESH_RATE) * HEIGHT_Kd;
			D=0;
			prevPIDHeightError = error;
			velocity = P + D;
	    } else {
			velocity = 0;
		}
		targetPIDVel = velocity;
	}

	// Set carriage to move to safe height from floor to have claw in vertical position
	//private void safeHeight() {
	//	if (HORIZONTAL_LIMIT_PRESSED = ! true && targetPIDHeight < SAFE_HEIGHT) {
	//		targetPIDHeight = SAFE_HEIGHT;
	//	}		
	//}
	
	// Move the carriage at the given velocity, for free joysick control
	// Velocity can be +1.0 to -1.0 where +1.0 is up at 100% of MAX_IPS.
	// Update cycle is typcially 50 cycles per second.
	public void moveFree(double velocity)  {
		if (carriageState != CARRIAGE_STOPPED && carriageState != CARRIAGE_INIT) {
			if ((velocity > 0 && carriageHeight() < safeMaxHeight() ) ||
				( velocity < 0 && carriageHeight() > safeMinHeight()) ) {
				targetPIDVel = VEL_MAX_IPS * velocity;
				carriageState = CARRIAGE_FREE; 
			} else {
				// Exit FREE state when velocity = 0 or limits reached. Force current height to the target height.
				if (carriageState == CARRIAGE_FREE) {
					carriageState = CARRIAGE_IDLE;
					targetPIDHeight = carriageHeight();
				}
			}
		}
	}
	
	// Move carriage to a specific height (0 at bottom)
	public void carriageTo(double inches) {
		targetPIDHeight = Common.constrain(inches, safeMinHeight(), safeMaxHeight());
		carriageState = CARRIAGE_MOVING;
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
	}
	
	public void handOpen() {
	    handSolIn.set(true);
	    handSolOut.set(false);
	    handPositionOpen = true;
	}
	
	public void handClose() {
		handSolIn.set(false);
	    handSolOut.set(true);
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
			targetPIDVel = 0;
			carriageState = CARRIAGE_IDLE;
			encoder.reset();
		} else {
			targetPIDVel = 6;
		}
		PIDVelocity();
	}
	
	
	// Continue moving carriage until it has reached target for at fraction of a second
	private void updateMove() {
		if (Math.abs(carriageHeight() - targetPIDHeight) <= TOLERANCE) {
			// Reached target, but it needs to stay within bounds until timer finishes
			if (idleTimer.done()) {
				carriageState = CARRIAGE_IDLE;
			}
		} else {
			carriageState = CARRIAGE_MOVING;
			idleTimer.set(0.25);   // Reset timer for idle delay
		}
		PIDHeight();
		PIDVelocity();
	}
	
	
	// Call update method every refresh cycle to update carriage and wrist movement
	public void update() {
		if (carriageState == CARRIAGE_INIT) {
			updateInit();
		} else if (carriageState == CARRIAGE_FREE) {
			PIDVelocity();
			targetPIDHeight = carriageHeight();
		} else if (carriageState != CARRIAGE_STOPPED) {
			updateMove();
		}

		SmartDashboard.putNumber("Carriage Encoder Height", carriageHeight());
		SmartDashboard.putNumber("Carriage Target Height", targetPIDHeight);
		SmartDashboard.putNumber("Carriage Encoder Vel", encoder.getRate());
		SmartDashboard.putNumber("Carriage Target Vel", targetPIDVel);
		SmartDashboard.putNumber("Carriage PID Power", prevPIDPower);
		//SmartDashboard.putNumber("Carriage Encoder Count", encoder.get());
		
		
	}
	
}
	