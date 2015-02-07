package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw {
	
	//Mast States
	//Define Mast 
	Solenoid mastSol = new Solenoid(Constants.SOL_MAST);
	private static final boolean MAST_SOL_IN = true;  			//Solenoid setting to move mast in.
	private static final boolean MAST_SOL_OUT = ! MAST_SOL_IN;  //Solenoid setting to move mast out.
	private boolean mastPositionIn = true ;  					//Current mast position. True when mast is retracted.
	//Define Carriage 
	private Talon carriageMotor = new Talon(Constants.PWM_CARRIAGE_MOTOR);
	private DigitalInput carriageLimit = new DigitalInput(Constants.DIO_CARRIAGE_TOP);  //At top of mast
	private Encoder encoder = new Encoder(Constants.DIO_CARRIAGE_ENCODER_A, Constants.DIO_CARRIAGE_ENCODER_B,
				  true, EncodingType.k4X);  // Encoder should count positive values as carriage moves down
	private static final boolean CARRIAGE_LIMIT_PRESSED = false;  //Value of limit switch when pressed.
	private static final double CARRIAGE_LOW = 5;    //Inches carriage is above floor when at its lowest point
	private static final double CARRIAGE_HIGH = 65;  //Inches carriage is above floor when at its highest point
	private static final double CARRIAGE_COUNTS_PER_INCH = 25;   //Encoder ticks per inch of carriage movement
	// PID Carriage
	private static final double HEIGHT_Kp = 3.0;
    private static final double HEIGHT_Kd = 0.1;
    private static final double SPEED_Kp = 0.002;
    private static final double SPEED_Kd = 0.000;
    private double targetPIDHeight = CARRIAGE_HIGH;  //Carriage's home position
	private double prevPIDHeightError = 0.0;
	private double prevPIDSpeedError = 0.0;
	private double prevPIDSpeedPower = 0.0;
	private static final double SPEED_MAX_IPS = 15.0;
	private double targetPIDSpeed = 0.0;
	private static final boolean MOTOR_INVERT = true;  // Inverted means positive motor values moves carriage down
	private static final double MOTOR_MAX_POWER = 1.0; // Maximum power allowed for carriage motor
	private static final double SAFE_HEIGHT = 4.0;
	
	//Carriage States
	private int carriageState = CARRIAGE_IDLE;
	private final static int CARRIAGE_INIT = 0;
	private final static int CARRIAGE_IDLE = 1;
	private final static int CARRIAGE_MOVING = 2;

	//Define Forebar
	Solenoid forebarUpSol = new Solenoid(Constants.SOL_FOREBAR_UP);
	Solenoid forebarDownSol = new Solenoid(Constants.SOL_FOREBAR_DOWN);
	private static final boolean FOREBAR_SOL_ENABLE = true;
	private static final boolean FOREBAR_SOL_DISABLE = ! FOREBAR_SOL_ENABLE;
	private boolean forebarPostionUp = true;
	private boolean forebarPostionStop = false;
	
	//Define Wrist 
	private static boolean VERTICAL_LIMIT_PRESSED = false; // This is the value of limit switch when pressed
	private static boolean HORIZONTAL_LIMIT_PRESSED = false; // ^
	private DigitalInput verticalLimit = new DigitalInput(Constants.DIO_VERTICAL_WRIST);
	private DigitalInput horizontalLimit = new DigitalInput(Constants.DIO_HORIZONTAL_WRIST);
    private Talon wristMotor = new Talon(Constants.PWM_WRIST_MOTOR);    
    //Wrist States
    private static final int WRIST_IDLE = 0;
    private static final int WRIST_MOVING_HOR = 1;
    private static final int WRIST_MOVING_VER = 2;
    private int wristState = WRIST_IDLE;
    
	//Define Hand (the claw) 
    Solenoid handSol = new Solenoid(Constants.SOL_HAND);
	private static final boolean HAND_SOL_OPEN = true;			//Solenoid value to open the hand
	private static final boolean HAND_SOL_CLOSE = ! HAND_SOL_OPEN;  //Solenoid value to close the hand
	
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
		forebarDownSol.set(FOREBAR_SOL_DISABLE);
		forebarUpSol.set(FOREBAR_SOL_ENABLE);
		forebarPostionUp = false;
	}
	
	public void forebarStop() {
		forebarDownSol.set(FOREBAR_SOL_DISABLE);
		forebarUpSol.set(FOREBAR_SOL_DISABLE);
	}
	
	// CARRIAGE
	
	// Calculate the height of the carriage from the floor.
	public double carriageHeight() {
		return CARRIAGE_HIGH - encoder.getDistance();
	}
	
	// Set the power of the carriage motor, but restricting its movement to within safe limits.
	// Positive power values move the lift up (set MOTOR_INVERT if motor must run backwards).
	// Uses limit switch on top of mast and the encoder to measure distance from floor
	// to stay within safe limits.
	
	private void setMotor(double power) {
		// based on the direction, determine how to test limits
		// and move at the power provided.
		if (power > 0) {		//moving up
			if (carriageLimit.get() == CARRIAGE_LIMIT_PRESSED) {
				power = 0;
			} else {			//moving down
				if (carriageHeight() <= CARRIAGE_LOW) {
					power = 0;
				}
			}
		}
		// Invert direction of motor, if necessary
		if (MOTOR_INVERT) {
			power = -power;
		}
		// Power the motor
		carriageMotor.set(power);
	}
		
	private void PIDSpeed() {
		double error, power, P, D;
		// Calculate PID
		error = targetPIDSpeed - encoder.getRate();
		P = error * SPEED_Kp;
		D = (error - prevPIDSpeedError) / (1.0 / Constants.REFRESH_RATE) * SPEED_Kd;
		prevPIDSpeedError = error;
		//
		power = P + D + prevPIDSpeedPower;
		if (power > MOTOR_MAX_POWER) {
			power = MOTOR_MAX_POWER;
		} else if (power < -MOTOR_MAX_POWER) {
			power = -MOTOR_MAX_POWER;
		}
		setMotor(power);
		prevPIDSpeedPower = power;
	}
	
	private void PIDHeight() {
	    double error, speed, P, D;
		// Calculate PID
		error = targetPIDHeight - carriageHeight();  // Targeted height from floor versus current carriage position  
		P = error * HEIGHT_Kp;
		D = (error - prevPIDHeightError) / (1.0 / Constants.REFRESH_RATE) * HEIGHT_Kd;
		prevPIDHeightError = error;
		
		speed = P + D;
		if (speed > SPEED_MAX_IPS) {
			speed = SPEED_MAX_IPS;
		} else if (speed < -SPEED_MAX_IPS) {
			speed = -SPEED_MAX_IPS;
		}
		targetPIDSpeed = speed;
	}

	private void safeHeight() {
		if (HORIZONTAL_LIMIT_PRESSED = ! true && targetPIDHeight < SAFE_HEIGHT) {
			targetPIDHeight = SAFE_HEIGHT;
		}		
	}
	
	// WRIST

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
	// Call update method every refresh cycle to update carriage and wrist movement
	public void update() {
		PIDHeight();
		PIDSpeed();
		wristUpdate();
		SmartDashboard.putNumber("Carriage Encoder", encoder.getDistance());
		SmartDashboard.putBoolean("Carriage Limit?", carriageLimit.get() == CARRIAGE_LIMIT_PRESSED);
		
	}
}
	