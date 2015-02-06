package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class Claw {
	
	//Mast States

	//Carriage States
	//private final static int CARRIAGE_IDLE = 0;
	//private final static int CARRIAGE_MOVING = 1;
	//Forebar States
	//private static final int FOREBAR_UP = 1;
	//private static final int FOREBAR_DOWN = 0;
	//Wrist States
	//private final static int WRIST_MOVING = 1;
	//private final static int WRIST_IDLE = 0;
	//Hand States
	
	//Define Mast 
	Solenoid mastSol = new Solenoid(Constants.SOL_MAST);
	private static final boolean MAST_IN = true;
	private static final boolean MAST_OUT = ! MAST_IN;
	private boolean mastPositionIn = true ;  // True when mast is retracted
	//Define Carriage 
	private static final boolean CARRIAGE_LIMIT_PRESSED = false;
	private static final double CARRIAGE_LOWER_LIMIT = 0;
	private DigitalInput CarriageLimit = new DigitalInput(Constants.DIO_CARRIAGE_TOP);
	private Encoder encoder = new Encoder(Constants.DIO_CARRIAGE_ENCODER_A, Constants.DIO_CARRIAGE_ENCODER_B,
				  true, EncodingType.k4X);
	
	// PID Carriage
	private static final double HEIGHT_Kp = 3.0;
    private static final double HEIGHT_Kd = 0.1;
    private static final double SP_Kp = 0.002;
    private static final double SP_Kd = 0.000;
    private double targetPIDHeight = CARRIAGE_LOWER_LIMIT;
	private double prevPIDHeightError = 0.0;
	private double prevPIDSpeedError = 0.0;
	private double prevPIDSpeedPower = 0.0;
	private static final double SP_MAX_IPS = 15.0;
	private double targetPIDSpeed = 0.0;
	private Talon liftMotor = new Talon(Constants.PWM_CARRIAGE_MOTOR);
	private static final boolean MOTOR_INVERT = true;
	private static final double MOTOR_MAX_POWER = 1.0;


	//Define Forebar
	Solenoid forebarUp = new Solenoid(Constants.SOL_FOREBAR_UP);
	Solenoid forebarDown = new Solenoid(Constants.SOL_FOREBAR_DOWN);
	private static final boolean FOREBAR_SOL_ENABLE = true;
	private static final boolean FOREBAR_SOL_DISABLE = ! FOREBAR_SOL_ENABLE;
	private boolean forebarPostionUp = true;
	private boolean forebarPostionStop = false;
	
	//Define Wrist 
	private static final boolean VERTICAL_LIMIT_PRESSED = false;
	private static final boolean HORIZONTAL_LIMIT_PRESSED = false;
	private DigitalInput verticalLimit = new DigitalInput(Constants.DIO_VERTICAL_WRIST);
	private DigitalInput horizontalLimit = new DigitalInput(Constants.DIO_HORIZONTAL_WRIST);
    private Talon wristMotor = new Talon(Constants.PWM_WRIST_MOTOR);
    
    //Wrist States
    private int wristState = WRIST_STOPPED_HOR;
    private static final int WRIST_STOPPED_HOR = 1;
    private static final int WRIST_STOPPED_VER = 2;
    private static final int WRIST_MOVING_HOR = 3;
    private static final int WRIST_MOVING_VER = 4;
   
	//Define Hand 
    Solenoid handSol = new Solenoid(Constants.SOL_HAND);
	private static final boolean HAND_OPEN = true;
	private static final boolean HAND_CLOSE = ! HAND_OPEN;
	
	public double getHeight() {
		return encoder.getDistance() + CARRIAGE_LOWER_LIMIT;
	}
		
	public void mastIn() {
	    mastSol.set(MAST_IN);
	    mastPositionIn = true;
	    	}	
	
	public void mastOut() {
		mastSol.set(MAST_OUT);
		mastPositionIn = false;
	}
	
	public void mastToggle() {
		if(mastPositionIn) {
			mastOut();
		} else {
			mastIn();
		}
	}
	
	public void FOREBAR_UP() {
		forebarUp.set(FOREBAR_SOL_ENABLE);
		forebarDown.set(FOREBAR_SOL_DISABLE);
		forebarPostionUp = true;
	}
	
	public void FORBAR_SOL_DOWN() {
		forebarDown.set(FOREBAR_SOL_DISABLE);
		forebarUp.set(FOREBAR_SOL_ENABLE);
		forebarPostionUp = false;
	}
	
	public void forebareDown() {
		forebarDown.set(FOREBAR_SOL_DISABLE);
		forebarUp.set(FOREBAR_SOL_DISABLE);
		forebarPostionStop = true;
	}
	
	
	//CARRIAGE
	
	private void setMotor(double power) {
		// determine if we are moving upwards
		boolean directionUp = true;
		if (power < 0) {
				directionUp = false;
		}
		// adjust direction of motorSpeed if it needs to be inverted
		if (MOTOR_INVERT) {
			power = -power;
		}
		// based on the direction, determine which limit switch to test
		// and move at the power provided.
		if (CarriageLimit.get() == CARRIAGE_LIMIT_PRESSED) {
				power = 0;
			}
		liftMotor.set(power);
		}
		
	
	private void PIDSpeed() {
		double error, power, P, D;
		// Calculate PID
		error = targetPIDSpeed - encoder.getRate();
		P = error * SP_Kp;
		D = (error - prevPIDSpeedError) / (1.0 / Constants.REFRESH_RATE) * SP_Kd;
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
	
	private void carriagePIDHeight() {
	    double error, speed, P, D;
		// Calculate PID
		error = targetPIDHeight - getHeight();  // Height from floor targeted and current encoder  
		P = error * HEIGHT_Kp;
		D = (error - prevPIDHeightError) / (1.0 / Constants.REFRESH_RATE) * HEIGHT_Kd;
		prevPIDHeightError = error;
		
		speed = P + D;
		if (speed > SP_MAX_IPS) {
			speed = SP_MAX_IPS;
		} else if (speed < -SP_MAX_IPS) {
			speed = -SP_MAX_IPS;
		}
		targetPIDSpeed = speed;
	}

	// WRIST


		public void	rotateClaw() {
			if wristState = WRIST_STOPPED_HOR {
				
			}
			
		}
}


		
	