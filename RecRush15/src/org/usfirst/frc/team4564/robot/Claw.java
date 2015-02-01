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
	private static final boolean LOWER_CARRIAGE_LIMIT_PRESSED = false;
	private DigitalInput lowerCarriageLimit = new DigitalInput(Constants.DIO_CARRIAGE_TOP);
	private Encoder encoder = new Encoder(Constants.DIO_CARRIAGE_ENCODER_A, Constants.DIO_CARRIAGE_ENCODER_B,
				  true, EncodingType.k4X);
	//Define Forebar
	Solenoid forebarUp = new Solenoid(Constants.SOL_FOREBAR_UP);
	Solenoid forebarDown = new Solenoid(Constants.SOL_FOREBAR_DOWN);
	private static final boolean FOREBAR_SOL_ENABLE = true;
	private static final boolean FOREBAR_SOL_DISABLE = ! FOREBAR_SOL_ENABLE;
	//Define Wrist 
	private static final boolean VERTICAL_LIMIT_PRESSED = false;
	private static final boolean HORIZONTAL_LIMIT_PRESSED = false;
	private DigitalInput verticalLimit = new DigitalInput(Constants.DIO_VERTICAL_WRIST);
	private DigitalInput horizontalLimit = new DigitalInput(Constants.DIO_HORIZONTAL_WRIST);
    private Talon wristMotor = new Talon(Constants.PWM_WRIST_MOTOR);
	//Define Hand 
    Solenoid handSol = new Solenoid(Constants.SOL_HAND);
	private static final boolean HAND_OPEN = true;
	private static final boolean HAND_CLOSE = ! HAND_OPEN;
	
	public void mastIn() {
	    mastSol.set(MAST_IN);
	    	}	
	
	public void mastOut() {
		mastSol.set(MAST_OUT);
	}
	
	public void mastToggle() {
		if(mastPositionIn) {
			mastOut();
		} else {
			mastIn();
		}
	}
}
		
	