package org.usfirst.frc.team4564.robot;

public class Constants {
	// DIO Ports
	public static final int DIO_LIFT_BOTTOM = 0; // Lower Limit Switch
	public static final int DIO_CARRIAGE_TOP = 1;  // Top Carriage limit switch
	public static final int DIO_LIFT_ENCODER_A = 2;  
	public static final int DIO_LIFT_ENCODER_B = 3;  
	public static final int DIO_DRIVE_FB_ENCODER_A = 4;  
	public static final int DIO_DRIVE_FB_ENCODER_B = 5;  
	public static final int DIO_DRIVE_LR_ENCODER_A = 6;
	public static final int DIO_DRIVE_LR_ENCODER_B = 7;
	public static final int DIO_CARRIAGE_ENCODER_A = 8; 
	public static final int DIO_CARRIAGE_ENCODER_B = 9;
	public static final int DIO_VERTICAL_WRIST =  10;  // Vertical limit switch on wrist
	public static final int DIO_HORIZONTAL_WRIST =  11;  // Horizontal limit switch on wrist
	//PWM Ports
	public static final int PWM_WRIST_MOTOR = 0;
	public static final int PWM_DRIVE_FR = 1;
	public static final int PWM_DRIVE_RR = 2;
	public static final int PWM_DRIVE_FL = 3;
	public static final int PWM_DRIVE_RL = 4;
	public static final int PWM_DRIVE_FC = 5;
	public static final int PWM_DRIVE_RC = 6;
	public static final int PWM_LIFT_MOTOR = 7;
	public static final int PWM_CARRIAGE_MOTOR = 8;
	//Solenoids
	public static final int SOL_MAST_IN = 2;
	public static final int SOL_BRAKE = 3;
	public static final int SOL_MAST_OUT = 4;
	public static final int SOL_HAND_IN = 6; //Was 1
	public static final int SOL_HAND_OUT = 7; //Was 0
	//public static final int SOL_FOREBAR_DOWN = 6;
	//public static final int SOL_FOREBAR_UP = 7;
	public static final int SOL_GRAPPLER_IN = 6; //Was 6
	public static final int SOL_GRAPPLER_OUT = 7; //Was 7
	
	//Analogs
	public static final int ANA_HEADING = 0;
	public static final int ANA_TILT = 1;
	
	//MISCELLANEOUS
	public static final double REFRESH_RATE = 50;  //Refresh rate for main loop and all related subsystem updates
	
} 