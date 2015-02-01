package org.usfirst.frc.team4564.robot;

public class Constants {
	// DIO Ports
	public static final int DIO_LIFT_BOTTOM = 0; // Lower Limit Switch
	public static final int DIO_LIFT_TOP = 1; //Upper Limit Switch
	public static final int DIO_LIFT_ENCODER_A = 2;  
	public static final int DIO_LIFT_ENCODER_B = 3;  
	public static final int DIO_DRIVE_FB_ENCODER_A = 4;  
	public static final int DIO_DRIVE_FB_ENCODER_B = 5;  
	public static final int DIO_DRIVE_LR_ENCODER_A = 6;
	public static final int DIO_DRIVE_LR_ENCODER_B = 7;
	
	
	
	//PWM Ports
	public static final int PWM_DRIVE_FR = 0;
	public static final int PWM_DRIVE_RR = 1;
	public static final int PWM_DRIVE_FL = 2;
	public static final int PWM_DRIVE_RL = 3;
	public static final int PWM_DRIVE_FC = 4;
	public static final int PWM_DRIVE_RC = 5;
	public static final int PWM_LIFT_MOTOR = 6;

	
	
	//MISCELLANEOUS
	public static final double REFRESH_RATE = 100;  //Refresh rate for main loop and all related subsystem updates
	
} 