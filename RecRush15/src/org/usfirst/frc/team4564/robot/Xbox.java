package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Xbox extends Joystick {
	
	public Xbox(int port) {
		super(port);
	}

	public boolean A() {
		return getRawButton(1);
	}
	
	public boolean B() {
		return getRawButton(2);
	}
	
	public boolean X() {
		return getRawButton(3);
	}
	
	public boolean Y() {
		return getRawButton(4);
	}
	
	public boolean rightBumper() {
		return getRawButton(5);
	}
	
	public boolean leftBumper() {
		return getRawButton(6);
		
	}
	
	public boolean select() {
		return getRawButton(7);
	}
	
	public boolean start() {
		return getRawButton(8);
	}
	
	public boolean leftClick() {
		return getRawButton(9);
	}
	
	public boolean rightClick() {
		return getRawButton(10);
	}
	
	public boolean dpadUp() {
		return getPOV(0) == 0;
	}
	
	public boolean dpadRight() {
		return getPOV(0) == 90;
	}
	
	public boolean dpadDown() {
		return getPOV(0) == 180;
	}
	
	public boolean dpadLeft() {
		return getPOV(0) == 270;
	}
	
	public double leftX() {
		return getRawAxis(1);
	}
	
	public double leftY() {
		return getRawAxis(2);
	}
	
	public double leftTrigger() {
		return getRawAxis(3);
	}
	
	public double rightX() {
		return getRawAxis(4);
	}
	
	public double rightY() {
		return getRawAxis(5);
	}
	
	public double rightTrigger() {
		return getRawAxis(6);
	}
}
