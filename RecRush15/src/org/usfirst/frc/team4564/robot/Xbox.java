package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Xbox extends Joystick {
	// Variables to remember previous button state 
	private boolean prevA = false;
	private boolean prevB = false;
	private boolean prevX = false;
	private boolean prevY = false;
	private boolean prevRightBumper = false;
	private boolean prevLeftBumper = false;
	private boolean prevSelect = false;
	private boolean prevStart = false;
	private boolean prevLeftClick  = false;
	private boolean prevRightClick = false;
	private boolean prevDpadUp = false;
	private boolean prevDpadDown = false;
	private boolean prevDpadRight = false;
	private boolean prevDpadLeft = false;
	
	
	public Xbox(int port) {
		super(port);
	}
	
	public boolean A() {
		return getRawButton(1);
	}
	
	public boolean whenA() {
		if (A()) {
			if (prevA) {
				return false;
			} else {
				prevA = true;
				return true;
			}
		} else {
			prevA = false;
			return false;
		}
	}
	
	public boolean B() {
		return getRawButton(2);
	}
	
	public boolean whenB() {
		if (B()) {
			if (prevB) {
				return false;
			} else {
				prevB = true;
				return true;
			}
		} else {
			prevB = false;
			return false;
			}
		}
	
	public boolean X() {
		return getRawButton(3);
	}
	
	public boolean whenX() {
		if (X()) {
			if (prevX) {
				return false;
			} else {
				prevX = true;
				return true;
			}
		} else {
			prevX = false;
			return false;
		}
	}

	public boolean Y() {
		return getRawButton(4);
	}
	
	public boolean whenY() {
		if (Y()) {
			if (prevY) {
				return false;
			} else {
				prevX = true;
				return true;
				
			}
		} else {
			prevY  = false;
			return false;
			}
		}
	
	public boolean rightBumper() {
		return getRawButton(5);
	}
	
	public boolean whenRightBumper() {
		if (rightBumper()) {
			if (prevRightBumper) {
				return false;
			} else {
				prevRightBumper = true;
				return true;
			}
		} else {
			prevRightBumper = false;
			return false;
		}	
	}
	
	public boolean leftBumper() {
		return getRawButton(6);
		
	}
	
	public boolean whenLeftBumper() {
		if (leftBumper()) {
			if (prevLeftBumper) {
				return false;
			} else {
				prevLeftBumper = true;
				return true;
			}
		} else {
			prevLeftBumper = false;
			return false;
		}
	}
	public boolean select() {
		return getRawButton(7);
	}
	
	public boolean whenSelect() {
		if (select()) {
			if (prevSelect) {
				return false;
			} else {
				prevSelect = true;
				return true;
			}
		} else {
			prevSelect = false;
			return false;
		}
	}
	
	public boolean start() {
		return getRawButton(8);
	}
	
	public boolean whenStart() {
		if (leftBumper()) {
			if (prevStart) {
				return false;
			} else {
				prevStart = true;
				return true;
			}
		} else {
			prevStart = false;
			return false;
		}
	}
	
	public boolean leftClick() {
		return getRawButton(9);
	}
	
	public boolean whenLeftClick() {
		if (leftClick()) {
			if (prevLeftClick) {
				return false;
			} else {
				prevLeftClick = true;
				return true;
			}
		} else {
			prevLeftClick = false;
			return false;
		}
	}
	
	public boolean rightClick() {
		return getRawButton(10);
	}
	
	public boolean whenRightClick() {
		if (rightClick()) {
			if (prevRightClick) {
				return false;
			} else {
				prevRightClick = true;
				return true;
			}
		} else {
			prevRightClick = false;
			return false;
		}
	}

	public boolean dpadUp() {
		return getPOV(0) == 0;
	}
	
	public boolean whenDpadUp() {
		if (dpadUp()) {
			if (prevDpadUp) {
				return false;
			} else {
				prevDpadUp = true;
				return true;
			}
		} else {
			prevDpadUp = false;
			return false;
		}
	}
	
	public boolean dpadRight() {
		return getPOV(0) == 90;
	}
	
	public boolean whenDpadRight() {
		if (dpadRight()) {
			if (prevDpadRight) {
				return false;
			} else {
				prevDpadRight = true;
				return true;
			}
		} else {
			prevDpadRight = false;
			return false;
		}
	}
	public boolean dpadDown() {
		return getPOV(0) == 180;
	}
	
	public boolean whenDpadDown() {
		if (dpadDown()) {
			if (prevDpadDown) {
				return false;
			} else {
				prevDpadDown = true;
				return true;
			}
		} else {
			prevDpadDown = false;
			return false;
		}
	}
	
	public boolean dpadLeft() {
		return getPOV(0) == 270;
	}

	public boolean whenDpadLeft() {
		if (dpadLeft()) {
			if (prevDpadLeft) {
				return false;
			} else {
				prevDpadLeft = true;
				return true;
			}
		} else {
			prevDpadLeft = false;
			return false;
		}
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
