package org.usfirst.frc.team4564.robot;

public class Common {

	public boolean within(double value, double target, double tolerance) {
		boolean result=false;
		if (Math.abs(value - target) <= tolerance) {
			result = true;
		} if (Math.abs(value - target) > tolerance) {
			result = false;
		}
		return result;
	}
	
	// Constrain value within min and max values.
	// Negative values constrained within -Min and -Max.
	public static double constrain(double value, double min, double max) {
		if (value >= 0) {
			if (value > max) {
				value = max;
			} else if (value < min) {
				value = min;
			}
	 	} else {
	 		if (value < -max) {
				value = -max;
			} if (value > -min) {
				value = -min;
			}
		}
		return value;
	}
	
	public static void debug(String a) {
		System.out.println(System.currentTimeMillis()/1000+": "+a);
	}
}


