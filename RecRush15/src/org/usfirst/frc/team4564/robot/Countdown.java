package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.Timer;

public class Countdown {

double startTime;
double endTime;
public Countdown() {}

	public void set(double seconds) {
		startTime = Timer.getFPGATimestamp();
		endTime = startTime + seconds;
	}

	// Return countdown time remaining
	public double time() {
		double timeLeft = endTime - Timer.getFPGATimestamp();
		if (endTime <= 0) {
			return 0;
		} else {
			return timeLeft;
		}
	}

	// Is countdown done?
	public boolean done() {
		if (Timer.getFPGATimestamp() > endTime) {
			return true;
		} else {
			return false;
		}
	}
}
