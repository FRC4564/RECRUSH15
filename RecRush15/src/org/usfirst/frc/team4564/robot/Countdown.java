package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.Timer;

public class Countdown {

double startTime;
double endTime;
public Countdown() {}

	public void set(double seconds) {
		startTime = System.currentTimeMillis();
		endTime = startTime + seconds*1000;
	}

	// Return countdown time remaining
	public double time() {
		double timeLeft = endTime - System.currentTimeMillis();
		if (endTime <= 0) {
			return 0;
		} else {
			return timeLeft/1000;
		}
	}

	// Is countdown done?
	public boolean done() {
		if (System.currentTimeMillis() > endTime) {
			return true;
		} else {
			return false;
		}
	}
}
