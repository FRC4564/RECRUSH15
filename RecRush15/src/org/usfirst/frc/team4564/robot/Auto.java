package org.usfirst.frc.team4564.robot;

import java.util.ArrayList;

public class Auto {
	
	// Robot Subsystems
	private DriveTrain dt;
	private Lift lift;
	private Claw claw;
	
	private Countdown autoTimer;  //Make sure auto run is no longer than 15 seconds
	ArrayList<String> script = new ArrayList<String>();
	private ArrayList<ArrayList<String>> playbook = new ArrayList<ArrayList<String>>();
	
	// Command processing states
	private static final int STOPPED = 0;  //No script active
	private static final int RUNNING = 1;  //Process has begun, awaiting success or fail
	private static final int DONE = 2;  //Process step completed
	private static final int TIMEOUT = 3;  //Process step failed with timeout
	private static final int INVALID = 4;  //Command processor failed to set status properly
	private static final int BADCOMMAND = 5; //Invalid command
	private int processStatus = STOPPED;
	private Countdown processTimer = new Countdown();  //Some steps require a timer			
	
	public Auto (DriveTrain dt, Lift lift, Claw claw) {
		this.dt = dt;
		this.lift = lift;
		this.claw = claw;
	

		//Script 1
		script.add("driveInit");
		script.add("driveForward 12");
		script.add("driveWait 5");
		script.add("driveBackward 12");
		script.add("driveWait 5");
		playbook.add(script);
		script.clear();
		//Script 2
		script.add("liftInit");
		script.add("liftWait 5");
		script.add("level 2");
		script.add("liftWait 5");
		playbook.add(script);
		script.clear();
		
		
	}
	
	public void run(int play) {
		int stepIndex = 0;  			// Pointer to next step to process
		Countdown autoTimer = new Countdown();	// Countdown timer to make sure run stays within autonomous period
		autoTimer.set(15);
		
		script = playbook.get(play);		// Load the script from the playbook

		while (! autoTimer.done()) {
			if (stepIndex < script.size()) {
				switch (runStep(script.get(stepIndex))) {
					case DONE:
						stepIndex += 1;
						break;
					case TIMEOUT:
						System.err.print("TIMEOUT AT STEP "+ stepIndex);
						stepIndex = 999;
						break;
					case INVALID:
						System.err.print("INVALID STATUS - PROCESS COMMAND METHOD BUG - AT STEP "+ stepIndex);
						stepIndex = 999;
						break;
					case BADCOMMAND:
						System.err.print("UNKNOWN COMMAND VERB AT STEP "+ stepIndex);
						stepIndex = 999;
						break;
				}
			}
		}
	}
	
	private int runStep(String step) {
		String command;
		double parameter;
		
		command = step.split(" ")[0];
		command = command.toUpperCase();
		parameter = Double.valueOf(step.split(" ")[1]);
		
		return runCommand(command,parameter);				
	}

	private int runCommand(String command,double parameter)  {
		int status = INVALID;  //Status will change if command processes properly
		
		switch (command) {
			// driveInit
			case "DRIVEINIT":
				dt.init();
				status = DONE;
				break;
			// driveForward <inches>	
			case "DRIVEFORWARD":
				dt.moveForward(parameter);
				status = DONE;
				break;
			// liftWait <timeout>	
			case "LIFTWAIT":
				if (processStatus == RUNNING) {
					if (lift.isIdle()) {
						status = DONE;
					} else if (processTimer.done()) {
						status = TIMEOUT;
					}
				} else {
					if (parameter == 0) {
						processTimer.set(999);
					} else {
						processTimer.set(parameter);
					}
					status = RUNNING;
				}
				break;
			// Unknown command
			default:
				status = BADCOMMAND;
				break;
		}
		return status;
	}
}
