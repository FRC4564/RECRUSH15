package org.usfirst.frc.team4564.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto {
	
	// Robot Subsystems
	private DriveTrain dt;
	private Lift lift;
	private Claw claw;
	//Script Functions
	private Countdown autoTimer;  //Make sure auto run is no longer than 15 seconds
	private int selectedPlay = 1;  // Play # from playbook to run, stating at 1
	ArrayList<String> script = new ArrayList<String>();
	private ArrayList<ArrayList<String>> playbook = new ArrayList<ArrayList<String>>();
	// Command processing states
	private static final int STOPPED = 0;  //No script active
	private static final int STARTING = 1; //Starting a new process command
	private static final int RUNNING = 2;  //Process has begun, awaiting success or fail
	private static final int DONE = 3;  //Process command completed
	private static final int TIMEOUT = 4;  //Process command failed with timeout
	private static final int INVALID = 5;  //Command processor failed to set status properly
	private static final int BADCOMMAND = 6; //Invalid command
	
	private int processStatus = STOPPED;
	private Countdown processTimer = new Countdown();  //Some steps require a timer			
	// Constructor
	public Auto (DriveTrain d, Lift l, Claw c) {
		dt = d;
		lift = l;
		claw = c;

		//Script 1 Testing.
		script.add("driveInit");
		script.add("wait 5");
		script.add("driveForward 50");
		script.add("driveWait 5");
		script.add("driveBackward 50");
		script.add("driveWait 5");
		playbook.add(new ArrayList<String>(script));
		//Script 2 Our tote.
		script.clear();
		script.add("driveInit");
		script.add("clawInit");
		script.add("liftInit");
		script.add("liftWait 5");
		script.add("leftLevel 2");
		script.add("liftWait 5");

		//playbook.add(new ArrayList<String>(script));
		//script.clear();
		//Script 3 Our bin.
		//Script 4 Our bin and tote.
		//Script 5 Go to autozone.
		//Script 6 Our tote and right neighbor tote  (left starting postion).
		//Script 7 Our tote and right neighbor tote  (Middle starting postion).
		//Script 8 Our tote and left neighbor tote  (Right starting postion).
		//Script 9 Our tote and left neighbor tote  (Middle starting postion).
		//Script 10 Custom auto script.
	}
	
	// Increment the playbook selected play number, restricting to size of playbook
	public void nextPlay() {
		if (selectedPlay < playbook.size())
			selectedPlay += 1;
	}
	
	// Decrement the playbook selected play number, keeping it from going below zero.
	public void prevPlay() {
		if (selectedPlay > 0)
			selectedPlay -= 1;
	}
	
	// Returns the currently selected play number
	public int getPlay() {
		return selectedPlay;
	}
	
	// Run the selected play script, step by step.  Will run until all steps are done, or 15 seconds elapse.
	// Error conditions during the run will halt step execution and stop the run.
	public void run() {
		int stepIndex = 0;  			        // Pointer to the current step with a play script.
		String step;							// Current script step 
		Countdown autoTimer = new Countdown();	// Countdown timer to make sure the play completes within...
		
// ****** Changed from 14.9 to 60 for testing
		autoTimer.set(60.0);						// ...the 15 second autonomous period
		
		script = playbook.get(selectedPlay - 1);		// Load the script from the playbook
		Common.debug("Auto Script:" + script);
		Common.debug("Command: "+script.get(stepIndex));
		step = script.get(stepIndex);
		processStatus = STARTING;
		while (! autoTimer.done() && stepIndex < script.size()) {
			// Run the current step
			processStatus = runStep(step);
			// Process the status of the step
			switch (processStatus) {
				case DONE:
					stepIndex += 1;
					step = script.get(stepIndex);
					processStatus = STARTING;
					Common.debug("Command: " + step);
					break;
				case TIMEOUT:
					Common.debug("AutoRun: TIMEOUT on step "+ step);
					stepIndex = 999;
					break;
				case INVALID:
					Common.debug("AutoRun: INVALID STATUS - runCommand didn't set a status on step "+ step);
					stepIndex = 999;
					break;
				case BADCOMMAND:
					Common.debug("AutoRun: UNKNOWN COMMAND VERB/PARAMETER on step "+ step);
					stepIndex = 999;
					break;
			}
			// Refresh sub-systems every loop cycle
			dt.update();
			lift.update();
			claw.update();
			Timer.delay(1.0/Constants.REFRESH_RATE);
		}
		// Auto run complete - stop robot and be ready for TeleOp
		dt.hDrive(0,0,0);  //Stop drivetrain movement
		dt.setSpeed(100);  //Resume full speed operation
	}
	
	// Parse a step into Command and Parameter and execute. 
	// Returns result of the runCommand.
	private int runStep(String step) {
		String command;
		double parameter;
		command = step.split(" ")[0];
		command = command.toUpperCase();
		if (step.indexOf(" ") > 0) {
			parameter = Double.valueOf(step.split(" ")[1]);
			return runCommand(command, parameter);
		}
		return runCommand(command, 0);
	}

	// Runs the command, based on the current processStatus and returns an updated status.
	// The status needs to be set to STARTING when a new command is being initiated.
	// For commands that require multiple cycles to complete, the status will be set to RUNNING until DONE.
	// A status of INVALID signifies a logic error and runCommand itself.
	// A status of BADCOMMAND indicates unknown command verb or bad parameter for command.
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
			case "DRIVEBACKWARD":
				dt.moveForward(-parameter);
				break;
			case "DRIVERIGHT":
				//dt.;
				break;
			case "DRIVELEFT":
				//dt.;
				break;
			case "DRIVETURN":
				dt.rotate(parameter);
				break;
			case "DRIVEHEADING":
				dt.rotateTo(parameter);
				break;
			case "DRIVESPEED":
				dt.setSpeed((int) parameter);
				break;
			case "DRIVEWAIT":
				if (processStatus == STARTING) {
					if (parameter == 0) {
						processTimer.set(999);
					} else {
						processTimer.set(parameter);
					}
					status = RUNNING;
				} else {
					if (dt.isIdle()) {
						status = DONE;
					} else if (processTimer.done()) {
						status = TIMEOUT;
					} else {
						status = RUNNING;
					}
				}
				break;
			case "LIFTINIT":
				lift.init();
				break;
			case "LIFTLEVEL":
				if (parameter >= 0 && parameter <= 6) {
				lift.gotoLevel((int)parameter);
				} else {
					status = BADCOMMAND;
				}
				break;
			case "LIFTRELEASE":
				//lift.release();
				break;
			case "LIFTTO":
				lift.gotoHeight(parameter);
				break;
			case "LIFTWAIT":
				if (processStatus == STARTING) {
					if (parameter == 0) {
						processTimer.set(999);
					} else {
						processTimer.set(parameter);
					}
					status = RUNNING;
				} else {
					if (lift.isIdle()) {
						status = DONE;
					} else if (processTimer.done()) {
						status = TIMEOUT;
					} else {
						status = RUNNING;
					}	
				}
				break;
			case "WAIT":
				if (processStatus == STARTING) {
					processTimer.set(parameter);
					status = RUNNING;
				} else {
					if (processTimer.done()) {
						status = DONE;
					} else {
						status = RUNNING;
					}
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
