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
	private static final int RUNNING = 1;  //Process has begun, awaiting success or fail
	private static final int DONE = 2;  //Process step completed
	private static final int TIMEOUT = 3;  //Process step failed with timeout
	private static final int INVALID = 4;  //Command processor failed to set status properly
	private static final int BADCOMMAND = 5; //Invalid command
	private int processStatus = STOPPED;
	private Countdown processTimer = new Countdown();  //Some steps require a timer			
	// Constructor
	public Auto (DriveTrain dt, Lift lift, Claw claw) {
		this.dt = dt;
		this.lift = lift;
		this.claw = claw;

		//Script 1 All totes.
		script.add("driveInit");
		script.add("driveForward 12");
		script.add("driveWait 5");
		script.add("driveBackward 12");
		script.add("driveWait 5");
		playbook.add(new ArrayList<String>(script));
		script.clear();
		//Script 2 Our tote.
		script.add("liftInit");
		script.add("liftWait 5");
		script.add("level 2");
		script.add("liftWait 5");
		playbook.add(new ArrayList<String>(script));
		script.clear();
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
	
	public void run() {
		int stepIndex = 0;  			        // Pointer to the current step with a play script.
		Countdown autoTimer = new Countdown();	// Countdown timer to make sure the play completes within...
		autoTimer.set(15);						// ...the 15 second autonomous period
		
		script = playbook.get(selectedPlay - 1);		// Load the script from the playbook
		System.out.println(script);

		while (! autoTimer.done()) {
			if (stepIndex < script.size()) {
				switch (runStep(script.get(stepIndex))) {
					case DONE:
						stepIndex += 1;
						SmartDashboard.putString("Step ndx", script.get(stepIndex));
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
			dt.updateMove();
			lift.update();
			claw.update();
			Timer.delay(2);
		}
	}
	
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

	private int runCommand(String command,double parameter)  {
		int status = INVALID;  //Status will change if command processes properly
		SmartDashboard.putString("Command", command);
		return DONE;
		/*switch (command) {
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
				dt.rotateTo(dt.getHeading() + parameter);
				break;
			case "DRIVEHEADING" :
				dt.rotateTo(parameter);
				break;
			case "DRIVEWAIT":
				if (processStatus == RUNNING) {
					if (dt.isIdle()) {
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
		return status;*/
	}
}
