package org.usfirst.frc.team4564.robot;

import java.util.ArrayList;
import java.io.*;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
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
    private static final String COMMANDS="|SCRIPT|DRIVEINIT|LIFTINIT|CLAWINIT|DRIVESPEED|DRIVEFORWARD|DRIVEBACKWARD|DRIVELEFT|DRIVERIGHT|DRIVETURN|DRIVEHEADING|DRIVEWAIT|LIFTBOTTOM|LIFTLEVEL|LIFTRELEASE|LIFTTO|LIFTWAIT|HANDOPEN|HANDCLOSE|MASTIN|MASTOUT|FOREBARUP|FOREBARDOWN|CARRIAGETO|CARRIAGEWAIT";
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
		
		//When defining a script be sure the script does the following:
		//  1. driveInit - to calibrate gyro, set encoders, and zero targets. Suggest waiting a second or two after.
		//  2. liftInit - to bring lift to bottom and zero encoder - complete before end of auto mode
		//  3. clawInit - to bring lift to top of mast and zero encoder - complete before end of auto mode
		//  4. driveSpeed - adjust motors before driving, if less then full speed is desired.
		
		//Script 1 Testing.

		script.add("driveInit");
		script.add("liftInit");
		script.add("liftWait");
		script.add("mastIn");
		script.add("handClose");
		//script.add("wait 2");
		script.add("liftlevel 2");
		script.add("driveForward 83");
		script.add("driveWait 10");
		//script.add("wait 2");
		script.add("liftBottom");
		script.add("liftWait 3");
		script.add("liftlevel 2");
		script.add("Wait 2");
		script.add("driveturn 90");
		script.add("driveWait 10");
		script.add("driveSpeed 85");
		script.add("driveForward 110");
		script.add("driveWait 10");
		script.add("clawInit");
		script.add("carriageWait");
		playbook.add(new ArrayList<String>(script));
		//Script 2 Our tote.
		script.clear();
		script.add("driveInit");
		script.add("wait 1");
		script.add("driveTurn 90");
		script.add("wait 5");
		script.add("driveTurn 90");
		script.add("wait 5");
		script.add("driveTurn 90");
		script.add("wait 5");
		script.add("driveTurn -270");
		script.add("driveWait 5");
		script.add("wait 5");
		playbook.add(new ArrayList<String>(script));
		// Special secret script
		//script.clear();
		
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
			Common.debug("Play number: " + selectedPlay);
	}
	
	// Decrement the playbook selected play number, keeping it from going below zero.
	public void prevPlay() {
		if (selectedPlay > 1)
			selectedPlay -= 1;
			Common.debug("Play number: " + selectedPlay);
	}
	
	// Returns the currently selected play number
	public int getPlayNum() {
		return selectedPlay;
	}
	
	//Return script name of selected play
	public String getPlayName() {
		return playbook.get(selectedPlay - 1).get(0);
	}
	
	// Load playbook from file
    public void load(String file) {
      //  ArrayList<String> script = new ArrayList<String>();
	  //  ArrayList<ArrayList<String>> playbook = new ArrayList<ArrayList<String>>();
        String command;
        boolean failed = false;
        playbook.clear();
        script.clear();
        try {
            // Show current directory
            String current = new java.io.File( "." ).getCanonicalPath();
            System.out.println("Current Directory: " + current);
            // Open script file and process each line
            String line;
            FileReader fileReader = new FileReader("/home/lvuser/" + file);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            // Process each line of the file
           while((line = bufferedReader.readLine()) != null && ! failed) {
               String result = parseLine(line);
               command = result.split(" ")[0].toUpperCase();  
               if (command == "ERROR") {
                   System.out.println("Stopping Script Load");
                   failed = true;
               } else if (command.length() > 0) {
                   if (command.equals("SCRIPT")) { //Start of a Script?
                       System.out.println("Start of script "+result);
                       if (script.size() > 0) {  // If we already have a script started, save it and start anew
                           playbook.add(new ArrayList<String>(script));
                           script.clear();
                       }
                   }
                   script.add(result);  // Add command to script
                } 
            }
            // Finish up
            if (script.size() > 0) {  //Save the final play script, if we have one
                       playbook.add(new ArrayList<String>(script));
            }
            System.out.println(playbook);
            bufferedReader.close();
        }
        catch(IOException e) {
            e.printStackTrace();
        }
    }
    
    // parseLine loaded from script file
    // Validates line, returning line cleaned of extra spaces
    private static String parseLine(String line) {
        String command, parameter, result;
        result = "";
        line = line.trim();
        if (line.length() > 0) {      //Process line if not empty
            if (! line.substring(0,1).equals("#")) {  //Process line if it doesn't being with a comment symbol (#)
                command = line.split(" ")[0];                         // Parse command off front of line
                parameter = line.substring(command.length()).trim();  // Remainder of line is the parameter
                if (COMMANDS.indexOf(command.toUpperCase()) >= 0) {   // If SCRIPT command, then start a new playbook entry
                    
                    //System.out.println("GOOD: '"+command+ "' PARAM: " + parameter);
                    result = command+" "+parameter;
                } else {
                    System.out.println("INVALID COMMAND: "+command+ " PARAM: " + line);
                    result = "ERROR";
                }
            }
        }
        return result.trim();
    }

	// Run the selected play script, step by step.  Will run until all steps are done, or 15 seconds elapse.
	// Error conditions during the run will halt step execution and stop the run.
	public void run() {
		int stepIndex = 0;  			        // Pointer to the current step with a play script.
		boolean done = false;					// Loop through commands until finished or errored
		String step;							// Current script step 
		Countdown autoTimer = new Countdown();	// Countdown timer to make sure the play completes within...
		
// ****** Changed from 14.9 to 60 for testing
		autoTimer.set(60.0);						// ...the 15 second autonomous period
		
		script = playbook.get(selectedPlay - 1);		// Load the script from the playbook
		Common.debug("Auto Script:" + script);
		processStatus = STARTING;
		step = script.get(stepIndex);
		Common.debug("Command: "+step);
		while (!done && RobotState.isEnabled()) {
			// See if loop should stop
			if (autoTimer.done()) {
				Common.debug("AUTO took too long...had to exit");
				done = true;
			}  else {
				// Run the current step
				processStatus = runStep(step);
				// Process the status of the step
				switch (processStatus) {
					case DONE:
						stepIndex += 1;
						if (stepIndex >= script.size()) {
							Common.debug("AUTO script ended normally");
							done = true;
						} else {
							processStatus = STARTING;
							step = script.get(stepIndex);
							Common.debug("Command: " + step);
						}
						break;
					case TIMEOUT:
						Common.debug("AutoRun: TIMEOUT on step "+ step);
						done = true;
						break;
					case INVALID:
						Common.debug("AutoRun: INVALID STATUS - runCommand didn't set a status on step "+ step);
						done = true;
						break;
					case BADCOMMAND:
						Common.debug("AutoRun: UNKNOWN COMMAND VERB/PARAMETER on step "+ step);
						done = true;
						break;
				}
				// Refresh sub-systems every loop cycle
				dt.update();
				lift.update();
				claw.update();
				Timer.delay(1.0/Constants.REFRESH_RATE);

			}
		}
		// Auto run complete - stop robot and be ready for TeleOp
		dt.stop();  //Stop drivetrain movement
		dt.setSpeed(100);  //Resume full speed operation
		lift.stop();
		claw.stop();
	}
	
	// Parse a step into Command and Parameter and execute. 
	// Returns result of the runCommand.
	private int runStep(String step) {
		String command;
		String parameter;
		command = step.split(" ")[0];
		command = command.toUpperCase();
		if (step.indexOf(" ") > 0) {
			parameter = step.split(" ")[1];
		} else {
			parameter = "0";
		}
		return runCommand(command, parameter);
	}

	// Runs the command, based on the current processStatus and returns an updated status.
	// The status needs to be set to STARTING when a new command is being initiated.
	// For commands that require multiple cycles to complete, the status will be set to RUNNING until DONE.
	// A status of INVALID signifies a logic error and runCommand itself.
	// A status of BADCOMMAND indicates unknown command verb or bad parameter for command.
	private int runCommand(String command,String parameter)  {
		int status = INVALID;  //Status will change if command processes properly
		switch (command) {
			case "SCRIPT":
				status = DONE;
				break;
			// driveInit
			case "DRIVEINIT":
				dt.init();
				status = DONE;
				break;
			// driveForward <inches>	
			case "DRIVEFORWARD":
				dt.moveForward(Double.parseDouble(parameter));
				status = DONE;
				break;
			// liftWait <timeout>
			case "DRIVEBACKWARD":
				dt.moveBackward(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "DRIVERIGHT":
				dt.moveRight(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "DRIVELEFT":
				dt.moveLeft(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "DRIVETURN":
				dt.rotate(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "DRIVEHEADING":
				dt.rotateTo(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "DRIVESPEED":
				dt.setSpeed(Integer.parseInt(parameter));
				status = DONE;
				break;
			case "DRIVEWAIT":
				if (processStatus == STARTING) {
					if (Double.parseDouble(parameter) == 0) {
						processTimer.set(999);
					} else {
						processTimer.set(Double.parseDouble(parameter));
					}
					status = RUNNING;
				} else {  // running status
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
				status = DONE;
				break;
			case "LIFTLEVEL":
				if (Integer.parseInt(parameter) >= 0 && Integer.parseInt(parameter) <= 6) {
					lift.gotoLevel(Integer.parseInt(parameter));
					status = DONE;
				} else {
					status = BADCOMMAND;
				}
				break;
			case "LIFTRELEASE":
				lift.releaseTote();
				status = DONE;
				break;
			case "LIFTTO":
				lift.gotoHeight(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "LIFTBOTTOM":
				lift.bottom();
				status = DONE;
				break;
			case "LIFTWAIT":
				if (processStatus == STARTING) {
					if (Double.parseDouble(parameter) == 0) {
						processTimer.set(999);
					} else {
						processTimer.set(Double.parseDouble(parameter));
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
			// CLAW COMMANDS
			case "CLAWINIT":
				claw.init();
				status = DONE;
				break;
			case "CARRIAGETO":
				claw.carriageTo(Double.parseDouble(parameter));
				status = DONE;
				break;
			case "CARRIAGEWAIT":
				if (processStatus == STARTING) {
					if (Double.parseDouble(parameter) == 0) {
						processTimer.set(999);
					} else {
						processTimer.set(Double.parseDouble(parameter));
					}
					status = RUNNING;
				} else {
					if (claw.isIdle()) {
						status = DONE;
					} else if (processTimer.done()) {
						status = TIMEOUT;
					} else {
						status = RUNNING;
					}	
				}
				break;
			case "HANDOPEN":
				claw.handOpen();
				status = DONE;
				break;
			case "HANDCLOSE":
				claw.handClose();
				status = DONE;
				break;
			case "MASTIN":
				claw.mastIn();
				status = DONE;
				break;
			case "MASTOUT":
				claw.mastOut();
				status = DONE;				
				break;
			case "FOREBARUP":
				claw.forebarUp();
				status = DONE;
				break;
			case "FOREBARDOWN":
				claw.forebarDown();
				status = DONE;				
				break;
			case "WAIT":
				if (processStatus == STARTING) {
					processTimer.set(Double.parseDouble(parameter));
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
