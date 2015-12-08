
package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 */
public class Robot extends SampleRobot {
    DriveTrain dt;
    Auto auto;
    Xbox joyTote = new Xbox(0);
    Xbox joyBin = new Xbox(1);
    Compressor comp = new Compressor();
    Lift lift = new Lift();
    Claw claw = new Claw();

    public Robot() {
    	Common.debug("Constructing drive train");
        dt = new DriveTrain();
        dt.setSafetyEnabled(false);  //Safety will be enabled for TeleOp
        dt.setExpiration(0.1);   
        auto = new Auto(dt, lift, claw);
        auto.load("playbook.txt");
    }
    
    // ROBOTINIT
    // Runs once, after robot is booted
    public void robotInit() {
    	Common.debug("Starting: robotInit()");
    	comp.start();
        dt.init();
    }
    
    // DISABLED MODE
    // Robot is disabled.  Allow selection of Auto play mode.
    public void disabled() {
    	Common.debug("Starting: disabled mode");
        while (isEnabled() == false) {
        	if (joyTote.whenDpadUp() || joyBin.whenDpadUp()) {
        		Common.debug("Incrementing play number");
        		auto.nextPlay();
        	}
        	
        	if (joyTote.whenDpadDown() || joyBin.whenDpadDown()) {
        		Common.debug("Decrementing play number");
        		 auto.prevPlay();
        	}
        	if (joyTote.whenSelect()  || joyBin.whenSelect()) {
        		 auto.load("playbook.txt");
        	}
        		
    		SmartDashboard.putNumber("Play Number",auto.getPlayNum());
    		SmartDashboard.putString("Play", auto.getPlayName());
        	Timer.delay(.1);
        } 
        Common.debug("Ending: disabled mode");
    }

    
    // AUTONOMOUS MODE
    public void autonomous() {
        dt.setSafetyEnabled(false);
        Common.debug("Starting Auto Play #"+auto.getPlayNum());
        auto.run();
    }

    // TELEOP MODE
    public void operatorControl() {
    	Common.debug("Starting: teleop");
    	dt.init();
        dt.setSafetyEnabled(true);  
        while (isOperatorControl() && isEnabled()) {
        	
        	// DRIVE TRAIN
        	if (joyTote.leftY() == 0 && joyTote.leftX() == 0 && joyTote.rightTrigger() == 0 && joyTote.leftTrigger() == 0) {
	       		dt.hDrive(-joyBin.leftY(), joyBin.leftX(), joyBin.leftTrigger() - joyBin.rightTrigger());
        	} else {
	       		dt.hDrive(joyTote.leftY(), joyTote.leftX(), joyTote.rightTrigger() - joyTote.leftTrigger());	// Drive with arcade style using left stick by default	
        	}
	       	
        	// GRAPPLER
	    	if (joyTote.whenA()) {							// Toggle grappler open/close
	    		claw.grapplerToggle();	    	
	    	}
        	
        	// LIFT
    		/* Disable the level up/down with A/B buttons
	    	if (joyTote.whenA()) {							// Level down
	    		lift.levelDown();	    	
	    	}
	    	if (joyTote.whenB()) {							// level up
	    		lift.levelUp();
	    	}
	    	*/
	    	if (lift.isIdle() || lift.isMoving()) {			// Move lift freely, if ready for movement
        		if (joyTote.rightY() !=0 ) {
        			lift.moveFree(joyTote.rightY());
        		}
        		else  {
        			lift.moveFree(joyBin.rightY());
        		}
        	}
        	
        	if (joyTote.whenStart() || joyBin.whenStart()) {					
        		Common.debug("Initializing Lift and Claw");
        		lift.init();
        		claw.init();
        	}

        	// COMPRESSOR
        	if (joyTote.whenSelect() || joyBin.whenSelect()) {					// Toggle compressor
        		if (comp.enabled() == true) {
        			comp.stop();
        		} if (comp.enabled() == false) {
        			comp.start();
        		}	
        	}
        	
        	// CARRIAGE
        	if (joyBin.leftBumper()) {					
         		// Carriage up
        	}
        	if (joyBin.leftTrigger() > .5) {
        		 // Carriage down
        	}
        	
        	claw.moveFree(-joyBin.rightY());			// Move carriage freely, if ready for movement
        	
 /* disabled this code for Grappler       	
        	// FOREBAR
        	if (joyBin.dpadUp() || joyBin.leftBumper()) {						// Forebar is tri-state: Up, Down or Stopped
        		claw.forebarUp();
        	} else if (joyBin.dpadDown() || joyBin.rightBumper()) {
        		claw.forebarDown();
        	} else {
        		claw.forebarStop();
        	}
*/

        	// MAST
        	if (joyBin.whenY()) {
        		claw.mastToggle();        	
        	} 
        	
        	//CLAW
 	        if (joyBin.whenDpadRight()) {		
 	        	//Claw rotate
 	        }	
 	        if (joyBin.whenDpadLeft()) {
 	        	//Claw rotate
 	        }
 	        
 	        // HAND
 	        if (joyBin.whenA()) {
 	        	claw.handToggle();
 	        }
 	        
 	        // Wrist
 	        if (joyBin.dpadLeft()) {
 	        	claw.wristLeft();
 	        } else if (joyBin.dpadRight()) {
 	        		claw.wristRight();
 	        } else {
 	        	claw.wristStop();
 	        }
 	        
        	// UPDATE SUBSYSTEMS
        	lift.update();
        	claw.update();

            Timer.delay(1.0 / Constants.REFRESH_RATE);		// wait before repeating main update loop
        }
        Common.debug("Ending: teleop");
    }

    /**
     * Runs during test mode
     */
    public void test() {
    	Common.debug("Starting: test");
        dt.setSafetyEnabled(true);  
    	while (isEnabled()) {
    		dt.hDrive(0.0, joyTote.leftX(), 0.0);
    	   	//dt.update();
	    	lift.update();
	    	SmartDashboard.putNumber("Gyro Angle", dt.gyro.getAngle());
	    	SmartDashboard.putNumber("Gyro Rate", dt.gyro.getRate());
	    	
	    	Timer.delay(1.0 / Constants.REFRESH_RATE);
        }
    }
}