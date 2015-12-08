
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

/*public class Robot2 extends SampleRobot {
    DriveTrain dt;
    Auto auto;
    Xbox j1 = new Xbox(0);
    Xbox j2 = new Xbox(1);
    Compressor comp = new Compressor();
    Lift lift = new Lift();
    Claw claw = new Claw();
    private boolean j1Boolean = false;

    public Robot2() {
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
        	if (j1.whenDpadUp() || j2.whenDpadUp()) {
        		Common.debug("Incrementing play number");
        		auto.nextPlay();
        	}
        	
        	if (j1.whenDpadDown() || j2.whenDpadDown()) {
        		Common.debug("Decrementing play number");
        		 auto.prevPlay();
        	}
        	if (j1.whenSelect()  || j2.whenSelect()) {
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
        	
        	// Disable j2 if either j1 joystick is active
            if ((j1.rightY() != 0.0) || (j1.rightX() != 0.0) || (j1.leftX() != 0.0) || (j1.leftY() != 0.0)) {
    			j1Boolean = true;
    		}
            else{
            	j1Boolean = false;
            }
        	
        	// DRIVE TRAIN
        	if (j1.leftY() == 0 && j1.leftX() == 0 && j1.rightTrigger() == 0 && j1.leftTrigger() == 0 && j1Boolean == false) {
	       		dt.hDrive(-j2.leftY(), j2.leftX(), j2.leftTrigger() - j2.rightTrigger());
        	} else {
	       		dt.hDrive(j1.leftY(), j1.leftX(), j1.rightTrigger() - j1.leftTrigger());	// Drive with arcade style using left stick by default	
        	}
	       		
        	// LIFT
    		
	    	if (j1.whenA()) {							// Level down
	    		lift.levelDown();	    	
	    	}
	    	if (j1.whenB()) {							// level up
	    		lift.levelUp();
	    	}
	    	if (lift.isIdle() || lift.isMoving()) {			// Move lift freely, if ready for movement
        		if (j1.rightY() !=0) {
        			lift.moveFree(j1.rightY());
        		}
        		else {
        			lift.moveFree(j2.rightY());
        		}
        	}
        	
        	if (j1.whenStart() || j2.whenStart()) {					
        		Common.debug("Initializing Lift and Claw");
        		lift.init();
        		claw.init();
        	}

        	// COMPRESSOR
        	if (j1.whenSelect() || j2.whenSelect()) {					// Toggle compressor
        		if (comp.enabled() == true) {
        			comp.stop();
        		} if (comp.enabled() == false) {
        			comp.start();
        		}	
        	}
        	
        	
        	// UPDATE SUBSYSTEMS
        	lift.update();
        	claw.update();

            Timer.delay(1.0 / Constants.REFRESH_RATE);		// wait before repeating main update loop
        }
        Common.debug("Ending: teleop");
    }

    //**
    // * Runs during test mode
    //
    public void test() {
    	Common.debug("Starting: test");
        dt.setSafetyEnabled(true);  
    	if (lift.isIdle() != true) {
    		lift.init();
    	}
    	if (claw.isIdle() != true) {
    		claw.init();
    	}

    	while (isEnabled()) {
    	   	dt.update();
	    	lift.update();
	    	claw.update();
	    	
	    	Timer.delay(1.0 / Constants.REFRESH_RATE);
        }
    }
} 
*/