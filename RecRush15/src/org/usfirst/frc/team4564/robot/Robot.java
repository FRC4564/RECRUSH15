
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
    Xbox joyTote = new Xbox(0);
    Xbox joyBin = new Xbox(1);
    Compressor comp = new Compressor();
    Lift lift = new Lift();
    Claw claw = new Claw();
    Auto auto = new Auto(dt, lift, claw);

    public Robot() {
    	Common.debug("Constructing drive train");
        dt = new DriveTrain();
        dt.setSafetyEnabled(false);  //Safety will be enabled for TeleOp
        dt.setExpiration(0.1);    
    }
    
    // ROBOTINIT
    // Runs once, after robot is booted
    public void robotInit() {
    	Common.debug("Starting: robotInit()");
    	comp.stop();
        dt.init();
    }
    
    // DISABLED MODE
    // Robot is disabled.  Allow selection of Auto play mode.
    public void disabled() {
    	Common.debug("Starting: disabled mode");
        while (isEnabled() == false) {
        	if (joyTote.whenDpadUp()) {
        		Common.debug("Incrementing play number");
        		auto.nextPlay();
        	}
        	
        	if (joyTote.whenDpadDown()) {
        		Common.debug("Decrementing play number");
        		 auto.prevPlay();
        	}
//        	SmartDashboard.putNumber("Play #", auto.getPlay());
        	Timer.delay(.1);
        } 
        Common.debug("Ending: disabled mode");
    }
    

    // AUTONOMOUS MODE
    public void autonomous() {
        dt.setSafetyEnabled(false);
//        Auto auto = new Auto(dt, lift, claw);
//        auto.run();
    }


    // TELEOP MODE
    public void operatorControl() {
    	Common.debug("Starting: teleop");
        dt.setSafetyEnabled(true);  
        while (isOperatorControl() && isEnabled()) {
        	
        	// DRIVE TRAIN
        	if (joyBin.leftY() == 0 && joyBin.leftX() == 0 && joyBin.rightTrigger() == 0) {
	       		dt.hDrive(joyTote.leftY(), joyTote.leftX(), joyTote.rightTrigger() - joyTote.leftTrigger());	// Drive with arcade style using left stick by default
	
        	} if (joyTote.leftY() == 0 && joyTote.leftX() == 0 && joyTote.rightX() == 0) {
	       		dt.hDrive(joyBin.leftY(), joyBin.leftX(), joyBin.rightTrigger() - joyBin.leftTrigger());

        	// NOTHING

	        } if (joyBin.whenRightClick()) {        		
        		//Nothing!
	        	
	        } if (joyBin.whenLeftClick()) {
	        	//Nothing
	        	
	        } if (joyTote.whenRightClick()) {        	
        		//Nothing!
	        	
	        } if (joyTote.whenLeftClick()) {
	        	//Nothing!
        	
	        } if (joyTote.rightBumper()) {			
    			//Nothing!
	        	
    		} if (joyTote.leftBumper()) {
    			//Nothing!
    		}

        	// LIFT
    		
	    	if (joyTote.whenA()) {	// Lift up
	    		lift.levelDown();
	    	
	    	} if (joyTote.whenB()) {
	    		lift.levelUp();
	    		
	    	} if (lift.isIdle() || lift.isMoving()) {		// Move lift freely, if ready for movement
        		if (joyTote.rightY() !=0 ) {
        			lift.moveFree(joyTote.rightY());
        		}
        	}

        	if (lift.isIdle() || lift.isMoving()) {		// Move lift freely, if ready for movement
        		lift.moveFree(joyBin.rightY());
        	}
        	
        	if (joyTote.whenStart()) {					
        		Common.debug("Initializing Lift and Claw");
        		lift.init();
        		claw.init();
        	}
        	
        	if (joyBin.whenStart()) {					
        		Common.debug("Initializing Lift and Claw");
        		lift.init();
        		claw.init();
        	}
        	
        	// COMPRESSOR
        	if (joyTote.whenSelect()) {					// Toggle compressor
        		if (comp.enabled() == true) {
        			comp.stop();
        		} if (comp.enabled() == false) {
        			comp.start();
        		}	
        	}
        	
        	if (joyBin.whenSelect()) {					// Toggle compressor
        		if (comp.enabled() == true) {
        			comp.stop();
        		} if (comp.enabled() == false) {
        			comp.start();
        		}	
        	}
        	
        	// CARRIAGE
        	 if (joyBin.leftBumper()) {					
         		// Carriage up
        	 } if (joyBin.leftTrigger() > .5) {
        		 // Carriage down
        	
        	// FOREBAR
        	if (joyBin.rightBumper()) {						// Forebar is tri-state: Up, Down or Stopped
        		claw.forebarUp();
        	} if (joyBin.rightTrigger() > .5) {
        		claw.forbarDown();
        	} else {
        		claw.forebarStop();

        	// MAST
        	if (joyBin.whenY()) {
        		claw.mastToggle();        	
        	} 
        	
        	//CLAW
 	        } if (joyBin.whenDpadRight()) {		
 	        	//Claw rotate
 	    		
 	        } if (joyBin.whenDpadLeft())
 	        	//Claw rotate
 	        	
        	// UPDATE SUBSYSTEMS
        	lift.update();
        	claw.update();

            Timer.delay(1.0 / Constants.REFRESH_RATE);		// wait before repeating main update loop
        }
        }
        Common.debug("Ending: teleop");
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}