
package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
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
    Xbox joyLift = new Xbox(0);
    Xbox joyClaw = new Xbox(1);
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
        	if (joyLift.whenDpadUp()) {
        		Common.debug("Incrementing play number");
        		auto.nextPlay();
        	}
        	
        	if (joyLift.whenDpadDown()) {
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
       		dt.hDrive(joyLift.leftY(), joyLift.leftX(), joyLift.rightX());	// Drive with arcade style using left stick by default

        	if (joyLift.leftBumper()) {					// Left bumper to rotate Left
        		dt.rotateLeft90();
            } else if (joyLift.rightBumper()) {			// Right bumper to rotate right
            	dt.rotateRight90();
        	}
        	
        	// LIFT
        	if (joyLift.whenStart()) {					// Start button to initilize lift and claw
        		Common.debug("Initializing Lift and Claw");
        		lift.init();
        		claw.init();
        	}
        	
        	if (joyLift.whenDpadUp()) {					// Level up
        		lift.levelUp();
        	}
        	
        	if (joyLift.whenDpadDown()) {				// Level down
        		lift.levelDown();
        	}
        	
        	if (joyLift.whenRightClick()) {        		// Release the tote
        		lift.releaseTote();
        	}
        	
        	if (lift.isIdle() || lift.isMoving()) {		// Move lift freely, if ready for movement
        		lift.moveFree(joyLift.rightY());
        	}
        	
        	// COMPRESSOR
        	if (joyLift.whenSelect()) {					// Toggle compressor
        		if (comp.enabled() == true) {
        			comp.stop();
        		} if (comp.enabled() == false) {
        			comp.start();
        		}	
        	}
        	
        	// FOREBAR
        	if (joyLift.X()) {						// Forebar is tri-state: Up, Down or Stopped
        		claw.forebarUp();
        	} else if (joyLift.Y()) {
        		claw.forbarDown();
        	} else {
        		claw.forebarStop();
        	}
        	
        	// MAST
        	if (joyLift.whenDpadLeft()) {
        			claw.mastToggle();        		
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
    }
}
