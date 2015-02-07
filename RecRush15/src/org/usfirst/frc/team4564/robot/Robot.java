
package org.usfirst.frc.team4564.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class.
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
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
    DriveTrain dt;
    Xbox joyLift = new Xbox(0);
    Xbox joyClaw = new Xbox(1);
    Talon fL = new Talon(Constants.PWM_DRIVE_FL);
    Talon rL = new Talon(Constants.PWM_DRIVE_RL);
    Talon fR = new Talon(Constants.PWM_DRIVE_FR);
    Talon rR = new Talon(Constants.PWM_DRIVE_RR);
    Victor fC = new Victor(Constants.PWM_DRIVE_FC);
    Victor rC = new Victor(Constants.PWM_DRIVE_RC);  
    Lift lift = new Lift();
    Claw claw = new Claw();
    Auto auto = new Auto(dt, lift, claw);
    //double prefTest;
    //Preferences prefs;
    //Command autoCommand;
    //SendableChooser autoChooser;
    
    int autoMode=0;
    
    // Pneumatics Code In Progress
    Compressor comp = new Compressor();

    public Robot() {
        dt = new DriveTrain(fL, rL, fR, rR, fC, rC);
        dt.setExpiration(0.1);    
    }
    
    public void robotInit() {
    	comp.stop();
    	//autoChooser = new SendableChooser();
    	//autoChooser.addDefault("RotateLeft", "Left");
    	//autoChooser.addObject("RotateRight", "Right");
    	//prefTest = prefs.getDouble("VelocityKp", 1.0);
		//SmartDashboard.putNumber("Prefs Velocity Kp", prefTest);	
    }
    
    // Robot is disabled.  Allow adjust of settings.
    public void disabled() {
        while (isEnabled() == false) {
        	if (joyLift.whenDpadUp()) {
        		auto.nextPlay();
        	}
        	
        	if (joyLift.whenDpadDown()) {
        		 auto.prevPlay();
        	}
        	SmartDashboard.putNumber("Play #", auto.getPlay());
        	Timer.delay(Constants.REFRESH_RATE);
        }
    }

    //Drive left & right motors for 2 seconds then stop
    public void autonomous() {
        dt.setSafetyEnabled(false);
        //dt.init();
        Auto auto = new Auto(dt, lift, claw);
        auto.run();
        //if (autoChooser.getSelected() == "Left") {
        //	dt.rotateLeft90();
        //} else {
        //	dt.rotateRight90();
        //}
        Timer.delay(2.0);		//    for 2 seconds
    }

    //Runs the motors with arcade steering.
    public void operatorControl() {
        dt.setSafetyEnabled(true);  
        dt.init();
        Timer.delay(1);
        comp.start();
        while (isOperatorControl() && isEnabled()) {
        	if (joyLift.A()) {							    // Left thumbstick click to do Translate drive
        		dt.translateDrive(joyLift.leftY(), joyLift.leftX());
        	} else {
        		dt.hDrive(joyLift.leftY(), joyLift.leftX(), 0);               // Drive with arcade style using left stick by default
        	}

        	if (joyLift.leftBumper()) {									// Left bumper to rotate Left
        		dt.rotateLeft90();
            } else if (joyLift.rightBumper()) {							// Right bumper to rotate right
            	dt.rotateRight90();
        	}
        	// Test lift homing function
        	if(joyLift.whenStart()) {									// Start button to Initilize to home
        		lift.init();
        	}
        	
        	// previously lift.levelGo(3);, using for moveFree test
        	if (lift.isIdle() || lift.isMoving()) {
        		lift.moveFree(joyLift.rightY());
        	}
        	
        	if(joyLift.X()) {									// X-button to go to 25"
        		lift.gotoHeight(25);
        	}
        	
        	if (joyLift.whenA()) {
        		if (comp.enabled() == true) {
        			comp.stop();
        		} if (comp.enabled() == false) {
        			comp.start();
        		}	
        	}

        	lift.update();
        	/**if(stick.getRawButton(10))
        	{
        		valve1.set(true);
        	}
        	else
        	{
        		valve1.set(false);
        	}
        	
        	if(stick.getRawButton(9))
        	{
        		valve2.set(true);
        	}
        	else
        	{
        		valve2.set(false);
        	}**/
        	
        	//if(valveswitch.get()== true) 
        	//{
        	//	valve1.set(true);
        	//	valve2.set(false);
        	//}
        	//else {
        	//	valve1.set(false);
        	//	valve2.set(true);
        	//	
        	//}
       
            Timer.delay(1.0 / Constants.REFRESH_RATE);		// wait before repeating main update loop
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}