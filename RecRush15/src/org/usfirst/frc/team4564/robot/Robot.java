
package org.usfirst.frc.team4564.robot;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Solenoid;
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
    Joystick stick;
    Talon fL = new Talon(0);
    Talon rL = new Talon(1);
    Talon fR = new Talon(2);
    Talon rR = new Talon(3);
    Victor fC = new Victor(4);
    Victor rC = new Victor(5);  
    Lift lift = new Lift();
    
// Jacob's Pneumatics Code
     
  //  Compressor comp = new Compressor();
  //  Solenoid valve1 = new Solenoid(0);
  //  Solenoid valve2 = new Solenoid(1);
  //  DigitalInput valveswitch = new DigitalInput(0);

    public Robot() {
        dt = new DriveTrain(fL, rL, fR, rR, fC, rC);
        dt.setExpiration(0.1);
        stick = new Joystick(0);
    }
   // public void robotInit() {
   //     compressor.start();
   // }
    
    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
        dt.setSafetyEnabled(false);

        Timer.delay(2.0);		//    for 2 seconds
    }

   /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        dt.setSafetyEnabled(true);
        dt.init();
        // compressor control
        //comp.setClosedLoopControl(true);
        while (isOperatorControl() && isEnabled()) {
        	if (stick.getRawButton(2)){
        		dt.arcadeDriver(stick.getY(), stick.getX()); // drive with arcade style (use right stick)
        	} else if (stick.getRawButton(3)) {
        		dt.translateDrive(stick.getY(), stick.getX());
        	} else {
        		dt.hDrive(stick.getY(), stick.getX(), 0);
        	}
        	
        	if (stick.getRawButton(4)){
        		dt.rotateLeft();
            } else if (stick.getRawButton(5)){
            	dt.rotateRight();
        	}
        	// Test lift homing function
        	if(stick.getRawButton(6)) {
        		lift.init();
        	}
        	
        	
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
       
            Timer.delay(0.01);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    }
}
