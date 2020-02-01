/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  Jaguar leftFront = new Jaguar(0);
  Jaguar rightFront = new Jaguar(2);
  Jaguar leftBack = new Jaguar(1);
  Jaguar rightBack = new Jaguar(3);

  XboxController controller = new XboxController(0);
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  boolean driveEnabled = false;
  Flywheel flywheel = new Flywheel();



  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    driveEnabled = true;
  }

  @Override
  public void teleopPeriodic() {
    double driveMult = .33;
    double turnMult = .25;
    double turnMultThreshold = .5;
    double driveMultThreshold = .5;

    //Double turn speed when right trigger is depressed
    if(controller.getTriggerAxis(Hand.kLeft) > turnMultThreshold)
      turnMult = .5;
    else
      turnMult = .25;

    //Double drive speed when left trigger is depressed
    if(controller.getTriggerAxis(Hand.kRight) > driveMultThreshold)
      driveMult = .66;
    else
      driveMult = .33;

    //Get joystick input and scale it
    double leftY = -controller.getY(Hand.kLeft)*driveMult;
    double rightX = controller.getX(Hand.kRight)*turnMult;

    //Drive
    if(driveEnabled){
      leftFront.set(rightX + leftY);
      rightFront.set(rightX - leftY);
      leftBack.set(rightX + leftY);
      rightBack.set(rightX - leftY);
    }


    //flywheel.runFlywheel(controller.getY(Hand.kRight));
    flywheel.runRPM(-1000);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
