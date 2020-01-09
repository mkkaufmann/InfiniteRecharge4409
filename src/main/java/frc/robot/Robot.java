/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.util.commands.CommandScheduler;

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

  DriveTrain driveTrain = new DriveTrain();
  Joystick leftJS = new Joystick(0);
  Joystick rightJS = new Joystick(1);
  

  @Override
  public void robotInit() {
         
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
	  CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.drive(leftJS.getY(), rightJS.getY());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
