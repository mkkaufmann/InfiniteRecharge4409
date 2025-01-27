/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;

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


   Flywheel flywheel = new Flywheel();
   Limelight limelight = new Limelight();
   Drivetrain drivetrain = new Drivetrain();
   XboxController controller = new XboxController(0);

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

    //drivetrain.tankDrive(controller.getY(Hand.kLeft), controller.getY(Hand.kRight));

    limelight.runLimelight();
    //limelight.getLLdistance();
    if(controller.getBButton())
      limelight.autoAim(drivetrain, controller.getY(Hand.kLeft));
      else
        drivetrain.arcadeDrive(controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
