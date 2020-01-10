/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
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
//	  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,0), {}, new Pose2d(1,0,0), new TrajectoryConfig(0,0,0));
//	  driveTrain.driveTrajectory(trajectory);
  }

  @Override
  public void teleopPeriodic() {
	  CommandScheduler.getInstance().run();
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
