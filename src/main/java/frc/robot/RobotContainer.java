package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.*;
import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainer {
  
  private Drivetrain drive = new Drivetrain();

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    config.setKinematics(drive.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
      config
    );

    RamseteCommand command = new RamseteCommand(
      trajectory,
      drive::getPose,
      new RamseteController(),
      drive.getFeedForward(),
      drive.getKinematics(),
      drive::getSpeeds,
      drive.getleftPIDController(),
      drive.getrightPIDController(),
      drive::setOutput,
      drive
    );
    return command;
  }
};
