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

  public enum autonRoutine{
    DRIVE_OFF_LINE,
    DRIVE_OFF_LINE_AND_SHOOT
  }
  public Command getAutonomousCommand(Drivetrain drive, autonRoutine routine) {
    TrajectoryConfig config = new TrajectoryConfig(2, 2);
    config.setKinematics(drive.getKinematics());

    Trajectory trajectory;
    switch(routine){
      case DRIVE_OFF_LINE:
        trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())),
        config
      );
        break;
      case DRIVE_OFF_LINE_AND_SHOOT:
        trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(7.763067665761969,-7.60015061618154, new Rotation2d()), new Pose2d(10.442660926842251,-5.948070560477909, new Rotation2d())),
          config
        );
        break;
        default:
        trajectory = TrajectoryGenerator.generateTrajectory(
          Arrays.asList(new Pose2d(), new Pose2d(0, 0, new Rotation2d())),
          config
        );
    }
    



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

