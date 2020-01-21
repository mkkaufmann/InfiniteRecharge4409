/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.util.commands.CommandScheduler;
import frc.robot.util.commands.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  //DriveTrain driveTrain = new DriveTrain();
  XboxController controller = new XboxController(0); 
  RamseteCommand traj;
  ColorSensor colorSensor = new ColorSensor();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

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
    //driveTrain.zeroSensors();
    ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
    Rotation2d startRot = new Rotation2d(0);
    Rotation2d endRot = new Rotation2d(0);
    Pose2d startPose = new Pose2d(0,0,startRot);
    Pose2d endPose = new Pose2d(2.3938,0,endRot);
	  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, waypoints, endPose, new TrajectoryConfig(.25,.25));
    //traj = driveTrain.getTrajectory(trajectory);
    //traj.schedule();
  }

  @Override
  public void teleopPeriodic() {
    //driveTrain.updateOdometry();
    CommandScheduler.getInstance().run();
    
    if(controller.getBButtonPressed()){
      colorSensor.printColors();
    }
    //read values periodically
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);

  //post to smart dashboard periodically
  SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
    
    //driveTrain.printEncoderValues();
    //driveTrain.drive(-controller.getY(Hand.kLeft), -controller.getY(Hand.kRight));
    //driveTrain.driveVelocityMetersPerSecond(1.0, 1.0);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
