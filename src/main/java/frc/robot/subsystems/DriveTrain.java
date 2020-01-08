package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.util.NavX;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.Constants;

public class DriveTrain {
    //motors
    TalonSRX leftFront = new TalonSRX(Constants.Drivetrain.leftFrontPort);        
    TalonSRX rightFront = new TalonSRX(Constants.Drivetrain.rightFrontPort);
    VictorSPX leftRear = new VictorSPX(Constants.Drivetrain.leftRearPort);
    VictorSPX rightRear = new VictorSPX(Constants.Drivetrain.rightRearPort);

    //sensors
    NavX navx = new NavX();

    //odometry, etc for autonomous
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.drivetrainWidth);
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
    RamseteController ramseteController = new RamseteController();

    public DriveTrain(){
        leftFront.configFactoryDefault();
        rightFront.configFactoryDefault();

        leftRear.follow(leftFront);
        rightRear.follow(rightFront);
    }

    public void drive(double left, double right){
        leftFront.set(ControlMode.PercentOutput, left);
        rightFront.set(ControlMode.PercentOutput, right);
    }

    public void driveTrajectory(Trajectory trajectory){
    }

    public void updateOdometry(){
	//todo: add encoder values here
       	odometry.update(new Rotation2d(navx.getYawRadians()), 0, 0);
    }

}
