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
import frc.robot.util.commands.RamseteCommand;

public class DriveTrain {
    //motors
    TalonSRX leftFront = new TalonSRX(0);        
    TalonSRX rightFront = new TalonSRX(1);
    VictorSPX leftRear = new VictorSPX(2);
    VictorSPX rightRear = new VictorSPX(3);

    //sensors
    NavX navx = new NavX();

    //odometry, etc for autonomous
    DifferentialDriveKinematics kinematics 
	    = new DifferentialDriveKinematics(1.0);
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
    RamseteController ramseteController = new RamseteController();
    
    //todo add encoders
    //add pid
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
	RamseteCommand command = new RamseteCommand(trajectory, odometry::getPoseMeters, ramseteController, kinematics, this::driveVelocityMetersPerSecond);
	//make the command run
    }

    //make m/s 
    private void driveVelocityMetersPerSecond(Double left, Double right){
	leftFront.set(ControlMode.Velocity, left);
	leftFront.set(ControlMode.Velocity, right);
    }

    public void updateOdometry(){
	//todo: add encoder values here
       	odometry.update(new Rotation2d(navx.getYawRadians()), 0, 0);
    }

}
