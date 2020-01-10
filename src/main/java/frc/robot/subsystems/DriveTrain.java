package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.util.NavX;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.util.commands.RamseteCommand;

public class DriveTrain {

    //constants
    static final int ENCODER_TICKS_PER_REVOLUTION = 4096;
    static final double WHEEL_DIAMETER_INCHES = 6;

    //motors
    TalonSRX leftMaster = new TalonSRX(0);        
    TalonSRX rightMaster = new TalonSRX(1);
    VictorSPX leftSlave = new VictorSPX(2);
    VictorSPX rightSlave = new VictorSPX(3);

    //sensors
    NavX navx = new NavX();

    //odometry, etc for autonomous
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(1.0);
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), new Pose2d());
    RamseteController ramseteController = new RamseteController();
    
    //todo add encoders
    //add pid
    public DriveTrain(){
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();

	//one encoder may be backwards
	leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
	rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    public void drive(double left, double right){
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public void driveTrajectory(Trajectory trajectory){
	RamseteCommand command = new RamseteCommand(trajectory, odometry::getPoseMeters, ramseteController, kinematics, this::driveVelocityMetersPerSecond);
	command.schedule();
    }

    //make m/s 
    private void driveVelocityMetersPerSecond(Double left, Double right){
	    leftMaster.set(ControlMode.Velocity, left);
	    rightMaster.set(ControlMode.Velocity, right);
    }

    public void updateOdometry(){
	//todo: add encoder values here
       	odometry.update(new Rotation2d(navx.getYawRadians()), 0, 0);
    }

    private double encoderTicksToRevolutions(int ticks){
        return 0;
    }

    private double encoderTicksToInches(int ticks){
        return 0;
    }
}
