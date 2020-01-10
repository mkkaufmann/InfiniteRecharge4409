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
    static final double WHEEL_DIAMETER_INCHES = 6.0;
    static final double INCHES_PER_METER = 39.37;

    //motors
    TalonSRX leftMaster = new TalonSRX(3);        
    TalonSRX rightMaster = new TalonSRX(1);
    VictorSPX leftSlave = new VictorSPX(4);
    VictorSPX rightSlave = new VictorSPX(0);

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

        leftMaster.config_kP(0,0.2);
        rightMaster.config_kP(0,0.2);
        leftMaster.config_kI(0,0);
        rightMaster.config_kI(0,0);
        leftMaster.config_kD(0,0);
        rightMaster.config_kD(0,0);
        leftMaster.config_kF(0,0.4625);
        rightMaster.config_kF(0,0.4625);

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

    private void driveVelocityMetersPerSecond(Double left, Double right){
	    leftMaster.set(ControlMode.Velocity, inchesToTicks(left * INCHES_PER_METER));
	    rightMaster.set(ControlMode.Velocity, inchesToTicks(right * INCHES_PER_METER));
    }

    public void updateOdometry(){
	//todo: add encoder values here
       	odometry.update(
			new Rotation2d(navx.getYawRadians()), 
			encoderTicksToInches(leftMaster.getSelectedSensorPosition())/INCHES_PER_METER, 
			encoderTicksToInches(rightMaster.getSelectedSensorPosition())/INCHES_PER_METER
			);
    }

    private double encoderTicksToRevolutions(int ticks){
        return ticks / ENCODER_TICKS_PER_REVOLUTION;
    }

    private double revolutionsToInches(double revolutions){
        return revolutions * Math.PI * WHEEL_DIAMETER_INCHES;
    }

    private double encoderTicksToInches(int ticks){
        return revolutionsToInches(encoderTicksToRevolutions(ticks));
    }

    private double inchesToRevolutions(double inches){
	return inches / (Math.PI * WHEEL_DIAMETER_INCHES);
    }
    
    private double revolutionsToTicks(double revolutions){
	return revolutions * ENCODER_TICKS_PER_REVOLUTION;
    }

    private double inchesToTicks(double inches){
	return revolutionsToTicks(inchesToRevolutions(inches)); 
    }
}
