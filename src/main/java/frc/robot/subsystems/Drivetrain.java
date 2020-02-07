package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Drivetrain{
    
    TalonSRX leftMaster = new TalonSRX(3);
    TalonSRX leftFollower = new TalonSRX(4);
    TalonSRX rightMaster = new TalonSRX(1);
    TalonSRX rightFollower = new TalonSRX(0);

    public Drivetrain(){
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        leftMaster.setSensorPhase(true);

        leftMaster.config_kP(0,0.0);
        rightMaster.config_kP(0,0.0);
        leftMaster.config_kI(0,0);
        rightMaster.config_kI(0,0);
        leftMaster.config_kD(0,0);
        rightMaster.config_kD(0,0);
        leftMaster.config_kF(0,0.4625);
        rightMaster.config_kF(0,0.4625);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);
    }

    public void tankDrive(double left, double right){
      leftMaster.set(ControlMode.PercentOutput, -left);
      rightMaster.set(ControlMode.PercentOutput, right);
    }

}
