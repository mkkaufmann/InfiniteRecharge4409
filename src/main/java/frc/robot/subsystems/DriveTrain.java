package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveTrain {
    TalonSRX leftFront = new TalonSRX(0);        
    TalonSRX rightFront = new TalonSRX(1);
    VictorSPX leftRear = new VictorSPX(2);
    VictorSPX rightRear = new VictorSPX(3);

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

}