package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hopper extends SubsystemBase {
    VictorSPX hopper = new VictorSPX(10);
    public Hopper() {

    }

    public void feed(double feedSpeed) {
        hopper.set(ControlMode.PercentOutput, feedSpeed);
    }
    public void stop() {
        hopper.set(ControlMode.PercentOutput, 0);
    }
}
