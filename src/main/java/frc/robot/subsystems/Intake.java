package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    VictorSPX intake = new VictorSPX(9);
    DoubleSolenoid pusherOuter = new DoubleSolenoid(3, 4);
    boolean isOut = false;
    public Intake() {

    }

    public void intake() {
        intake.set(ControlMode.PercentOutput, 0.75);
    }
    public void outtake() {
        intake.set(ControlMode.PercentOutput, -0.75);
    }
    public void stop() {
        intake.set(ControlMode.PercentOutput, 0);
    }
    public void deploy() {
        pusherOuter.set(DoubleSolenoid.Value.kForward);
    }
    public void reset() {
        pusherOuter.set(DoubleSolenoid.Value.kReverse);
    }
}