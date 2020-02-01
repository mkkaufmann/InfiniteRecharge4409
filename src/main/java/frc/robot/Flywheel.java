package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


class Flywheel {

    static final int MAX_MOTOR_RPM = 18370;
    static final double GEARBOX_RATIO = 3.25;
    static final int ENCODER_TICKS_PER_MOTOR_REVOLUTION = 12;
    static final double MAX_FLYWHEEL_RPM = MAX_MOTOR_RPM/GEARBOX_RATIO;
    static final double ENCODER_TICKS_PER_FLYWHEEL_REVOLUTION = ENCODER_TICKS_PER_MOTOR_REVOLUTION * GEARBOX_RATIO;

    TalonSRX flywheelMaster = new TalonSRX(0);
    VictorSPX flywheelFollower = new VictorSPX(1);
    
    public Flywheel(){
        flywheelMaster.setInverted(true);
        flywheelFollower.setInverted(true);
        flywheelFollower.follow(flywheelMaster);
        flywheelMaster.config_kP(0, 40);
        flywheelMaster.config_kF(0, 2.78443113772);
        flywheelMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    }

    public void runFlywheel(double output){
        System.out.println("Encoder position" + flywheelMaster.getSelectedSensorPosition());
        flywheelMaster.set(ControlMode.PercentOutput, output);
    }

    public void runVelocity(double velocity){
        System.out.println("Encoder velocity" + (flywheelMaster.getSelectedSensorVelocity()));
        System.out.println("Encoder difference from velocity " + (Math.abs(velocity) - (Math.abs(flywheelMaster.getSelectedSensorVelocity()))));
        flywheelMaster.set(ControlMode.Velocity, velocity);
    }

    public void runRPM(double rpm){
        runVelocity(rpm/600*ENCODER_TICKS_PER_FLYWHEEL_REVOLUTION);
    }

}