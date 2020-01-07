import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveTrain {
    TalonSRX leftFront = new TalonSRX(0);        
    TalonSRX rightFront = new TalonSRX(1);
    VictorSPX leftRear = new VictorSPX(2);
    VictorSPX rightRear = new VictorSPX(3);
    
}