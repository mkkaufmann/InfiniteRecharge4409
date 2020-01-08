package frc.robot.util;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavX extends AHRS{

	public NavX(){
		super(SPI.Port.kMXP);	
	}

	public double getYawRadians(){
		return getYaw() / 180 * Math.PI;		
	}
}
