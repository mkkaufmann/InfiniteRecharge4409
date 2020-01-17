package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {

    ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private int red;
    private int blue;
    private int green;

    public void printColors(){
        red = colorSensor.getRed();
        green = colorSensor.getGreen();
        blue = colorSensor.getBlue();
        
        System.out.println("Red: " + red + " Green: " + green + " Blue: " + blue);
    }

}