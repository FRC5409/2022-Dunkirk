package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.kDriveTrain;

public final class Convert {
    
    public static double EncoderUnitsToInches(double encoderUnits){
        if(SmartDashboard.getBoolean("LowGear", true)){
            return encoderUnits / kDriveTrain.encoderCPR * kDriveTrain.lowGearConversionFactor * kDriveTrain.wheelCircumferenceInches;
        }
        else{
            return encoderUnits / kDriveTrain.encoderCPR * kDriveTrain.highGearConversionFactor * kDriveTrain.wheelCircumferenceInches;
        }
    }

    public static double InchesToEncodeUnits(double inches){
        System.out.println(inches);
        if(SmartDashboard.getBoolean("LowGear", true)){
            return inches * kDriveTrain.encoderCPR / kDriveTrain.lowGearConversionFactor / kDriveTrain.wheelCircumferenceInches;
        }
        else{
            return inches * kDriveTrain.encoderCPR / kDriveTrain.highGearConversionFactor / kDriveTrain.wheelCircumferenceInches;
        }
    }
}
