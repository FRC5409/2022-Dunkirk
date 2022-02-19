package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.kDriveTrain;

public final class Convert {
    
    public static double EncoderUnitsToInches(float encoderUnits){
        if(SmartDashboard.getBoolean("LowGear", true)){
            return encoderUnits / kDriveTrain.encoderCPR * kDriveTrain.lowGearConversionFactor * kDriveTrain.wheelCircumferenceInches;
        }
        else{
            return encoderUnits / kDriveTrain.encoderCPR * kDriveTrain.highGearConversionFactor * kDriveTrain.wheelCircumferenceInches;
        }
    }

    public static double InchesToEncodeUnits(double inches){
        if(SmartDashboard.getBoolean("LowGear", true)){
            return inches * kDriveTrain.encoderCPR / kDriveTrain.lowGearConversionFactor / kDriveTrain.wheelCircumferenceInches;
        }
        else{
            return inches * kDriveTrain.encoderCPR / kDriveTrain.highGearConversionFactor / kDriveTrain.wheelCircumferenceInches;
        }
    }
}
