// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.DoubleBinaryOperator;

/** Add your docs here. */
public class UserUnits {

    private static final double gearRatio = 28.31731;
    private static final double sprocketT = 15;
    private static final double chainPitch = .375;

    public static double getClimberMotorRev2Inches(double motorRevs){
        return motorRevs * getConvertionValue();
    }

    public static double getClimberInches2MotorRevs(double inches){
        return inches / getConvertionValue();
    }

    private static double getConvertionValue(){
        return (sprocketT  *chainPitch  /gearRatio);
    }

}
