package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;

public class Constants {

    public class DrivetrainConstants {

        public static final double kCoupleRatio = 3.8507487;

        public static final double kDriveGearRatio = 6.03;
        public static final double kSteerGearRatio = 26.09;

            // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("kachow", "./logs/example.hoot");

        public static final int kPigeonId = 0;

        // Front Left
        public static final int kFrontLeftDriveMotorId = 0;
        public static final int kFrontLeftSteerMotorId = 2;
        public static final int kFrontLeftEncoderId = 1;

        public static final Distance kFrontLeftXPos = Inches.of(10.375);
        public static final Distance kFrontLeftYPos = Inches.of(10.375);

        // Front Right
        public static final int kFrontRightDriveMotorId = 19;
        public static final int kFrontRightSteerMotorId = 17;
        public static final int kFrontRightEncoderId = 0;

        public static final Distance kFrontRightXPos = Inches.of(10.375);
        public static final Distance kFrontRightYPos = Inches.of(-10.375);
    
        // Back Left
        public static final int kBackLeftDriveMotorId = 1;
        public static final int kBackLeftSteerMotorId = 3;
        public static final int kBackLeftEncoderId = 2;

        public static final Distance kBackLeftXPos = Inches.of(-10.375);
        public static final Distance kBackLeftYPos = Inches.of(10.375);
    
        // Back Right
        public static final int kBackRightDriveMotorId = 18;
        public static final int kBackRightSteerMotorId = 16;
        public static final int kBackRightEncoderId = 3;

        public static final Distance kBackRightXPos = Inches.of(-10.375);
        public static final Distance kBackRightYPos = Inches.of(-10.375);
    }
}
