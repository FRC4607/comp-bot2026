package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class DrivetrainConstants {
        
    }

    public class ClimberConstants {

        /** CAN ID of the first outer chain motor. */
        public static final int kOuterMotor1CANID = 6;

        /** Can ID of the second outer chain motor. */
        public static final int kOuterMotor2CANID = 23;

        /** CAN ID of the first inner chain motor. */
        public static final int kInnerMotor1CANID = 34;

        /** CAN ID of the second inner chain motor. */
        public static final int kInnerMotor2CANID = 15;
    }
}
