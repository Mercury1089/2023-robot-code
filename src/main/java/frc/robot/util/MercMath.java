package frc.robot.util;

import frc.robot.Constants.UNITS;
import frc.robot.subsystems.arm.Telescope;

public class MercMath {
    public static double pigeonUnitsToDegrees(double pigeonUnits) {
        return pigeonUnits * 360 / UNITS.PIGEON_NATIVE_UNITS_PER_ROTATION;
    }

    public static double degreesToPigeonUnits(double degrees) {
        return UNITS.PIGEON_NATIVE_UNITS_PER_ROTATION * degrees / 360;
    }

    public static double degreesToEncoderTicks(double degrees) {
        return UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION * degrees / 360;
    }

    public static double encoderTicksToDegrees(double ticks) {
        return ticks * 360 / UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION;
    }

    public static double inchesToEncoderTicks(double inches) {
        return inches / (Math.PI * Telescope.SPROCKET_DIAMETER_INCHES) *
                (UNITS.MAG_ENCODER_TICKS_PER_REVOLUTION);
    }

}
