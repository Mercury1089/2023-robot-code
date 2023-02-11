// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Absolute (X, Y) of certain field locations
 * (thanks design :)
 */
public class KnownLocations {

    // auton starts
    public final Pose2d 
        START_TOPMOST,
        START_TOP_SECOND,
        START_BOTTOM_SECOND,
        START_BOTTOMMOST;
    
    // elements
    public final Pose2d
        ELEMENT1,
        ELEMENT2,
        ELEMENT3,
        ELEMENT4;
    
    //charging station
    public final Pose2d
        CHARGING_CENTER,
        CHARGING_TOP_LEFT,
        CHARGING_TOP_RIGHT,
        CHARGING_BOTTOM_LEFT,
        CHARGING_BOTTOM_RIGHT;
    
        
    public static Alliance allianceColor = DriverStation.getAlliance();

    public KnownLocations() {
        // ToDo: configure theta values
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
        
        if (allianceColor == Alliance.Blue) {
            START_TOPMOST = Pose2dInch(54.93, 199.65, 0);
            START_TOP_SECOND = Pose2dInch(54.93, 173.52, 0);
            START_BOTTOM_SECOND = Pose2dInch(54.93, 41.67, 0);
            START_BOTTOMMOST = Pose2dInch(54.93, 16.15, 0);

            ELEMENT1 = Pose2dInch(279.31, 180.02, 0);
            ELEMENT2 = Pose2dInch(279.31, 132.02, 0);
            ELEMENT3 = Pose2dInch(279.31, 84.02, 0);
            ELEMENT4 = Pose2dInch(279.31, 36.02, 0);

            CHARGING_CENTER = Pose2dInch(513.93, 107.85, 0);
            CHARGING_TOP_LEFT=  Pose2dInch(117.16, 155.51, 0);
            CHARGING_TOP_RIGHT = Pose2dInch(190.96, 155.51, 0);
            CHARGING_BOTTOM_LEFT = Pose2dInch(117.16, 60.2, 0);
            CHARGING_BOTTOM_RIGHT = Pose2dInch(190.96, 60.2, 0);

        } else {
            START_TOPMOST = Pose2dInch(598.41, 199.65, 0);
            START_TOP_SECOND = Pose2dInch(598.41, 173.52, 0);
            START_BOTTOM_SECOND = Pose2dInch(598.41, 41.67, 0);
            START_BOTTOMMOST = Pose2dInch(598.41, 16.15, 0);

            ELEMENT1 = Pose2dInch(374.03, 180.02, 0);
            ELEMENT2 = Pose2dInch(374.03, 132.02, 0);
            ELEMENT3 = Pose2dInch(374.03, 84.02, 0);
            ELEMENT4 = Pose2dInch(374.03, 36.02, 0);

            CHARGING_CENTER = Pose2dInch(499.41, 107.85, 0);
            CHARGING_TOP_LEFT = Pose2dInch(462.38, 155.51, 0);
            CHARGING_TOP_RIGHT = Pose2dInch(536.18, 155.51, 0);
            CHARGING_BOTTOM_LEFT = Pose2dInch(462.38, 60.2, 0);
            CHARGING_BOTTOM_RIGHT = Pose2dInch(536.18, 60.2, 0);
        }
    }

    /** Convenience method to create Pose2d from inches */
    private Pose2d Pose2dInch(double xInches, double yInches, double thetaDegrees) {
        return new Pose2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches), Rotation2d.fromDegrees(thetaDegrees));
    }
}
