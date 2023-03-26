// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/** Absolute (X, Y) of certain field locations
 * (thanks design :)
 */
public class KnownLocations {

    // do nothing auton
    public static final PathPoint
        DO_NOTHING = PathPointInch(0, 0, 0, 0);

    // auton starts
    public final PathPoint 
        START_TOPMOST,
        START_TOP_SECOND,
        START_BOTTOM_SECOND,
        START_BOTTOMMOST;
    
    // elements
    public final PathPoint
        ELEMENT1,
        ELEMENT2,
        ELEMENT3,
        ELEMENT4;
    
    //charging station
    public final PathPoint
        CHARGING_CENTER,
        CHARGING_TOP_LEFT,
        CHARGING_TOP_RIGHT,
        CHARGING_BOTTOM_LEFT,
        CHARGING_BOTTOM_RIGHT;

    public final PathPoint
        WAYPOINT_TOP,
        WAYPOINT_BOTTOM;
    
        
    public Alliance allianceColor = DriverStation.getAlliance();

    public KnownLocations() {

        // ToDo: configure theta values
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system
        
        allianceColor = DriverStation.getAlliance();
        if (allianceColor == Alliance.Blue) {
            START_TOPMOST = PathPointInch(54.93, 199.65, 180, 180);
            START_TOP_SECOND = PathPointInch(54.93, 173.52, 180, 180);
            START_BOTTOM_SECOND = PathPointInch(54.93, 41.67, 180, 180);
            START_BOTTOMMOST = PathPointInch(54.93, 16.15, 180, 180);

            ELEMENT1 = PathPointInch(279.31, 180.02, 180, 0);
            ELEMENT2 = PathPointInch(279.31, 132.02, 180, 0);
            ELEMENT3 = PathPointInch(279.31, 84.02, 180, 0);
            ELEMENT4 = PathPointInch(279.31, 36.02, 180, 0);

            CHARGING_CENTER = PathPointInch(153.93, 107.85, 0, 0);
            CHARGING_TOP_LEFT=  PathPointInch(117.16, 155.51, 0, 0);
            CHARGING_TOP_RIGHT = PathPointInch(190.96, 155.51, 0, 0);
            CHARGING_BOTTOM_LEFT = PathPointInch(117.16, 60.2, 0, 0);
            CHARGING_BOTTOM_RIGHT = PathPointInch(190.96, 60.2, 0, 0);

            WAYPOINT_TOP = PathPointInch(190.96, 185.62, 180, 180);
            WAYPOINT_BOTTOM = PathPointInch(190.96, 30.101, 180, 180);
        } else {
            START_TOPMOST = PathPointInch(598.41, 199.65, 0, 0);
            START_TOP_SECOND = PathPointInch(598.41, 173.52, 0, 0);
            START_BOTTOM_SECOND = PathPointInch(598.41, 41.67, 0, 0);
            START_BOTTOMMOST = PathPointInch(598.41, 16.15, 0, 0);

            ELEMENT1 = PathPointInch(374.03, 180.02, 0, 180);
            ELEMENT2 = PathPointInch(374.03, 132.02, 0, 180);
            ELEMENT3 = PathPointInch(374.03, 84.02, 0, 180);
            ELEMENT4 = PathPointInch(374.03, 36.02, 0, 180);

            CHARGING_CENTER = PathPointInch(499.41, 107.85, 0, 180);
            CHARGING_TOP_LEFT = PathPointInch(462.38, 155.51, 0, 180);
            CHARGING_TOP_RIGHT = PathPointInch(536.18, 155.51, 0, 180);
            CHARGING_BOTTOM_LEFT = PathPointInch(462.38, 60.2, 0, 180);
            CHARGING_BOTTOM_RIGHT = PathPointInch(536.18, 60.2, 0, 180);

            WAYPOINT_TOP = PathPointInch(463.39, 185.62, 0, 180);
            WAYPOINT_BOTTOM = PathPointInch(463.39, 30.1, 0, 180);
        }
    }

    /** Convenience method to create Pose2d from inches */
    private static Pose2d Pose2dInch(double xInches, double yInches, double thetaDegrees) {
        return new Pose2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches), Rotation2d.fromDegrees(thetaDegrees));
    }
    /** Convenience method to create PathPoint from inches */
    private static PathPoint PathPointInch(double xInches, double yInches, double headingDegrees, double rotationDegrees) {
        return new PathPoint(new Translation2d(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches)), Rotation2d.fromDegrees(headingDegrees), Rotation2d.fromDegrees(rotationDegrees));
    }
}
