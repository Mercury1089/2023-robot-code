// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SWERVE;
import frc.robot.util.MercMath;

public class Drivetrain extends SubsystemBase {

  private SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
  private PigeonIMU pigeon;
  private SwerveDriveOdometry odometry;
  private SwerveDriveKinematics swerveKinematics;
  
  private final double WHEEL_WIDTH = 26.5; // distance between front/back wheels (in inches)
  private final double WHEEL_LENGTH = 26.5; // distance between left/right wheels (in inches)


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // configure swerve modules
    frontLeftModule = new SwerveModule(0, 0, 0);
    frontRightModule = new SwerveModule(0, 0, 90);
    backLeftModule = new SwerveModule(0, 0, 180);
    backRightModule = new SwerveModule(0, 0, 270);

    //configure gyro
    pigeon = new PigeonIMU(0);
    pigeon.configFactoryDefault();
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5);

    // wpilib convienence classes
    /*
     * swerve modules relative to robot center --> kinematics object --> odometry object 
     */

    double widthFromCenter = Units.inchesToMeters(WHEEL_WIDTH) / 2;
    double lengthFromCenter = Units.inchesToMeters(WHEEL_LENGTH) / 2;

    swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(lengthFromCenter, widthFromCenter),
      new Translation2d(lengthFromCenter, -widthFromCenter),
      new Translation2d(-lengthFromCenter, widthFromCenter),
      new Translation2d(-lengthFromCenter, -widthFromCenter)
    );

    odometry = new SwerveDriveOdometry(
      swerveKinematics, 
      Rotation2d.fromDegrees(MercMath.pigeonUnitsToDegrees(pigeon.getYaw())),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
      });
  }

  public void resetYaw() {
    pigeon.setYaw(0);
  }

  public void calibratePigeon() {
    pigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.BootTareGyroAccel);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(pigeon.getYaw()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        pose);
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  public void joyDrive(double xSpeed, double ySpeed, double angularSpeed) {
    xSpeed *= SWERVE.MAX_DIRECTION_SPEED;
    ySpeed *= SWERVE.MAX_DIRECTION_SPEED;
    angularSpeed *= SWERVE.MAX_ROTATIONAL_SPEED;

    // general swerve speeds --> speed per module
    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, angularSpeed));
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    frontLeftModule.setDesiredState(frontLeft);
    frontRightModule.setDesiredState(frontRight);
    backLeftModule.setDesiredState(backLeft);
    backRightModule.setDesiredState(backRight);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
    Rotation2d.fromDegrees(MercMath.pigeonUnitsToDegrees(pigeon.getYaw())),
    new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(),
      backRightModule.getPosition()
    });
  }


}