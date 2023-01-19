// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SwerveOnJoysticks extends CommandBase {
  Drivetrain drivetrain;
  CommandJoystick leftJoy, rightJoy;
  double xSpeed, ySpeed, angularSpeed;
  /** Creates a new SwerveOnJoysticks. */
  public SwerveOnJoysticks(Drivetrain drivetrain, CommandJoystick leftJoy, CommandJoystick rightJoy) {
    // Use addRequirements() here to declare subsystem dependencies.
    setName("SwerveOnJoysticks");
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.leftJoy = leftJoy;
    this.rightJoy = rightJoy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 0 if in deadband
    // (speed - deadband) / (1 - deadband)
    xSpeed = MathUtil.applyDeadband(leftJoy.getX(), SWERVE.JOYSTICK_DEADBAND);
    ySpeed = MathUtil.applyDeadband(leftJoy.getY(), SWERVE.JOYSTICK_DEADBAND);
    angularSpeed = MathUtil.applyDeadband(rightJoy.getX(), SWERVE.JOYSTICK_DEADBAND);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    this.drivetrain.joyDrive(xSpeed, ySpeed, angularSpeed);

    xSpeed = MathUtil.applyDeadband(leftJoy.getX(), SWERVE.JOYSTICK_DEADBAND);
    ySpeed = MathUtil.applyDeadband(leftJoy.getY(), SWERVE.JOYSTICK_DEADBAND);
    angularSpeed = MathUtil.applyDeadband(rightJoy.getX(), SWERVE.JOYSTICK_DEADBAND);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}