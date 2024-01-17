// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Telescope.TelescopePosition;
import frc.robot.util.MercMath;

public class MoveTelescope extends Command {
  /** Creates a new MoveTelescope. */
  Telescope scope;
  TelescopePosition scopePos;
  public final double THRESHOLD_INCHES = 2;
  public MoveTelescope(Telescope scope, TelescopePosition scopePos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scope = scope;
    this.scopePos = scopePos;
    addRequirements(scope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scope.setPosition(scopePos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return scope.getError() < MercMath.inchesToEncoderTicks(THRESHOLD_INCHES);
  }
}
