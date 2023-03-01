// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DS_USB;
import frc.robot.Constants.JOYSTICK_BUTTONS;
import frc.robot.auton.Autons;
import frc.robot.commands.arm.ManualArm;
import frc.robot.commands.drivetrain.SwerveOnJoysticks;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.LEDState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.Wrist.WristPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; 
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private CommandJoystick rightJoystick, leftJoystick;
  private CommandXboxController gamepad;

  private Trigger left1, left2, left3, left4, left5, left6, left7, left8, left9, left10, left11;
  private Trigger right1, right2, right3, right4, right5, right6, right7, right8, right9, right10, right11;
  private Trigger gamepadA, gamepadB, gamepadX, gamepadY, gamepadRB, gamepadLB, gamepadL3, gamepadBack, 
  gamepadStart, gamepadLeftStickButton, gamepadRightStickButton, gamepadLT, gamepadRT, gamepadPOVDown, gamepadPOVUpLeft, 
  gamepadPOVUp, gamepadPOVUpRight, gamepadPOVLeft, gamepadPOVRight, gamepadPOVDownRight, gamepadPOVDownLeft;

  private Supplier<Double> gamepadLeftX, gamepadLeftY, gamepadRightX, gamepadRightY, rightJoystickX, rightJoystickY, leftJoystickX, leftJoystickY;

  private GenericEntry allianceBooleanBox;

  private Autons auton;
  private GamePieceLEDs LEDs;
  private Arm arm;
  private Telescope telescope;
  private Wrist wrist;
  private Drivetrain drivetrain;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // buttons & bindings
    leftJoystick = new CommandJoystick(DS_USB.LEFT_STICK);
    rightJoystick = new CommandJoystick(DS_USB.RIGHT_STICK);
    gamepad = new CommandXboxController(DS_USB.GAMEPAD);
    configureBindings();
    
    // subsystems & sensors
    LEDs = new GamePieceLEDs();
    
    arm = new Arm();
    telescope = new Telescope();
    wrist = new Wrist();
    arm.setDefaultCommand(new RunCommand(() -> arm.moveArm(gamepadRightY), arm));


    drivetrain = new Drivetrain();
    drivetrain.setDefaultCommand(new SwerveOnJoysticks(drivetrain, leftJoystickX, leftJoystickY, rightJoystickX));
    drivetrain.resetGyro();

    wrist.setDefaultCommand(new RunCommand(() -> wrist.moveWrist(() -> -gamepadLeftY.get()), wrist));
    gamepadA.onTrue(new RunCommand(() -> wrist.setWristPosition(WristPosition.FLOOR), wrist));
    gamepadB.onTrue(new RunCommand(() -> wrist.setWristPosition(WristPosition.TOP_CONE), wrist));
    gamepadX.onTrue(new RunCommand(() -> wrist.setWristPosition(WristPosition.BOTTOM_MOST), wrist));

    ShuffleboardTab tab = Shuffleboard.getTab("Competition");
    GenericEntry elementBooleanBox = tab.add("Cone or Cube", false).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "#4d3399", "Color when false", "#ffff4d")).getEntry();
    SmartDashboard.putData("Box Color True", new InstantCommand(() -> elementBooleanBox.setBoolean(true)));
    SmartDashboard.putData("Box Color False", new InstantCommand(() -> elementBooleanBox.setBoolean(false)));

    GenericEntry allianceBooleanBox = tab.add("Alliance Color", false).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "red", "Color when false", "blue")).getEntry();

    LEDs.setBooleanBox(elementBooleanBox);
    
    // autons
    auton = new Autons(drivetrain, allianceBooleanBox);

    // gamepadA.onTrue(new InstantCommand(() -> LEDs.setColor(Colors.CELEBRATION)));
    // gamepadX.onTrue(
    //   new ParallelCommandGroup(
    //     new InstantCommand(() -> LEDs.setColor(Colors.PURPLE)),
    //     new InstantCommand(() -> elementBooleanBox.setBoolean(true))
    //   )
    // );
    gamepadY.onTrue(
      new RunCommand(() -> LEDs.lightUp(LEDState.YELLOW), LEDs)
    );
    // gamepadB.onTrue(new InstantCommand(() -> LEDs.setColor(Colors.OFF)));

    left1.onTrue(new RunCommand(() -> drivetrain.lockSwerve(), drivetrain));
    left2.onTrue(auton.testSwerveCommand());

    left4.onTrue(new InstantCommand(() -> drivetrain.setTrajectorySmartdash(auton.generateDriveStraightTraj(), "traj"), drivetrain));
    left5.onTrue(auton.driveStraight());
    left6.onTrue(new InstantCommand(() -> SmartDashboard.putString("ALLIANCE COLOR", DriverStation.getAlliance().toString())));

    // in honor of resetTurret
    left10.onTrue(new InstantCommand(() -> drivetrain.resetGyro(), drivetrain));

    right1.onTrue(new InstantCommand(() -> drivetrain.joyDrive(0, 0, 0), drivetrain));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

        left1 = leftJoystick.button(JOYSTICK_BUTTONS.BTN1);
        left2 = leftJoystick.button(JOYSTICK_BUTTONS.BTN2);
        left3 = leftJoystick.button(JOYSTICK_BUTTONS.BTN3);
        left4 = leftJoystick.button(JOYSTICK_BUTTONS.BTN4);
        left5 = leftJoystick.button(JOYSTICK_BUTTONS.BTN5);
        left6 = leftJoystick.button(JOYSTICK_BUTTONS.BTN6);
        left7 = leftJoystick.button(JOYSTICK_BUTTONS.BTN7);
        left8 = leftJoystick.button(JOYSTICK_BUTTONS.BTN8);
        left9 = leftJoystick.button(JOYSTICK_BUTTONS.BTN9);
        left10 = leftJoystick.button(JOYSTICK_BUTTONS.BTN10);
        left11 = leftJoystick.button(JOYSTICK_BUTTONS.BTN11);

        right1 = rightJoystick.button(JOYSTICK_BUTTONS.BTN1);
        right2 = rightJoystick.button(JOYSTICK_BUTTONS.BTN2);
        right3 = rightJoystick.button(JOYSTICK_BUTTONS.BTN3);
        right4 = rightJoystick.button(JOYSTICK_BUTTONS.BTN4);
        right5 = rightJoystick.button(JOYSTICK_BUTTONS.BTN5);
        right6 = rightJoystick.button(JOYSTICK_BUTTONS.BTN6);
        right7 = rightJoystick.button(JOYSTICK_BUTTONS.BTN7);
        right8 = rightJoystick.button(JOYSTICK_BUTTONS.BTN8);
        right9 = rightJoystick.button(JOYSTICK_BUTTONS.BTN9);
        right10 = rightJoystick.button(JOYSTICK_BUTTONS.BTN10);
        right11 = rightJoystick.button(JOYSTICK_BUTTONS.BTN11);

        gamepadA = gamepad.a();
        gamepadB = gamepad.b();
        gamepadX = gamepad.x();
        gamepadY = gamepad.y();
        gamepadRB = gamepad.rightBumper();
        gamepadLB = gamepad.leftBumper();
        gamepadBack = gamepad.back();
        gamepadStart = gamepad.start();
        gamepadLeftStickButton = gamepad.leftStick();
        gamepadRightStickButton = gamepad.rightStick();
        gamepadLT = gamepad.leftTrigger();
        gamepadRT = gamepad.rightTrigger();
        
        gamepadPOVDown = gamepad.povDown();
        gamepadPOVUpLeft = gamepad.povUpLeft();
        gamepadPOVUp = gamepad.povUp();
        gamepadPOVUpRight = gamepad.povUpRight();
        gamepadPOVLeft = gamepad.povLeft();
        gamepadPOVRight = gamepad.povRight();
        gamepadPOVDownRight = gamepad.povDownRight();
        gamepadPOVDownLeft = gamepad.povDownLeft();

        gamepadLeftX = () -> gamepad.getLeftX();
        gamepadRightX = () -> gamepad.getRightX();
        gamepadLeftY = () -> gamepad.getLeftY();
        gamepadRightY = () -> gamepad.getRightY();

        leftJoystickX = () -> leftJoystick.getX();
        leftJoystickY = () -> rightJoystick.getY();
        rightJoystickX = () -> rightJoystick.getX();
        rightJoystickY = () -> rightJoystick.getY();


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Autons getAutonomous() {
    // An example command will be run in autonomous
    return auton;
  }
}
