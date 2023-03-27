package frc.robot.auton;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.GamePieceLEDs;
import frc.robot.subsystems.GamePieceLEDs.LEDState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Claw;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.Claw.ClawPosition;
import frc.robot.subsystems.arm.Telescope.TelescopePosition;
import frc.robot.subsystems.arm.Wrist.WristPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class Autons {

    private SendableChooser<PathPoint> autonChooser;
    private SendableChooser<PathPoint> startingPoseChooser;
    private PathPoint currentSelectedAuton;
    private PathPoint currentSelectedPose;
    private PathConstraints pathConstraints;
    private PIDController turningPIDController;
    private PIDController xController, yController;
    private Command autonCommand;
    private KnownLocations knownLocations;

    private Alliance allianceColor;

    private final double TURNING_P_VAL = 1;
    private final double X_P_VAL = 1, Y_P_VAL = 1;
    private final double MAX_DIRECTIONAL_SPEED = 3, MAX_ACCELERATION = 3;

    private Drivetrain drivetrain;
    private Arm arm;
    private Telescope telescope;
    private Wrist wrist;
    private Claw claw;
    private GamePieceLEDs LEDs;

    /**
     * made by rohan no thanks to owen :(
     */
    public Autons(Drivetrain drivetrain, Arm arm, Telescope telescope, Wrist wrist, Claw claw, GamePieceLEDs LEDs) {

        this.allianceColor = DriverStation.getAlliance();

        this.knownLocations = new KnownLocations();
        this.currentSelectedAuton = knownLocations.ELEMENT1;
        this.currentSelectedPose = knownLocations.START_TOPMOST;

        this.drivetrain = drivetrain;
        this.arm = arm;
        this.telescope = telescope;
        this.wrist = wrist;
        this.claw = claw;

        this.pathConstraints = new PathConstraints(MAX_DIRECTIONAL_SPEED, MAX_ACCELERATION); 
        turningPIDController = new PIDController(TURNING_P_VAL, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        xController = new PIDController(X_P_VAL, 0, 0);
        yController = new PIDController(Y_P_VAL, 0, 0);
        
        setChoosers();

        this.LEDs = LEDs;
    }

    public void setChoosers() {
        autonChooser = new SendableChooser<PathPoint>();
        autonChooser.setDefaultOption("DO NOTHING", KnownLocations.DO_NOTHING);
        autonChooser.addOption("Preload", knownLocations.ELEMENT1);
        autonChooser.addOption("Element 1", knownLocations.ELEMENT1);
        autonChooser.addOption("Element 2", knownLocations.ELEMENT2);
        autonChooser.addOption("Element 3", knownLocations.ELEMENT3);
        autonChooser.addOption("Element 4", knownLocations.ELEMENT4);
        autonChooser.addOption("Charging Station", knownLocations.CHARGING_CENTER);
        SmartDashboard.putData("Auton Chooser", autonChooser);
        SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());

        this.startingPoseChooser = new SendableChooser<PathPoint>();
        this.startingPoseChooser.setDefaultOption("TOPMOST", knownLocations.START_TOPMOST);
        this.startingPoseChooser.addOption("TOP SECOND", knownLocations.START_TOP_SECOND);
        this.startingPoseChooser.addOption("BOTTOM SECOND", knownLocations.START_BOTTOM_SECOND);
        this.startingPoseChooser.addOption("BOTTOMMOST", knownLocations.START_BOTTOMMOST);
        SmartDashboard.putData("Manual Starting Pose", startingPoseChooser);

        
    }

    public Command getAutonCommand() {
        return buildAutonCommand();
    }

    public Command buildAutonCommand() {        
        // SET OUR INITIAL POST
        drivetrain.setManualPose(new Pose2d(currentSelectedPose.position, currentSelectedPose.holonomicRotation));
        
        if (currentSelectedAuton == KnownLocations.DO_NOTHING) {
            SmartDashboard.putBoolean("isDoNothing", true);
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj1");
            drivetrain.setTrajectorySmartdash(new Trajectory(), "traj2");
            return getHomeCommand(arm, telescope, wrist, claw, LEDs);
        }

        List<PathPoint> waypoints = List.of();
        PathPoint finalPose = currentSelectedPose;

        if (currentSelectedPose == knownLocations.START_TOPMOST || currentSelectedPose == knownLocations.START_TOP_SECOND) {
            waypoints = Arrays.asList(knownLocations.WAYPOINT_TOP);

            if (currentSelectedPose == knownLocations.START_TOPMOST) {
                finalPose = knownLocations.START_TOP_SECOND;
            } else {
                finalPose = knownLocations.START_TOPMOST;
            }
        } else if (currentSelectedPose == knownLocations.START_BOTTOM_SECOND || currentSelectedPose == knownLocations.START_BOTTOMMOST) {
            waypoints = Arrays.asList(knownLocations.WAYPOINT_BOTTOM);

            if (currentSelectedPose == knownLocations.START_BOTTOMMOST) {
                finalPose = knownLocations.START_BOTTOM_SECOND;
            } else {
                finalPose = knownLocations.START_BOTTOMMOST;
            }
        }

        // if charging station is selected, this is our final destination (only 1 traj required)
        if (currentSelectedAuton == knownLocations.CHARGING_CENTER) {
            waypoints = List.of(); // ToDo: get waypoint for directly in front of the charging station
            finalPose = currentSelectedAuton;


            PathPlannerTrajectory traj1 = generateSwerveTrajectory(currentSelectedPose, waypoints, finalPose);
            drivetrain.setTrajectorySmartdash(traj1, "traj1");
            Command firstSwerveCommand = generateSwerveCommand(traj1);

            // reset the 2nd trajectory on the field widget
            PathPlannerTrajectory traj2 = new PathPlannerTrajectory();
            drivetrain.setTrajectorySmartdash(traj2, "traj2");

            return firstSwerveCommand;
        }

        PathPlannerTrajectory traj1 = generateSwerveTrajectory(currentSelectedPose, waypoints, finalPose);
        drivetrain.setTrajectorySmartdash(traj1, "traj1");
        Command firstSwerveCommand = generateSwerveCommand(traj1);
        // pickUpCommand()
        // tuckInCommand()
        // Trajectory traj2 = generateSwerveTrajectory(currentSelectedAuton, finalPose, waypoints);
        // drivetrain.setTrajectorySmartdash(traj2, "traj2");
        // Command secondSwerveCommand = generateSwerveCommand(traj2);
        // scoreCommand()

        return new SequentialCommandGroup(
            getHomeCommand(arm, telescope, wrist, claw, LEDs).until(() -> arm.isAtPosition(ArmPosition.INSIDE)),
            getAutonScoreHighCommand(arm, telescope, wrist, claw),
            new InstantCommand(() -> LEDs.lightUp(LEDState.CELEBRATION), LEDs),
            firstSwerveCommand
        );
    }

    public PathPlannerTrajectory generateSwerveTrajectory(PathPoint initialPose, List<PathPoint> waypoints, PathPoint finalPose) {
        List<PathPoint> points = new ArrayList<PathPoint>();
        points.addAll(waypoints);
        points.add(finalPose);
        PathPoint point2 = points.remove(0);
        // Following passes an array to vararg (see: https://programming.guide/java/passing-list-to-vararg-method.html)
        return PathPlanner.generatePath(pathConstraints, initialPose, point2, points.toArray(new PathPoint[0]));
    }

    /** Generate the swerve-specfic command by building the desired trajectory */
    public Command generateSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            () -> drivetrain.getPose(), // Functional interface to feed supplier
            drivetrain.getKinematics(),
            // Position controllers
            xController,
            yController,
            turningPIDController,
            (x) -> drivetrain.setModuleStates(x),
            false,
            drivetrain);
        return swerveControllerCommand;
    }

    /**
     * The logic for picking up, moving, and scoring pieces
     * will be used in both auton and throughout the game on buttons
     * so we declare in auton, and just call these methods to tie to 
     * buttons in RBC
     * @return Command
     */

     public Command getSubstationCommand(Arm arm, Telescope telescope, Wrist wrist, Claw claw) {
        return new ParallelCommandGroup(
            new RunCommand(() -> arm.setPosition(ArmPosition.RAMP_PICKUP), arm),
            new RunCommand(() -> telescope.setPosition(TelescopePosition.HOME), telescope),
            new RunCommand(() -> wrist.setPosition(WristPosition.RAMP), wrist),
            new RunCommand(() -> claw.setPosition(ClawPosition.RAMP), claw)   
        );
    }
    public Command getBulldozeCommand(Arm arm, Telescope telescope, Wrist wrist) {
        return new ParallelCommandGroup(
            new RunCommand(() -> arm.setPosition(ArmPosition.BULLDOZER), arm),
            new RunCommand(() -> wrist.setPosition(WristPosition.LEVEL), wrist),
            new SequentialCommandGroup(
              new WaitUntilCommand(() -> arm.isAtPosition(ArmPosition.BULLDOZER)),
              new ParallelCommandGroup(
                new RunCommand(() -> telescope.setPosition(TelescopePosition.BULLDOZER), telescope)
              )
            )
        );
    }

    public Command getHybridBulldozeCommand(Arm arm, Telescope telescope, Wrist wrist) {
        return new ParallelCommandGroup(
            new RunCommand(() -> arm.setPosition(ArmPosition.BULLDOZER), arm),
            new SequentialCommandGroup(
              new WaitUntilCommand(() -> arm.isAtPosition(ArmPosition.BULLDOZER)),
              new ParallelCommandGroup(
                new RunCommand(() -> telescope.setPosition(TelescopePosition.HOME), telescope),
                new RunCommand(() -> wrist.setPosition(WristPosition.LEVEL), wrist)
              )
            )
          );
    }

    public Command getTuckInCommand(Arm arm, Telescope telescope, Wrist wrist) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunCommand(() -> telescope.setPosition(TelescopePosition.HOME), telescope),
                new RunCommand(() -> wrist.setPosition(WristPosition.INSIDE), wrist),
                new RunCommand(() -> arm.setPosition(ArmPosition.BULLDOZER), arm))
                    .until(() -> telescope.isAtPosition(TelescopePosition.INSIDE)),
            new ParallelCommandGroup(
                new RunCommand(() -> telescope.setPosition(TelescopePosition.HOME), telescope),
                new RunCommand(() -> wrist.setPosition(WristPosition.INSIDE), wrist),
                new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm)
            )
        );
    }

    public Command getHomeCommand(Arm arm, Telescope telescope, Wrist wrist, Claw claw, GamePieceLEDs LEDs) {
        return new ParallelCommandGroup(
            new RunCommand(() -> telescope.setPosition(TelescopePosition.HOME), telescope),
            new RunCommand(() -> wrist.setPosition(WristPosition.INSIDE), wrist),
            new SequentialCommandGroup(
                new InstantCommand(() -> LEDs.lightUp(LEDState.YELLOW), LEDs),
                new RunCommand(() -> claw.close(LEDs), claw)
            ),
            new SequentialCommandGroup(
              new WaitUntilCommand(() -> telescope.isAtPosition(TelescopePosition.INSIDE)),
              new RunCommand(() -> arm.setPosition(ArmPosition.HOME), arm)
            )
          );
    }

    public Command getScorePieceMidCommand(Arm arm, Telescope telescope, Wrist wrist) {
        return new ParallelCommandGroup(
            new RunCommand(() -> arm.setPosition(ArmPosition.MID_SCORE), arm),
            new SequentialCommandGroup(
              new WaitUntilCommand(() -> arm.isAtPosition(ArmPosition.MID_SCORE)),
              new ParallelCommandGroup(
                new RunCommand(() -> telescope.setPosition(TelescopePosition.MID_SCORE), telescope),

                new RunCommand(() -> wrist.setPosition(WristPosition.MID_SCORE), wrist)
              )
            )
          );
    }

    public Command getScorePieceHighCommand(Arm arm, Telescope telescope, Wrist wrist) {
        return new ParallelCommandGroup(
            new RunCommand(() -> arm.setPosition(ArmPosition.HIGH_SCORE), arm),
            new SequentialCommandGroup(
              new WaitUntilCommand(() -> arm.getArmPosition() > ArmPosition.MID_SCORE.degreePos),
              new ParallelCommandGroup(
                new RunCommand(() -> telescope.setPosition(TelescopePosition.HIGH_SCORE), telescope),
                new RunCommand(() -> wrist.setPosition(WristPosition.HIGH_SCORE), wrist)
              )
            )
          );
    }

    public Command getAutonScoreHighCommand(Arm arm, Telescope telescope, Wrist wrist, Claw claw) {
        return new SequentialCommandGroup(
            getScorePieceHighCommand(arm, telescope, wrist).until(() ->
            (telescope.isAtPosition(TelescopePosition.HIGH_SCORE))),
            new RunCommand(() -> wrist.setPosition(WristPosition.LEVEL), wrist).until(() -> wrist.isAtPosition(WristPosition.LEVEL)),
            new RunCommand(() -> claw.open(), claw).until(() -> claw.isAtPosition(ClawPosition.OPEN)),
            getTuckInCommand(arm, telescope, wrist).until(() -> arm.isAtPosition(ArmPosition.INSIDE))
        );
        
    }

    public void updateDash() {
        // run constantly when disabled
        PathPoint currAuton = autonChooser.getSelected();
        PathPoint currPose = startingPoseChooser.getSelected();

        Alliance color = DriverStation.getAlliance();

        if (color != this.allianceColor) {
            this.allianceColor = color;
            this.knownLocations = new KnownLocations();
            SmartDashboard.putString("alliance color!", this.allianceColor.toString());
            setChoosers();
            this.autonCommand = buildAutonCommand();
        }
        
        if (currAuton != this.currentSelectedAuton) {
            this.currentSelectedAuton = currAuton;
            SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());
            this.autonCommand = buildAutonCommand();
        }

        if (currPose != this.currentSelectedPose) {
            this.currentSelectedPose = currPose;
            this.autonCommand = buildAutonCommand();
        }
    }    
}