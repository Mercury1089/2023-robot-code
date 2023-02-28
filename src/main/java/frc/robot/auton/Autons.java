package frc.robot.auton;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;


public class Autons {

    private SendableChooser<Pose2d> autonChooser;
    private SendableChooser<Pose2d> startingPoseChooser;
    private Pose2d currentSelectedAuton;
    private Pose2d currentSelectedPose;
    private Drivetrain drivetrain;
    private TrajectoryConfig trajConfig;
    private ProfiledPIDController turningPIDController;
    private PIDController xController, yController;
    private Command swerveCommand;
    private KnownLocations knownLocations;

    private final double TURNING_P_VAL = 1;
    private final double X_P_VAL = 1, Y_P_VAL = 1;
    private final TrapezoidProfile.Constraints trapezoidalConstraint;
    private final double MAX_DIRECTIONAL_SPEED = 3, MAX_ACCELERATION = 3;
    private final double MAX_ROTATIONAL_SPEED = Math.PI;

    /**
     * made by rohan no thanks to owen :(
     */
    public Autons(Drivetrain drivetrain, GenericEntry allianceWidget) {

        this.knownLocations = new KnownLocations(allianceWidget);
        this.currentSelectedAuton = knownLocations.ELEMENT1;
        this.currentSelectedPose = knownLocations.START_TOPMOST;
        this.drivetrain = drivetrain;
        
        this.trajConfig = new TrajectoryConfig(MAX_DIRECTIONAL_SPEED, MAX_ACCELERATION).setKinematics(this.drivetrain.getKinematics());
        this.trapezoidalConstraint = new TrapezoidProfile.Constraints(
            MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);

        turningPIDController = new ProfiledPIDController(TURNING_P_VAL, 0, 0, this.trapezoidalConstraint);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        xController = new PIDController(X_P_VAL, 0, 0);
        yController = new PIDController(Y_P_VAL, 0, 0);
        
        autonChooser = new SendableChooser<Pose2d>();
        autonChooser.setDefaultOption("Element 1", knownLocations.ELEMENT1);
        autonChooser.addOption("Element 2", knownLocations.ELEMENT2);
        autonChooser.addOption("Element 3", knownLocations.ELEMENT3);
        autonChooser.addOption("Element 4", knownLocations.ELEMENT4);
        autonChooser.addOption("Charging Station", knownLocations.CHARGING_CENTER);
        SmartDashboard.putData("Auton Chooser", autonChooser);
        SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());

        this.startingPoseChooser = new SendableChooser<Pose2d>();
        this.startingPoseChooser.setDefaultOption("TOPMOST", knownLocations.START_TOPMOST);
        this.startingPoseChooser.addOption("TOP SECOND", knownLocations.START_TOP_SECOND);
        this.startingPoseChooser.addOption("BOTTOM SECOND", knownLocations.START_BOTTOM_SECOND);
        this.startingPoseChooser.addOption("BOTTOMMOST", knownLocations.START_BOTTOMMOST);
        SmartDashboard.putData("Manual Starting Pose", startingPoseChooser);
        SmartDashboard.putBoolean("MANUAL START NEEDED", false);

        this.swerveCommand = getAutonCommand();
    }

    public Command getAutonCommand() {
        // will eventually use this.swerveCommand; 
        // in conjunction w/ other subsystems;
        // to build full autons;
        long nanos = System.nanoTime();
        generateTestTrajectory();
        long diff = System.nanoTime() - nanos;
        SmartDashboard.putNumber("RobotPathfinder Generation Time (s)", diff / 1e9);
        List<Translation2d> waypoints = List.of();
        Pose2d finalPose = currentSelectedPose;

        if (currentSelectedPose == knownLocations.START_TOPMOST || currentSelectedPose == knownLocations.START_TOP_SECOND) {
            waypoints = Arrays.asList(knownLocations.WAYPOINT_TOP);
            // 
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

        Trajectory traj1 = generateSwerveTrajectory(currentSelectedPose, currentSelectedAuton, waypoints);
        drivetrain.setTrajectorySmartdash(traj1, "traj1");
        Command firstSwerveCommand = generateSwerveCommand(traj1);
        Trajectory traj2 = generateSwerveTrajectory(currentSelectedAuton, finalPose, waypoints);
        drivetrain.setTrajectorySmartdash(traj2, "traj2");
        Command secondSwerveCommand = generateSwerveCommand(traj2);

        return new SequentialCommandGroup(
            firstSwerveCommand,
            secondSwerveCommand
        );
    }


    public Trajectory generateTestTrajectory() {
        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)), 
            List.of(new Translation2d(.5, .5)), 
            new Pose2d(1, 1, Rotation2d.fromDegrees(180)), 
            trajConfig);
    }

    public Command testSwerveCommand() {
       // drivetrain.setManualPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)), 
            List.of(new Translation2d(.5, .5)), 
            new Pose2d(1, 1, Rotation2d.fromDegrees(180)), 
            trajConfig);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            () -> drivetrain.getPose(),
            drivetrain.getKinematics(), 
            xController, 
            yController,
            turningPIDController,
            (x) -> drivetrain.setModuleStates(x),
            drivetrain);

        return swerveControllerCommand;
    }

    public Trajectory generateSwerveTrajectory(Pose2d initialPose, Pose2d finalPose, List<Translation2d> waypoints) {
        return TrajectoryGenerator.generateTrajectory(initialPose, waypoints, finalPose, trajConfig);
    }

    /** Generate the swerve-specfic command by building the desired trajectory */
    public Command generateSwerveCommand(Trajectory trajectory) {
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            () -> drivetrain.getPose(), // Functional interface to feed supplier
            drivetrain.getKinematics(),
            // Position controllers
            xController,
            yController,
            turningPIDController,
            (x) -> drivetrain.setModuleStates(x),
            drivetrain);
        return swerveControllerCommand;
    }

    public Trajectory generateDriveStraightTraj() {
        Pose2d initPose =  new Pose2d(Units.inchesToMeters(54.93), Units.inchesToMeters(199.65), Rotation2d.fromDegrees(0));
        Pose2d finalPose = new Pose2d(initPose.getX() + 5, initPose.getY(), Rotation2d.fromDegrees(180));
        Translation2d waypoint = new Translation2d(initPose.getX() + 2.5, initPose.getY() + 2.5);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            initPose, 
            List.of(waypoint),
            finalPose, 
            trajConfig);
        return traj;
    }

    /** Move straight in x direction */
    public Command driveStraight() {
        return new SwerveControllerCommand(
            generateDriveStraightTraj(),
            () -> drivetrain.getPose(), 
            drivetrain.getKinematics(), 
            xController, 
            yController, 
            turningPIDController,
            (x) -> drivetrain.setModuleStates(x),
            drivetrain
            );
        }


    public void updateDash() {
        // run constantly when disabled
        Pose2d currAuton = autonChooser.getSelected();
        Pose2d currPose = startingPoseChooser.getSelected();
        
        if (currAuton != this.currentSelectedAuton) {
            this.currentSelectedAuton = currAuton;
            SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());
            this.swerveCommand = getAutonCommand();
        }

        if (currPose != this.currentSelectedPose) {
            this.currentSelectedPose = currPose;
            this.swerveCommand = getAutonCommand();
        }
    }    
}
