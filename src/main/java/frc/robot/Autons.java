package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.SWERVE;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Autons {
    private SendableChooser<Auton> autonChooser;
    private Auton currentSelectedAuton;
    private Drivetrain drivetrain;
    private TrajectoryConfig trajConfig;
    private ProfiledPIDController turningPIDController;
    private PIDController xController, yController;
    

    private final double TURNING_P_VAL = 1;
    private final double X_P_VAL = 1, Y_P_VAL = 1;
    private final TrapezoidProfile.Constraints trapezoidalConstraint;
    private final double MAX_DIRECTIONAL_SPEED = 3, MAX_ACCELERATION = 3;
    private final double MAX_ROTATIONAL_SPEED = Math.PI;

    public Autons(Drivetrain drivetrain) {
        this.currentSelectedAuton = Auton.DEFAULT;
        this.drivetrain = drivetrain;
        this.trajConfig = new TrajectoryConfig(MAX_DIRECTIONAL_SPEED, MAX_ACCELERATION).setKinematics(this.drivetrain.getKinematics());
        this.trapezoidalConstraint = new TrapezoidProfile.Constraints(
            MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);

        turningPIDController = new ProfiledPIDController(TURNING_P_VAL, 0, 0, this.trapezoidalConstraint);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        xController = new PIDController(X_P_VAL, 0, 0);
        yController = new PIDController(Y_P_VAL, 0, 0);
        
        autonChooser = new SendableChooser<Auton>();
        autonChooser.setDefaultOption("DEFAULT", Auton.DEFAULT);
        SmartDashboard.putData("Auton Chooser", autonChooser);
        SmartDashboard.putData(autonChooser);
        SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());

    }
    public Command getAutonCommand() {
        // run once at the start of auton
        Trajectory trajectory = null;
        if (this.currentSelectedAuton == Auton.DEFAULT) {
            trajectory = TrajectoryGenerator.generateTrajectory(
                drivetrain.getInitialPose(),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                KnownPoses.GRID.pose, 
                trajConfig);
        }
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

    public void updateDash() {
        // run constantly when disabled
        Auton currAuton = autonChooser.getSelected();
        if (currAuton != this.currentSelectedAuton) {
            this.currentSelectedAuton = currAuton;
            SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());    
        }   
    }
    
    public enum Auton {
        DEFAULT
    }

    public enum KnownPoses {
        GRID(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        CHARGING_STATION(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
        LOADING_STATION(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        
        public final Pose2d pose;

        private KnownPoses(Pose2d pose) {
            this.pose = pose;
        }


    }
}
