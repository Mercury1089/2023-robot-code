package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Autons {
    private SendableChooser<Auton> autonChooser;
    private Auton currentSelectedAuton;

    public Autons() {
        this.currentSelectedAuton = Auton.DEFAULT;
        autonChooser = new SendableChooser<Auton>();
        autonChooser.setDefaultOption("DEFAULT", Auton.DEFAULT);
        SmartDashboard.putData("Auton Chooser", autonChooser);
        SmartDashboard.putData(autonChooser);

        SmartDashboard.putString("Auton Selected: ", this.currentSelectedAuton.toString());
    }

    public Command getAutonCommand() {
        // run once at the start of auton
        if (this.currentSelectedAuton == Auton.DEFAULT) {
            return null;
        } 
        return null;
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
