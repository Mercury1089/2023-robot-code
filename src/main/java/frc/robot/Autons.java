package frc.robot;

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
}
