package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import frc.robot.RobotContainer;
import frc.robot.util.interfaces.IMercShuffleBoardPublisher;

public class ShuffleDash {

    private RobotContainer robotContainer;
    private List<IMercShuffleBoardPublisher> publishers;
    
    public ShuffleDash(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        publishers = new ArrayList<IMercShuffleBoardPublisher>();
    }

    public void addPublisher(IMercShuffleBoardPublisher publisher) {
        publishers.add(publisher);
    }

    public void updateDash() {

        for (IMercShuffleBoardPublisher publisher : publishers) {
            publisher.publishValues();
        }
    }

    public RobotContainer.Autons getAuton() {
        return robotContainer.getSelectedAuton();
    }
}
