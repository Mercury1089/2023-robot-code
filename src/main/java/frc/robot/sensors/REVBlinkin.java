package frc.robot.sensors;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;

public class REVBlinkin {

    private Spark blinkin;
    private Colors color;
    public REVBlinkin() {
        // LEDS
        this.blinkin = new Spark(0);
        this.color = Colors.OFF;

    }

    public void setColor(Colors color) {
        this.color = color;
        if (color == Colors.OFF) {
            this.blinkin.set(0);
        } else if (color == Colors.YELLOW) {
            this.blinkin.set(0.69);
        } else if (color == Colors.PURPLE) {
            this.blinkin.set(0.91);
        } else {
            this.blinkin.set(0.15);
        }
        
    }

    public Colors getColor(){
        return this.color;
    }


    public enum Colors {
        OFF,
        YELLOW,
        PURPLE,
        CELEBRATION
    }
}
