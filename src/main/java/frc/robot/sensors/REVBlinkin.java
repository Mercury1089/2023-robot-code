package frc.robot.sensors;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class REVBlinkin {

    private Spark blinkin;
    private Colors color;
    
    public REVBlinkin() {
        // LEDS
        this.blinkin = new Spark(0);
        this.color = Colors.OFF;
        setColor(this.color);

    }

    public void setColor(Colors color) {
        this.color = color;
        this.blinkin.set(color.colorValue);
    }

    public Colors getColor() {
        return this.color;
    }

    public enum Colors {
        OFF(0.99), YELLOW(0.69), PURPLE(0.91), CELEBRATION(-0.87);

        public final double colorValue;

        Colors(double colorValue)  {
            this.colorValue = colorValue;
        }
    }
}
