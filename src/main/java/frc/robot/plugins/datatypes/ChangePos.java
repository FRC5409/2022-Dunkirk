package frc.robot.plugins.datatypes;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


public class ChangePos implements Sendable{
    double angle;
    double dist;

    public ChangePos(double angle, double dist){
        this.angle = angle;
        this.dist = dist;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ChangePos");
        builder.addDoubleProperty("angle", this::getAngle, null);
        builder.addDoubleProperty("distance", this::getDist, null);
    }


    @Override
    public boolean equals(Object obj) {
        ChangePos object = (ChangePos) obj;
        return (object.angle == angle && object.dist == dist);
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double newAngle){
        angle = newAngle;
    }

    public double getDist() {
        return dist;
    }

    public void setDist(double newDist){
        dist = newDist;
    }
}