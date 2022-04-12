package frc.robot.plugins.datatypes;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


public class ChangePos implements Sendable{
    double deltaAngle;
    double angle;
    double dist;
    double deltaDist;

    public ChangePos(double angle, double dist){
        this.angle = angle;
        this.dist = dist;
        deltaAngle = 0;
        deltaDist = 0;
    }


    public void update(double newAngle, double newDist){
        deltaAngle = newAngle - angle;
        deltaDist = newDist - dist;
        angle = newAngle;
        dist = newDist;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ChangePos");
        builder.addDoubleProperty("angle", this::getDeltaAngle, this::setDeltaAngle);
        builder.addDoubleProperty("distance", this::getDeltaDist, this::setDeltaDist);
    }

    public double getDeltaAngle(){
        return deltaAngle;
    }

    public void setDeltaAngle(double newDeltaAngle){
        deltaAngle = newDeltaAngle;
    }

    public double getDeltaDist(){
        return deltaDist;
    }

    public void setDeltaDist(double newDeltaDist){
        deltaDist = newDeltaDist;
    }
}