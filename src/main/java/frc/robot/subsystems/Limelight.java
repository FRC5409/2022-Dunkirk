package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Range;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

/**
 * Facilitates the control and access
 * of limelight hardware.
 * 
 * 
 * http://10.54.9.11:5801/
 * 
 * @author Keith Davies, Akil Pathiranage
 */
public class Limelight extends SubsystemBase implements Toggleable {
    /**
     * The Led mode of the limelight.
     */
    public enum LedMode {
        kModePipeline(0), kModeOff(1), kModeBlink(2), kModeOn(3);

        LedMode(double value) {
            this.value = value;
        }

        public final double value;
    }

    /**
     * The camera mode of the limelight.
     */
    public enum CameraMode {
        kModeVision(0), kModeDriver(1);

        CameraMode(double value) {
            this.value = value;
        }

        public final double value;
    }

    /**
     * The type of target seen by the limelight.
     */
    public enum TargetType {
        kHub, kNone
    }

    private final NetworkTable data;

    private final NetworkTableEntry targetPositionX, targetPositionY, pipelineIndex,
        targetStatus, targetArea, cameraMode, ledMode;

    private TargetType type;
    private Vector2 position;
    private double area;

    private boolean enabled;

    /**
     * Constructs the Limelight subsystem.
     */
    public Limelight() {
        data = NetworkTableInstance.getDefault().getTable(Constants.Limelight.NETWORK_TABLE_NAME);

        targetPositionX = data.getEntry("tx");
        targetPositionY = data.getEntry("ty");
        pipelineIndex   = data.getEntry("pipeline");
        targetStatus    = data.getEntry("tv");
        targetArea      = data.getEntry("ta");
        cameraMode      = data.getEntry("camMode");
        ledMode         = data.getEntry("ledMode");
        data.getEntry("stream").setNumber(0);
        
        position = new Vector2();
        area = 0.0;
        type = TargetType.kNone;

        enabled = false;
    }

    /**
     * Enables the Limelight.
     */
    public void enable() {
        if (Constants.kConfig.DEBUG)
            System.out.println("Enabled limelight");

        enabled = true;
    }

    /**
     * Disables the Limelight.
     */
    public void disable() {
        if (Constants.kConfig.DEBUG)
            System.out.println("Disabled limelight");
        
        setLedMode(LedMode.kModeOff);

        type = TargetType.kNone;

        enabled = false;
    }

    /**
     * Checks whether or not the limelight subsystem
     * is currently enabled.
     * 
     * @return The subsystems enabled state.
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Set's the camera mode of the limelight.
     * 
     * @param mode The camera mode.
     * 
     * @see CameraMode
     */
    public void setCameraMode(CameraMode mode) {
        cameraMode.setDouble(mode.value);
    }

    /**
     * Set's the led mode of the limelight.
     * 
     * @param mode The led mode.
     * 
     * @see LedMode
     */
    public void setLedMode(LedMode mode) {
        ledMode.setDouble(mode.value);
    }

    /**
     * Set's the pipeline index of the limelight.
     * 
     * @param index The pipeline index. [0-9]
     */
    public void setPipelineIndex(int index) {
        pipelineIndex.setDouble(Range.clamp(0, index, 9));
    }

    /**
     * Get's the current tracking target off the limelight
     * pipeline.
     * 
     * @return The limelight target.
     */
    public Vector2 getTargetPosition() {
        return new Vector2(position);
    }

    /**
     * Get's the type of target seen by the limelight.
     * 
     * @return The active target.
     */
    public TargetType getTargetType() {
        return type;
    }

    /**
     * Get's the current tracking target area.
     * 
     * @return The active target area.
     */
    public double getTargetArea() {
        return area;
    }

    /**
     * Checks to see if the limelight is currently tracking any targets.
     * 
     * @return Whether or not the limelight is tracking a target.
     */
    public boolean hasTarget() {
        return type != TargetType.kNone;
    }

    @Override
    public void periodic() {
        if (enabled) {
            if (targetStatus.getDouble(0) == 1) {
                position.x = targetPositionX.getDouble(position.x);
                position.y = targetPositionY.getDouble(position.y);

                area = targetArea.getDouble(area);
                type = TargetType.kHub;
            } else {
                type = TargetType.kNone;
            }
        }
    }
}