package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Range;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;
import frc.robot.utils.Vector3;

/**
 * Facilitates the control and access
 * of limelight hardware.
 * 
 * 
 * http://10.54.9.99:5801/ 
 * @author Keith Davies, Akil Pathiranage
 */
public class Limelight extends SubsystemBase implements Toggleable  {
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

    private NetworkTable      limelight_data;

    private NetworkTableEntry data_entry_tx,
                              data_entry_ty,
                              data_entry_ta,
                              data_entry_led_mode, 
                              data_entry_cam_mode,
                              data_entry_pipeline,
                              data_entry_has_targets;

    private Vector3           track_data;
    private TargetType        target_data;

    private boolean           enabled;

    /**
     * Constructs the Limelight subsystem.
     */
    public Limelight() {
        limelight_data         = NetworkTableInstance.getDefault().getTable(Constants.Limelight.NETWORK_TABLE_NAME);

        data_entry_tx          = limelight_data.getEntry("tx");
        data_entry_ty          = limelight_data.getEntry("ty");
        data_entry_ta          = limelight_data.getEntry("ta");

        data_entry_cam_mode    = limelight_data.getEntry("camMode");
        data_entry_led_mode    = limelight_data.getEntry("ledMode");
        data_entry_pipeline    = limelight_data.getEntry("pipeline");
        
        data_entry_has_targets = limelight_data.getEntry("tv");

        track_data             = new Vector3(0, 0, 0);
        target_data            = TargetType.kNone;

        enabled                = false;
    }

    /**
     * Enables the Limelight.
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Disables the Limelight.
     */
    public void disable() {
        enabled = false;
        target_data = TargetType.kNone;
        setLedMode(LedMode.kModeOff);
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
        data_entry_cam_mode.setDouble(mode.value);
    }

    /**
     * Set's the led mode of the limelight.
     * 
     * @param mode The led mode.
     * 
     * @see LedMode
     */
    public void setLedMode(LedMode mode) {
        data_entry_led_mode.setDouble(mode.value);
    }

    /**
     * Set's the pipeline index of the limelight.
     * 
     * @param index The pipeline index. [0-9]
     */
    public void setPipelineIndex(int index) {
        data_entry_pipeline.setDouble(Range.clamp(0, index, 9));
    }

    /**
     * Get's the current tracking target off the limelight
     * pipeline.
     * 
     * @return The limelight target.
     */
    public Vector2 getTarget() {
        return new Vector2(track_data.x, track_data.y);
    }

    /**
     * Get's the type of target seen by the limelight.
     * 
     * @return The active target.
     */
    public TargetType getTargetType() {
        return target_data;
    }

    /**
     * Get's the current tracking target area.
     * 
     * @return The active target area.
     */
    public double getTargetArea() {
        return track_data.z;
    }

    /**
     * Checks to see if the limelight is currently tracking any targets.
     * 
     * @return Whether or not the limelight is tracking a target.
     */
    public boolean hasTarget() {
        return target_data != TargetType.kNone;
    }

    private void updateTarget() {
        track_data.x = data_entry_tx.getDouble(track_data.x);
        track_data.y = data_entry_ty.getDouble(track_data.y);
        track_data.z = data_entry_ta.getDouble(track_data.z);
    }

    private boolean isTracking() {
        return data_entry_has_targets.getDouble(0) == 1;
    }
        
    @Override
    public void periodic() {
        if (enabled) {
            if (isTracking()) {
                updateTarget();
                target_data = TargetType.kHub;
            } else {
                target_data = TargetType.kNone;
            }
        }
    }
}