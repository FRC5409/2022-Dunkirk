package frc.robot.commands.shooter.state;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.TimedStateCommand;
import frc.robot.base.shooter.ShooterState;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;
import frc.robot.utils.Toggleable;
import frc.robot.utils.Vector2;

// TODO update doc
public class ActiveDelayedAlignShooterState extends TimedStateCommand {
    private final Property<DriveShooterOdometry> sharedOdometry;
    private final PIDController sharedController;
    
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;
    private final Trigger trigger;
    
    private DriveShooterOdometry odometry;
    private boolean done;
    private Property<ShooterState> shooterState;
    private Property<Boolean> buttonDebounce;
    private boolean debounce;

    public ActiveDelayedAlignShooterState(
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Trigger trigger,
        Property<ShooterState> shooterState,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<Boolean> buttonDebounce,
        PIDController sharedController
    ) {
        this.buttonDebounce = buttonDebounce;
        this.sharedController = sharedController;
        this.sharedOdometry = sharedOdometry;
        this.shooterState = shooterState;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.trigger = trigger;
        this.turret = turret;
        
        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (!Toggleable.isEnabled(limelight))
            throw new RuntimeException("Cannot align shooter when requirements are not enabled.");
        else if (sharedOdometry.get() == null)
            throw new RuntimeException("Cannot operate shooter when odometry is not initialized");

        if (!turret.isEnabled())
            turret.enable();

        odometry = sharedOdometry.get();

        shooterState.set(ShooterState.kTarget);
        done = false;

        debounce = buttonDebounce.get();
    }

    @Override
    public void execute() {
        sharedController.setP(SmartDashboard.getNumber("Shooter P", 0.0));
        sharedController.setI(SmartDashboard.getNumber("Shooter I", 0.0));
        sharedController.setD(SmartDashboard.getNumber("Shooter D", 0.0));

        // Update odometry target
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                turret.getRotation()
            );
        }

        if (odometry.getTarget() != null && !odometry.isLost()) {
            double offset = Math.toRadians(odometry.getTurretOffset());
            
            Vector2 kNextDirection = new Vector2(Math.cos(offset), Math.sin(offset)).unit();
            Vector2 kActiveDirection = odometry.getVisionDirection();

            double relativeRotation = Math.atan2(
                kActiveDirection.x * kNextDirection.y - kActiveDirection.y * kNextDirection.x,
                kActiveDirection.x * kNextDirection.x + kActiveDirection.y * kNextDirection.y
            );

            SmartDashboard.putNumber("Relative Target Rotation", relativeRotation);

            double output = sharedController.calculate(relativeRotation);
            SmartDashboard.putNumber("Direction output", output);

            if (Math.abs(output) > SmartDashboard.getNumber("Shooter Output Thresh", 0))
                turret.setReference(output, ReferenceType.kOutput);
            else
                turret.setReference(0, ReferenceType.kOutput);

            updateDashboard(kNextDirection);
        } else {
            done = true;
            next("frc.robot.shooter:sweep");
        }

        if (trigger.get() && !debounce) {
            next("frc.robot.shooter:operate");
            done = true;
        } else if (debounce) {
            debounce = false;
        }
        
        if(Constants.kConfig.DEBUG)
            SmartDashboard.putData("Robot Odometry", odometry);
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            limelight.disable();
            turret.disable();
        }
        buttonDebounce.set(false);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:align";
    }

    public void updateDashboard(Vector2 kNextDirection) {
        SmartDashboard.putNumber("Turret Rotation", turret.getRotation());
        SmartDashboard.putNumber("Odometry Rotation", odometry.getRotation());
        SmartDashboard.putNumber("Odometry Distance", odometry.getDistance());
        SmartDashboard.putNumber("Odometry Speed", odometry.getSpeed());
        SmartDashboard.putString("Odometry Target", odometry.getTarget().toString());
        SmartDashboard.putString("Odometry Vision Direction", odometry.getVisionDirection().toString());
        SmartDashboard.putString("Odometry Direction", odometry.getDirection().toString());
        SmartDashboard.putString("Odometry Next Direction", kNextDirection.toString());
        SmartDashboard.putString("Odometry Velocity", odometry.getVelocity().toString());
        SmartDashboard.putNumber("Odometry Turret Offset", odometry.getTurretOffset());
        SmartDashboard.putNumber("Odometry Flywheel Offset", odometry.getFlywheelOffset());
    }
}
