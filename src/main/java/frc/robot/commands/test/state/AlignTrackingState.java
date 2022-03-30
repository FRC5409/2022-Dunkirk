package frc.robot.commands.test.state;

import com.revrobotics.CANSparkMax.IdleMode;

import org.jetbrains.annotations.NotNull;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.command.StateCommandBase;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.odometry.DriveShooterOdometry;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.subsystems.shooter.ShooterTurret.ReferenceType;
import frc.robot.utils.Vector2;

public class AlignTrackingState extends StateCommandBase {
    private final Property<ShooterConfiguration> configuration;
    private final Property<DriveShooterOdometry> sharedOdometry;
    
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;

    private DriveShooterOdometry odometry;
    private ShooterOdometryModel model;

    private PIDController controller;
    private boolean done;

    public AlignTrackingState(
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Property<DriveShooterOdometry> sharedOdometry,
        Property<ShooterConfiguration> configuration
    ) {
        this.sharedOdometry = sharedOdometry;
        this.configuration = configuration;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.turret = turret;

        controller = new PIDController(0, 0, 0);
        controller.setSetpoint(0);

        SmartDashboard.putNumber("Shooter P", SmartDashboard.getNumber("Shooter P", 0.0));
        SmartDashboard.putNumber("Shooter I", SmartDashboard.getNumber("Shooter I", 0.0));
        SmartDashboard.putNumber("Shooter D", SmartDashboard.getNumber("Shooter D", 0.0));
        SmartDashboard.putNumber("Shooter Output Thresh", SmartDashboard.getNumber("Shooter Output Thresh", 0.0));
        SmartDashboard.putNumber("Target Offset Smoothing", SmartDashboard.getNumber("Target Offset Smoothing", 0.0));
        SmartDashboard.putNumber("Shooter Simulated Offset", SmartDashboard.getNumber("Shooter Simulated Offset", 0.0));
        
        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        if (!limelight.isEnabled())
            limelight.enable();

        limelight.setLedMode(LedMode.kModeOn);

        if (!turret.isEnabled())
            turret.enable();

        turret.setIdleMode(IdleMode.kBrake);
        
        ShooterConfiguration config = configuration.get();

        // Initialize odometry
        model = config.getOdometryModel();
        odometry = sharedOdometry.get();

        done = false;

        SmartDashboard.putData("Shooter Odometry Model", model);
    }

    @Override
    public void execute() {
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                //SmartDashboard.getNumber("Simulated Velocity", 0),
                turret.getRotation()
            );
        }

        if (odometry.hasTarget() && !odometry.isLost()) {
            controller.setP(SmartDashboard.getNumber("Shooter P", 0.0));
            controller.setI(SmartDashboard.getNumber("Shooter I", 0.0));
            controller.setD(SmartDashboard.getNumber("Shooter D", 0.0));

            double offset = Math.toRadians(odometry.getTurretOffset());
                //Math.toRadians(SmartDashboard.getNumber("Shooter Simulated Offset", 0.0));

            Vector2 kNextDirection = new Vector2(Math.cos(offset), Math.sin(offset)).unit();
            Vector2 kActiveDirection = odometry.getVisionDirection();

            double relativeRotation = Math.atan2(
                kActiveDirection.x * kNextDirection.y - kActiveDirection.y * kNextDirection.x,
                kActiveDirection.x * kNextDirection.x + kActiveDirection.y * kNextDirection.y
            );

            SmartDashboard.putNumber("Relative Target Rotation", relativeRotation);

            double output = controller.calculate(relativeRotation);
            SmartDashboard.putNumber("Direction output", output);

            if (Math.abs(output) > SmartDashboard.getNumber("Shooter Output Thresh", 0))
                turret.setReference(output, ReferenceType.kOutput);
            else
                turret.setReference(0, ReferenceType.kOutput);
            
            // double rotation = odometry.getRotation();
            // turret.setRotationTarget(-rotation*2);

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
        } else {
            done = true;
            next("frc.robot.shooter:sweep");
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted || getNextState() == null) {
            turret.disable();
            limelight.disable();
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public @NotNull String getStateName() {
        return "frc.robot.shooter:align";
    }
}
