package frc.robot.commands.test;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.Property;
import frc.robot.base.shooter.ActiveShooterOdometry;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.ShooterOdometryModel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;

public class ShooterOdometryTest extends CommandBase {
    private Property<ShooterConfiguration> configuration;
    
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;

    private ActiveShooterOdometry odometry;
    private ShooterOdometryModel model;

    public ShooterOdometryTest(
        ShooterTurret turret,
        DriveTrain drivetrain,
        Limelight limelight,
        Property<ShooterConfiguration> configuration
    ) {
        this.configuration = configuration;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.turret = turret;

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        limelight.enable();
        limelight.setLedMode(LedMode.kModeOn);

        turret.enable();
        turret.setIdleMode(IdleMode.kCoast);

        // Initialize odometry
        model = configuration.get().getOdometryModel();
        odometry = new ActiveShooterOdometry(model);

        SmartDashboard.putData("Shooter Odometry Model", model);
    }

    @Override
    public void execute() {
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                turret.getRotation(),
                drivetrain.getEncoderVelocity()
            );
        }

        SmartDashboard.putData("Shooter Odometry", odometry);
    }

    @Override
    public void end(boolean interrupted) {
        turret.disable();
        limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
