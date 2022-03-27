package frc.robot.commands.test;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.odometry.DriveByShooterOdometry;
import frc.robot.base.shooter.odometry.DriveByShooterOdometry;
import frc.robot.base.shooter.odometry.ShooterOdometryModel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.Limelight.TargetType;
import frc.robot.subsystems.shooter.ShooterTurret;
import frc.robot.utils.Vector2;

public class ShooterOdometryTest extends CommandBase {
    private Property<ShooterConfiguration> configuration;
    
    private final ShooterTurret turret;
    private final DriveTrain drivetrain;
    private final Limelight limelight;

    private DriveByShooterOdometry odometry;
    private ShooterOdometryModel model;

    private PIDController controller;

    private double kLastDiff;
    private boolean kActive;

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

        controller = new PIDController(0, 0, 0);

        SmartDashboard.putNumber("Shooter P", SmartDashboard.getNumber("Shooter P", 0.0));
        SmartDashboard.putNumber("Shooter I", SmartDashboard.getNumber("Shooter I", 0.0));
        SmartDashboard.putNumber("Shooter D", SmartDashboard.getNumber("Shooter D", 0.0));

        addRequirements(limelight, turret);
    }

    @Override
    public void initialize() {
        limelight.enable();
        limelight.setLedMode(LedMode.kModeOn);

        turret.enable();
        turret.setIdleMode(IdleMode.kCoast);
        
        ShooterConfiguration config = configuration.get();

        // Initialize odometry
        model = config.getOdometryModel();
        odometry = new DriveByShooterOdometry(
            model,
            config.getTargetFilter(),
            Constants.Shooter.FLYWHEEL_OFFSET_MAPPING,
            Constants.Shooter.TURRET_OFFSET_MAPPING
        );

        kActive = false;

        SmartDashboard.putData("Shooter Odometry Model", model);
    }

    @Override
    public void execute() {
        if (limelight.getTargetType() == TargetType.kHub) {
            odometry.update(
                limelight.getTargetPosition(),
                drivetrain.getEncoderVelocity(),
                turret.getRotation()
                //SmartDashboard.getNumber("Simulated Velocity", 0),
            );
        }

        controller.setP(SmartDashboard.getNumber("Shooter P", 0.0));
        controller.setI(SmartDashboard.getNumber("Shooter I", 0.0));
        controller.setD(SmartDashboard.getNumber("Shooter D", 0.0));

        double offset = Math.toRadians(odometry.getTurretOffset());
        
        if (kActive) {
            offset = MathUtil.interpolate(kLastDiff, offset, SmartDashboard.getNumber("Target Interpolation Factor", 1));
        } else {
            kActive = true;
        }

        Vector2 kNextDirection = new Vector2(Math.sin(offset), Math.cos(offset)).unit();
        Vector2 kActiveDirection = odometry.getDirection();
        

        double output = controller.calculate(kActiveDirection.x, kNextDirection.x);
        SmartDashboard.putNumber("Direction output", output);

        if (Math.abs(output) > SmartDashboard.getNumber("Shooter Thresh", 0)) {
          turret.setRotationTarget(turret.getRotationTarget() + output);
        }
        


        // double rotation = odometry.getRotation();
        // turret.setRotationTarget(-rotation*2);

        kLastDiff = offset;

        SmartDashboard.putNumber("Turret Rotation", turret.getRotation());
        SmartDashboard.putNumber("Odometry Rotation", odometry.getRotation());
        SmartDashboard.putNumber("Odometry Distance", odometry.getDistance());
        SmartDashboard.putNumber("Odometry Speed", odometry.getSpeed());
        SmartDashboard.putString("Odometry Target", odometry.getTarget().toString());
        SmartDashboard.putString("Odometry Direction", odometry.getDirection().toString());
        SmartDashboard.putString("Odometry Next Direction",kNextDirection.toString());
        SmartDashboard.putString("Odometry Velocity", odometry.getVelocity().toString());
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
