package frc.robot.commands.training.shooter;

import java.util.concurrent.Future;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.training.protocol.NetworkClient;
import frc.robot.training.protocol.NetworkStatus;
import frc.robot.training.protocol.NetworkResponse;
import frc.robot.training.protocol.NetworkRequest;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;
import frc.robot.Constants;
import frc.robot.base.Property;
import frc.robot.base.shooter.ShooterConfiguration;
import frc.robot.base.shooter.TrainingModelProvider;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingModel4;

public class SubmitSetpointData extends CommandBase {
    private final Property<ShooterConfiguration> configuration;
    private final TrainingModelProvider provider;
    private final TrainerDashboard dashboard;
    private final NetworkClient client;

    private Future<NetworkResponse> request;

    public SubmitSetpointData(
        Property<ShooterConfiguration> configuration,
        TrainingModelProvider provider,
        TrainerDashboard dashboard,
        NetworkClient client
    ) {
        this.configuration = configuration;
        this.dashboard = dashboard;
        this.provider = provider;
        this.client = client;

        request = null;
    }

    @Override
    public void initialize() {
        ShooterConfiguration config = configuration.get();
        if (config == null)
            throw new RuntimeException("Null configuration");

        TrainingModel4 executionModel = provider.getExecutionModel();

        BundleSendable payload = new BundleSendable();
            payload.putSendable("trainer.topic", new StringSendable("trainer:submitData"));
            payload.putSendable("trainer.configuration", new StringSendable(config.getMode().name()));

            payload.putDouble("trainer.data.speed",
                Constants.Shooter.SPEED_RANGE.normalize(executionModel.getSetpoint().getTarget()));

            payload.putDouble("trainer.data.distance",
                Constants.Shooter.DISTANCE_RANGE.normalize(executionModel.getDistance()));

        request = client.submitRequestAsync(new NetworkRequest(payload));
        System.out.println("Sent request " + payload);
    }

    @Override
    public void end(boolean interrupted) {
        try {
            NetworkResponse response = request.get();

            System.out.println("Received status : " + response.getStatus());
            System.out.println("Received payload : " + response.getSendableResult());

            if (response.getStatus() == NetworkStatus.STATUS_OK && response.getSendableResult() != null) {
                BundleSendable payload = (BundleSendable) response.getSendableResult();

                ArraySendable parameters = (ArraySendable) payload.getSendable("trainer.model.parameters");

                TrainingModel4 executionModel = provider.getExecutionModel();

                executionModel.setModel(
                    parameters.get(3, ValueSendable.class).getValue(double.class),
                    parameters.get(2, ValueSendable.class).getValue(double.class),
                    parameters.get(1, ValueSendable.class).getValue(double.class),
                    parameters.get(0, ValueSendable.class).getValue(double.class)
                );
            }

            dashboard.update();
        } catch (Exception e) {
            throw new RuntimeException("Failed to retrieve result of request", e);
        }
    }

    @Override
    public boolean isFinished() {
        return request.isDone() || request.isCancelled();
    }
}
