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

import frc.robot.base.training.TrainerContext;
import frc.robot.base.training.TrainerDashboard;
import frc.robot.base.training.TrainingConfiguration;
import frc.robot.base.training.TrainingModel4;

public class SubmitSetpointData extends CommandBase {
    private final TrainerDashboard dashboard;
    private final TrainerContext context;
    private final NetworkClient client;

    private Future<NetworkResponse> request;

    public SubmitSetpointData(TrainerDashboard dashboard, NetworkClient client, TrainerContext context) {
        this.context = context;
        this.client = client;
        this.dashboard = dashboard;

        request = null;
    }

    @Override
    public void initialize() {
        TrainingConfiguration configuration = context.getConfiguration();

        BundleSendable payload = new BundleSendable();
            payload.putSendable("trainer.topic", new StringSendable("trainer:submitData"));
            payload.putSendable("trainer.configuration", new StringSendable(context.getMode().name()));

            payload.putDouble("trainer.data.speed",
                Constants.Shooter.SPEED_RANGE.normalize(configuration.getExecutionModel().getSetpoint().getTarget()));

            payload.putDouble("trainer.data.distance",
                Constants.Shooter.DISTANCE_RANGE.normalize(configuration.getDistance()));

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

                TrainingModel4 executionModel = context.getConfiguration().getExecutionModel();

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
