package frc.robot.commands.training.shooter;

import java.util.concurrent.Future;

import frc.robot.Constants;
import frc.robot.base.Model4;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.training.protocol.NetworkClient;
import frc.robot.training.protocol.NetworkStatus;
import frc.robot.training.protocol.NetworkRequest;
import frc.robot.training.protocol.NetworkResponse;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;

public class RequestModelUpdate extends CommandBase {
    private final TrainerDashboard dashboard;
    private final TrainerContext context;
    private final NetworkClient client;

    private Future<NetworkResponse> request;

    public RequestModelUpdate(TrainerDashboard dashboard, NetworkClient client, TrainerContext context) {
        this.context = context;
        this.client = client;
        this.request = null;
        this.dashboard = dashboard;
    }

    @Override
    public void initialize() {
        BundleSendable payload = new BundleSendable();
            payload.putSendable("trainer.topic", new StringSendable("trainer:getModel"));
            payload.putSendable("trainer.configuration", new StringSendable(context.getMode().name()));

        System.out.println("Sent request " + payload);

        request = client.submitRequestAsync(
            new NetworkRequest(payload)
        );
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
                
                context.setExecutionModel(
                    new Model4(
                        parameters.get(3, ValueSendable.class).getValue(double.class),
                        parameters.get(2, ValueSendable.class).getValue(double.class),
                        parameters.get(1, ValueSendable.class).getValue(double.class),
                        parameters.get(0, ValueSendable.class).getValue(double.class),
                        Constants.Shooter.DISTANCE_RANGE,
                        Constants.Shooter.SPEED_RANGE
                    )
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
