package frc.robot.commands.training;

import java.util.concurrent.Future;

import frc.robot.Constants;
import frc.robot.training.TrainerDashboard;
import frc.robot.training.TrainerContext;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.training.protocol.NetworkClient;
import frc.robot.training.protocol.NetworkStatus;
import frc.robot.training.protocol.NetworkRequest;
import frc.robot.training.protocol.NetworkResponse;
import frc.robot.training.protocol.generic.KeyValueSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.utils.ShooterModel;

public class RequestModelUpdate extends CommandBase {
    private final TrainerContext _context;
    private final NetworkClient _client;
    private final TrainerDashboard m_dashboard;

    private Future<NetworkResponse> _request;

    public RequestModelUpdate(TrainerDashboard dashboard, NetworkClient client, TrainerContext context) {
        _context = context;
        _client = client;
        _request = null;
        m_dashboard = dashboard;
    }

    @Override
    public void initialize() {
        KeyValueSendable payload = new KeyValueSendable();
            payload.putSendable("trainer.topic", new StringSendable("trainer:getModel"));


        System.out.println("Sent request " + payload);

        _request = _client.submitRequestAsync(
            new NetworkRequest(payload)
        );
    }

    @Override
    public void end(boolean interrupted) {    
        try {
            NetworkResponse response = _request.get();
            if (response.getStatus() == NetworkStatus.STATUS_OK) {
                KeyValueSendable payload = (KeyValueSendable) response.getSendableResult();

                double modelA = payload.getDouble("trainer.model.parameters[3]");
                double modelB = payload.getDouble("trainer.model.parameters[2]");
                double modelC = payload.getDouble("trainer.model.parameters[1]");
                double modelD = payload.getDouble("trainer.model.parameters[0]");

                _context.setModel(
                    new ShooterModel(
                        modelA, modelB, modelC, modelD,
                        Constants.Shooter.DISTANCE_RANGE,
                        Constants.Shooter.SPEED_RANGE
                    )
                );

                m_dashboard.update();
                System.out.println("Received payload : " + payload);
            } else {
                System.out.println("Received status : " + response.getStatus());
            }
        } catch (Exception e) {
            throw new RuntimeException("Failed to retrieve result of request", e);
        }
    }

    @Override
    public boolean isFinished() {
        return _request.isDone() || _request.isCancelled();
    }
}
