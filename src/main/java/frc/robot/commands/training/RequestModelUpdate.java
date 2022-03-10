package frc.robot.commands.training;

import java.util.concurrent.Future;

import frc.robot.Constants;
import frc.robot.base.shooter.ShooterModel;
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
    private final TrainerContext _context;
    private final NetworkClient _client;
    private final TrainerDashboard _dashboard;

    private Future<NetworkResponse> _request;

    public RequestModelUpdate(TrainerDashboard dashboard, NetworkClient client, TrainerContext context) {
        _context = context;
        _client = client;
        _request = null;
        _dashboard = dashboard;
    }

    @Override
    public void initialize() {
        BundleSendable payload = new BundleSendable();
            payload.putSendable("trainer.topic", new StringSendable("trainer:getModel"));
            payload.putSendable("trainer.configuration", new StringSendable(_context.getMode().name()));

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
                BundleSendable payload = (BundleSendable) response.getSendableResult();

                ArraySendable parameters = (ArraySendable) payload.getSendable("trainer.model.parameters");

                ValueSendable modelParameterA = (ValueSendable) parameters.get(3);
                ValueSendable modelParameterB = (ValueSendable) parameters.get(2);
                ValueSendable modelParameterC = (ValueSendable) parameters.get(1);
                ValueSendable modelParameterD = (ValueSendable) parameters.get(0);

                ShooterModel lastModel = _context.getModel();

                _context.setModel(
                    new ShooterModel(
                        modelParameterA.getValue(double.class),
                        modelParameterB.getValue(double.class),
                        modelParameterC.getValue(double.class),
                        modelParameterD.getValue(double.class),
                        lastModel.kPitch,
                        lastModel.kHeight,
                        lastModel.kOffset,
                        Constants.Shooter.DISTANCE_RANGE,
                        Constants.Shooter.SPEED_RANGE
                    )
                );

                _dashboard.update();

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
