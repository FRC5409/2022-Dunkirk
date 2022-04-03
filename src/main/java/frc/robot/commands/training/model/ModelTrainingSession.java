package frc.robot.commands.training.model;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.base.training.TrainingModel3;
import frc.robot.training.protocol.NetworkConnection;
import frc.robot.training.protocol.NetworkSendable;
import frc.robot.training.protocol.NetworkServerRequest;
import frc.robot.training.protocol.NetworkServerResponse;
import frc.robot.training.protocol.NetworkSocket;
import frc.robot.training.protocol.NetworkStatus;
import frc.robot.training.protocol.generic.ArraySendable;
import frc.robot.training.protocol.generic.BundleSendable;
import frc.robot.training.protocol.generic.StringSendable;
import frc.robot.training.protocol.generic.ValueSendable;

public class ModelTrainingSession extends CommandBase {
    private final NetworkConnection connection;
    private final Map<String, TrainingModel3> models;

    private Future<NetworkServerRequest> request;

    public ModelTrainingSession(NetworkConnection connection, Map<String, TrainingModel3> models) {
        this.connection = connection;
        this.models = models;
    }

    @Override
    public void initialize() {
        request = null;
    }
    
    @Override
    public void execute() {
        if (request == null) {
            request = connection.getRequestAsync();
        } else if (request.isDone()) {
            try {
                process(request);
            } catch (Exception e) {
                e.printStackTrace();
            }
            request = null;
        }
    }


    private void process(Future<NetworkServerRequest> future) throws InterruptedException, ExecutionException, IOException {
        NetworkServerRequest req = future.get();
        
        NetworkStatus status = req.getStatus();
        ArraySendable payload = (ArraySendable) req.getPayload();
        
        System.out.println("Got request:\n Status - " + status +
            "\n Payload - " + String.valueOf(payload));

        if (status == NetworkStatus.STATUS_OK && payload != null) {
            for (NetworkSendable value : payload) {
                BundleSendable model = (BundleSendable) value;

                StringSendable modelName = (StringSendable) model.getSendable("trainer.model.name");
                if (models.containsKey(modelName.getValue())) {
                    ArraySendable parameters = (ArraySendable) model.getSendable("trainer.model.parameters");
                
                    TrainingModel3 targetModel = models.get(modelName.getValue());
                        targetModel.kA = parameters.get(0, ValueSendable.class).getValue(double.class);
                        targetModel.kB = parameters.get(1, ValueSendable.class).getValue(double.class);
                        targetModel.kC = parameters.get(2, ValueSendable.class).getValue(double.class);
                        
                    System.out.println("Updated model '" + modelName.getValue() + "'.");
                } else {
                    System.out.println("Failed to update model '" + modelName.getValue() + "'. Model does not exist.");
                }
            }

            req.fulfill(new NetworkServerResponse(NetworkStatus.STATUS_OK));
        } else {
            req.fulfill(new NetworkServerResponse(NetworkStatus.STATUS_ERROR));
        }
    }

    @Override
    public boolean isFinished() {
        return !connection.isConnected();
    }
}
