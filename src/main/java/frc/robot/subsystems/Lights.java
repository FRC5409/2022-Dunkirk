package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.base.LightSequence;

public class Lights extends SubsystemBase {
    private final SerialPort port;
    private LightSequence sequence;

    public Lights() {
        port = new SerialPort(Constants.Lights.SERIAL_BAUD_RATE, Constants.Lights.SERIAL_CHANNEL);
            port.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
            port.setWriteBufferSize(1);

        sequence = LightSequence.kDefault;
    }

    public void setSequence(LightSequence sequence) {
        port.write(new byte[sequence.getId()], 1);
    }

    public LightSequence getSequence() {
        return sequence;
    }
}
