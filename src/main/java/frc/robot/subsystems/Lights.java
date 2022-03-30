package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.base.lights.LEDAnimation;
import frc.robot.base.lights.AnimationID;

public class Lights extends SubsystemBase {



    private final SerialPort port;
    private LightSequence sequence;

    private Map animations 

    public Lights() {
        port = new SerialPort(Constants.Lights.SERIAL_BAUD_RATE, Constants.Lights.SERIAL_CHANNEL);
            port.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
            port.setWriteBufferSize(1);

        sequence = LightSequence.kDefault;
        /**
         * byte format
         * start animation, id
         * 01 00000
         * 0000000 time scale
         * 
         * set animation, id
         * 00 00000
         * 
         * set led
         * 
         * led num, timestamp
         *  00000000
         *  00000000
         * 
         * green
         *  00000000
         * 
         * red
         *  00000000
         * 
         * blue 
         *  00000000
         */  
    }

    public void createAnimations(){

    }

    private void addAnimation(AnimationID id, LEDAnimation animation){

    }

    public void startAnimation(AnimationID id) {
        port.write(new byte[]{(byte)(id.getId() | (byte)0b01000000)}, 1);
    }

    
}
