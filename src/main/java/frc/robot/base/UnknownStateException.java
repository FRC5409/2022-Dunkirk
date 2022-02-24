package frc.robot.base;

public class UnknownStateException extends RuntimeException {
    public UnknownStateException() {
        super();
    }

    public UnknownStateException(String s) {
        super(s);
    }

    public UnknownStateException(String message, Throwable cause) {
        super(message, cause);
    }

    public UnknownStateException(Throwable cause) {
        super(cause);
    }
}
