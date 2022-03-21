package frc.robot.utils;

import java.util.Arrays;
import java.util.stream.Collectors;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

// TODO docs
public class Matrix3 implements Sendable {
    public static final int SIZE = 9;

    private double m[];

    public static Matrix3 identity() {
        return new Matrix3();
    }

    public Matrix3() {
        m = new double[] {1,0,0,0,1,0,0,0,1};
    }

    public Matrix3(
        double m00, double m01, double m02,
        double m10, double m11, double m12,
        double m20, double m21, double m22
    ) { 
        m = new double[] {m00,m01,m02,m10,m11,m12,m20,m21,m22};
    }

    public Matrix3(double[] components) {
        assert components.length == SIZE;
        m = new double[SIZE];
        System.arraycopy(components, 0, m, 0, SIZE);
    }

    public Vector3 apply(Vector3 k) {
        return new Vector3(
            m[0]*k.x + m[1]*k.y + m[2]*k.z,
            m[3]*k.x + m[4]*k.y + m[5]*k.z,
            m[6]*k.x + m[7]*k.y + m[8]*k.z
        );
    }

    public Matrix3 apply(Matrix3 k) {
        return new Matrix3(
            m[0]*k.m[0] + m[1]*k.m[3] + m[2]*k.m[6],
            m[0]*k.m[1] + m[1]*k.m[4] + m[2]*k.m[7],
            m[0]*k.m[2] + m[1]*k.m[5] + m[2]*k.m[8],
            m[3]*k.m[0] + m[4]*k.m[3] + m[5]*k.m[6],
            m[3]*k.m[1] + m[4]*k.m[4] + m[5]*k.m[7],
            m[3]*k.m[2] + m[4]*k.m[5] + m[5]*k.m[8],
            m[6]*k.m[0] + m[7]*k.m[3] + m[8]*k.m[6],
            m[6]*k.m[1] + m[7]*k.m[4] + m[8]*k.m[7],
            m[6]*k.m[2] + m[7]*k.m[5] + m[8]*k.m[8]
        );
    }
    
    public Matrix3 inverse() throws ArithmeticException {
        final double d = determinant();
        if (d == 0.0) {
            throw new ArithmeticException("Cannot invert matrix with determinant of 0.");
        }
        
        return new Matrix3(
            m[4]*m[8] - m[5]*m[7], m[2]*m[7] - m[1]*m[8], m[1]*m[5] - m[2]*m[4],
            m[5]*m[6] - m[3]*m[8], m[0]*m[8] - m[2]*m[6], m[2]*m[3] - m[0]*m[5],
            m[3]*m[7] - m[4]*m[6], m[1]*m[6] - m[0]*m[7], m[0]*m[4] - m[1]*m[3]
        );
    };
    
    public double determinant() {
        return ( m[0] * (m[4] * m[8] - m[7] * m[5]) - 
                 m[3] * (m[1] * m[8] - m[7] * m[2]) +
                 m[6] * (m[1] * m[5] - m[4] * m[2]) ); 
    };
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("components", () -> m, null);
    }

    @Override
    public String toString() {
        return MatrixFormatter.format(
            Arrays.stream(m).boxed().collect(Collectors.toList()), "%.3f", 3, 3);
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Matrix3)) return false;
        Matrix3 other = (Matrix3) o;
        return Arrays.equals(m, other.m);
    }
}
