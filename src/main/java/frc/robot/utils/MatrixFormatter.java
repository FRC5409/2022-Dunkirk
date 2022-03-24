package frc.robot.utils;

import java.util.Collection;

public class MatrixFormatter {
    public static <T extends Number>
    String format(Collection<T> values, String format, int rows, int cols) {
        if (rows == 0 || cols == 0 || values.isEmpty())
            throw new IllegalArgumentException("Cannot format matrix with zero length");

        final int length = rows*cols;
        if (values.size() != length)
            throw new IllegalArgumentException("Expected matrix size of " + length + ", got " + values.size() + " instead.");
        
        StringBuilder sb = new StringBuilder();

        // Prepare table information
        String valueString[] = new String[length];
        int valueSeperator[] = new int[length];
        int valueLength[] = new int[length];

        int columnLeftIndent[] = new int[cols];
        int columnLength[] = new int[cols];

        int i = 0;
        for (T value : values) {
            int column = i % cols;

            String str = String.format(format, value);
            int len = str.length();

            int seperator = str.indexOf('.');

            // If the seperator does not exist, act as
            // if it were placed at the end of the value
            // E.g.   10  -> 10.
            if (seperator == -1)
                seperator = len;

            // Check if the value has a greater left indent
            // than the column (derived from seperator position)
            if (seperator > columnLeftIndent[column])
                columnLeftIndent[column] = seperator;

            valueString[i] = str;
            valueLength[i] = len;
            valueSeperator[i] = seperator;

            i++;
        }

        for (i = 0; i < length; i++) {
            int column = i % cols;

            // Calculate full value length (including left indent)
            int len = valueLength[i] + affix(columnLeftIndent[column] - valueSeperator[i]);

            // Check if the value has a greater length than the
            // column length
            if (len > columnLength[column])
                columnLength[column] = len;
        }

        for (i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                int k = i*cols+j;

                // Calculate left identation
                int leftIndent = affix(columnLeftIndent[j] - valueSeperator[k]);
                
                // Append left indentation
                repeat(sb, ' ', leftIndent);

                // Append value
                sb.append(valueString[k]);

                // Append right indentation
                repeat(sb, ' ', affix(columnLength[j] - valueLength[k] - leftIndent) + 1);
            }
            sb.append('\n');
        }

        // Remove very last newline character
        sb.deleteCharAt(sb.length()-1);

        return sb.toString();
    }

    private static void repeat(StringBuilder sb, char character, int count) {
        for (int i = 0; i < count; i++)
            sb.append(character);
    }

    private static int affix(int x) {
        return (x > 0) ? x : 0;
    }
}
