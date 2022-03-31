package frc.robot.base.command;
import java.util.regex.Pattern;

import org.jetbrains.annotations.Nullable;

public final class StateFormat {
    private static final String SEPERATOR = ":";
    private static final Pattern PATH_FORMAT_MATCHER = Pattern.compile("^([^"+SEPERATOR+"]+"+SEPERATOR+"?)+(?<!"+SEPERATOR+")$");
    private static final Pattern NAME_FORMAT_MATCHER = Pattern.compile("^[^"+SEPERATOR+"]+$");

    @Nullable
    public static final void validateName(String name) throws IllegalArgumentException {
        if (name.isEmpty())
            throw new IllegalArgumentException("Invalid empty state name.");
        else if (!NAME_FORMAT_MATCHER.matcher(name).matches())
            throw new IllegalArgumentException("Invalid state name '" + name + "'");
    }

    @Nullable
    public static final String[] parsePath(String path) throws IllegalArgumentException {
        if (path.isEmpty())
            throw new IllegalArgumentException("Invalid empty state path.");
        else if (!PATH_FORMAT_MATCHER.matcher(path).matches())
            throw new IllegalArgumentException("Invalid state path '" + path + "'");

        return path.split(SEPERATOR);
    }

    public static final String formatPath(String[] path) throws IllegalArgumentException {
        if (path.length == 0)
            throw new IllegalArgumentException("Invalid empty state path.");

        StringBuilder builder = new StringBuilder();
        for (String name : path) {
            builder.append(name);
            builder.append(SEPERATOR);
        }

        builder.deleteCharAt(builder.length()-1);

        return builder.toString();
    }
}
