package frc.robot.base;

class SimpleName implements Named {
    private final String name;

    public SimpleName(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }
}
