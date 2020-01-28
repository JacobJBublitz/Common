package org.frcteam2910.common.control;

public class MaxAngularVelocityConstraint extends TrajectoryConstraint {
    private final double maxAngularVelocity;

    public MaxAngularVelocityConstraint(double maxAngularVelocity) {
        this.maxAngularVelocity = maxAngularVelocity;
    }

    @Override
    public double getMaxAngularVelocity(Path.State state, double velocity) {
        return maxAngularVelocity;
    }
}
