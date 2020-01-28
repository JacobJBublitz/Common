package org.frcteam2910.common.control;

public class MaxAngularAccelerationConstraint extends TrajectoryConstraint {
    private final double maxAngularAcceleration;

    public MaxAngularAccelerationConstraint(double maxAngularAcceleration) {
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    @Override
    public double getMaxAngularAcceleration(Path.State state, double velocity, double angularVelocity) {
        return maxAngularAcceleration;
    }
}
