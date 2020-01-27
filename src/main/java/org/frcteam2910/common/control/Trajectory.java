package org.frcteam2910.common.control;

import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Rotation2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Trajectory {
    private final Path path;

    private final double duration;

    private List<ConstrainedPathState> constrainedPathStates = new ArrayList<>();
    private double[] pathStateStartTimes;

    public Trajectory(Path path, TrajectoryConstraint[] trajectoryConstraints, double sampleDistance) {
        this.path = path;

        initialForwardPass(0.0, trajectoryConstraints, sampleDistance);
        initialReversePass(0.0, trajectoryConstraints);

        pathStateStartTimes = new double[constrainedPathStates.size()];

        double duration = 0.0;
        for (int i = 0; i < constrainedPathStates.size(); i++) {
            pathStateStartTimes[i] = duration;
            duration += constrainedPathStates.get(i).getDuration();
        }
        this.duration = duration;
    }

    private void initialForwardPass(double trajectoryStartingVelocity, TrajectoryConstraint[] constraints, double sampleDistance) {
        double distance = 0.0;
        ConstrainedPathState lastState = new ConstrainedPathState(
                path.calculate(distance),
                0.0,
                0.0,
                trajectoryStartingVelocity,
                0.0,
                path.getRotationMap().firstEntry().getValue(),
                0.0,
                0.0,
                0.0
        );
        while (distance < path.getLength()) {
            double profileLength = sampleDistance;
            if (distance + sampleDistance > path.getLength()) {
                profileLength = path.getLength() - distance;
            }

            Path.State profileStartState = path.calculate(distance);
            Path.State profileEndState = path.calculate(distance + profileLength);

            double maxProfileEndVelocity = Arrays.stream(constraints)
                    .mapToDouble(c -> c.getMaxVelocity(profileEndState))
                    .min().orElseThrow(() -> new RuntimeException("Unable to determine max velocity for path state at end of profile"));

            // Create a constrained state ignoring rotation. Rotation will be handled in a later pass
            var state = new ConstrainedPathState(
                    profileStartState,
                    profileLength,
                    lastState.endingVelocity,
                    maxProfileEndVelocity,
                    0.0,
                    Rotation2.ZERO,
                    0.0,
                    0.0,
                    0.0
            );

            // If the max ending velocity is lower than the starting velocity we know that we have to decelerate
            double maxDeltaVelocity = maxProfileEndVelocity - state.startingVelocity;

            // Calculate the optimal acceleration for this profile
            double optimalAcceleration = Math.pow(maxDeltaVelocity, 2.0) / (2.0 * profileLength) + (state.startingVelocity / profileLength) * maxDeltaVelocity;
            if (MathUtils.epsilonEquals(optimalAcceleration, 0.0)) {
                // We are neither accelerating or decelerating
                state.acceleration = 0.0;
                state.endingVelocity = state.startingVelocity;
            } else if (optimalAcceleration > 0.0) {
                // We are accelerating
                double maxStartingAcceleration = Arrays.stream(constraints)
                        .mapToDouble(c -> c.getMaxAcceleration(profileStartState, state.startingVelocity))
                        .min().orElseThrow(() -> new RuntimeException("Unable to determine max acceleration for path state at start of profile"));
                double maxEndingAcceleration = Arrays.stream(constraints)
                        .mapToDouble(c -> c.getMaxAcceleration(profileEndState, state.startingVelocity)) // TODO: Find a way to use the ending velocity
                        .min().orElseThrow(() -> new RuntimeException("Unable to determine max acceleration for path state at end of profile"));

                // Take the lower of the two accelerations
                double acceleration = Math.min(maxStartingAcceleration, maxEndingAcceleration);

                // Use the optimal acceleration if we can
                acceleration = Math.min(acceleration, optimalAcceleration);

                // Find the maximum velocity we can reach during this profile
                double[] roots = MathUtils.quadratic(0.5 * acceleration, state.startingVelocity, -profileLength);
                double duration = Math.max(roots[0], roots[1]);

                state.endingVelocity = state.startingVelocity + acceleration * duration;
                state.acceleration = acceleration;
            } else {
                // If we need to decelerate before we reach the end of the profile than do it.
                // This acceleration may not be achievable. When we go over the trajectory in the reverse path we will
                // take care of this.
                state.acceleration = optimalAcceleration;
            }

            constrainedPathStates.add(state);
            lastState = state;

            distance += profileLength;
        }
    }

    private void initialReversePass(double trajectoryEndingVelocity, TrajectoryConstraint[] constraints) {
        for (int i = constrainedPathStates.size() - 1; i >= 0; i--) {
            var constrainedState = constrainedPathStates.get(i);

            constrainedState.endingVelocity = trajectoryEndingVelocity;
            if (i != constrainedPathStates.size() - 1) {
                constrainedState.endingVelocity = constrainedPathStates.get(i + 1).startingVelocity;
            }

            // Check if we are decelerating
            double deltaVelocity = constrainedState.endingVelocity - constrainedState.startingVelocity;
            if (deltaVelocity > 0.0) {
                // If we are not decelerating skip this state
                continue;
            }

            // Use the deceleration constraint for when we decelerate
            double deceleration = Arrays.stream(constraints)
                    .mapToDouble(c -> c.getMaxDeceleration(constrainedState.pathState, constrainedState.endingVelocity))
                    .min().orElseThrow(() -> new RuntimeException("Unable to determine max deceleration for path state at end of profile"));
            // Find how long it takes for us to decelerate to the ending velocity
            double decelTime = deltaVelocity / -deceleration;

            // Find how far we travel while decelerating
            double decelDist = 0.5 * deceleration * Math.pow(decelTime, 2.0) + constrainedState.endingVelocity * decelTime;

            // If we travel too far we have to decrease the starting velocity
            if (decelDist > constrainedState.length) {
                // We can't decelerate in time. Change the starting velocity of the segment so we can.
                double[] roots = MathUtils.quadratic(0.5 * deceleration, constrainedState.endingVelocity, -constrainedState.length);

                // Calculate the maximum time that we can decelerate for
                double maxAllowableDecelTime = Math.max(roots[0], roots[1]);

                // Find what our starting velocity can be in order to end at our ending velocity
                constrainedState.acceleration = -deceleration;
                constrainedState.startingVelocity = constrainedState.endingVelocity + deceleration * maxAllowableDecelTime;
            }
        }
    }

    public State calculate(double time) {
        int start = 0;
        int end = constrainedPathStates.size() - 1;
        int mid = start + (end - start) / 2;
        while (start <= end) {
            mid = (start + end) / 2;

            if (time > pathStateStartTimes[mid] + constrainedPathStates.get(mid).getDuration()) {
                start = mid + 1;
            } else if (time < pathStateStartTimes[mid]) {
                end = mid - 1;
            } else {
                break;
            }
        }

        ConstrainedPathState constrainedPathState = constrainedPathStates.get(mid);
        return constrainedPathState.calculate(time - pathStateStartTimes[mid]);
    }

    public double getDuration() {
        return duration;
    }

    public Path getPath() {
        return path;
    }

    class ConstrainedPathState {
        public Path.State pathState;
        public double length;
        public double startingVelocity;
        public double endingVelocity;
        public double acceleration;

        public Rotation2 startingRotation;
        public double startingAngularVelocity;
        public double endingAngularVelocity;
        public double angularAcceleration;

        public ConstrainedPathState(Path.State pathState, double length, double startingVelocity, double endingVelocity, double acceleration,
                                    Rotation2 startingRotation, double startingAngularVelocity, double endingAngularVelocity, double angularAcceleration) {
            this.pathState = pathState;
            this.length = length;
            this.startingVelocity = startingVelocity;
            this.endingVelocity = endingVelocity;
            this.acceleration = acceleration;

            this.startingRotation = startingRotation;
            this.startingAngularVelocity = startingAngularVelocity;
            this.endingAngularVelocity = endingAngularVelocity;
            this.angularAcceleration = angularAcceleration;
        }

        public double getDuration() {
            if (MathUtils.epsilonEquals(acceleration, 0.0)) {
                return length / startingVelocity;
            }

            if (MathUtils.epsilonEquals(endingVelocity, 0.0)) {
                return (startingVelocity / -acceleration);
            }

            double[] roots = MathUtils.quadratic(0.5 * acceleration, startingVelocity, -length);

            if (acceleration > 0.0) {
                return Math.max(roots[0], roots[1]);
            } else {
                return Math.min(roots[0], roots[1]);
            }
        }

        public State calculate(double time) {
            time = MathUtils.clamp(time, 0.0, getDuration());

            double distance = 0.5 * acceleration * Math.pow(time, 2.0) + startingVelocity * time + pathState.getDistance();

            Rotation2 rotation = startingRotation.rotateBy(
                    Rotation2.fromRadians(0.5 * angularAcceleration * Math.pow(time, 2.0) + startingAngularVelocity * time)
            );
            return new State(
                    path.calculate(distance),
                    acceleration * time + startingVelocity,
                    acceleration,
                    rotation,
                    angularAcceleration * time + startingAngularVelocity,
                    angularAcceleration
            );
        }
    }

    public static class State {
        // Linear motion
        private final Path.State pathState;
        private final double velocity;
        private final double acceleration;

        // Angular motion
        private final Rotation2 rotation;
        private final double angularVelocity;
        private final double angularAcceleration;

        public State(Path.State pathState, double velocity, double acceleration,
                     Rotation2 rotation, double angularVelocity, double angularAcceleration) {
            this.pathState = pathState;
            this.velocity = velocity;
            this.acceleration = acceleration;

            this.rotation = rotation;
            this.angularVelocity = angularVelocity;
            this.angularAcceleration = angularAcceleration;
        }

        public Path.State getPathState() {
            return pathState;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getAcceleration() {
            return acceleration;
        }

        public Rotation2 getRotation() {
            return rotation;
        }

        public double getAngularVelocity() {
            return angularVelocity;
        }

        public double getAngularAcceleration() {
            return angularAcceleration;
        }
    }
}
