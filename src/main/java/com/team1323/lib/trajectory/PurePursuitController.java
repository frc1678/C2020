package com.team1323.lib.trajectory;

import com.team1323.lib.geometry.*;

public class PurePursuitController<S extends IUnwrappableTranslation2d<S>>{
    protected final TrajectoryIterator<S> iterator_;
    protected final double sampling_dist_;
    protected final double lookahead_;
    protected final double goal_tolerance_;
    protected boolean done_ = false;
    public PurePursuitController(final DistanceView<S> path, double sampling_dist, double lookahead,
                                 double goal_tolerance) {
        sampling_dist_ = sampling_dist;
        lookahead_ = lookahead;
        goal_tolerance_ = goal_tolerance;
        iterator_ = new TrajectoryIterator<S>(path);
    }

    public UnwrappableTranslation2d steer(final UnwrappablePose2d current_pose) {
        done_ = done_ || (iterator_.isDone()
                && current_pose.getTranslation().distance(iterator_.getState().getTranslation()) <= goal_tolerance_);
        if (isDone()) {
            return new UnwrappableTranslation2d();
        }

        final double remaining_progress = iterator_.getRemainingProgress();
        double goal_progress = 0.0;
        // Find the first point > lookahead distance away from current_pose, or the last point otherwise.
        for (double progress = 0.0; progress <= remaining_progress; progress = Math.min(remaining_progress,
                progress + sampling_dist_)) {
            double dist = current_pose.getTranslation().distance(iterator_.preview(progress).state().getTranslation());
            if (dist > lookahead_) {
                if (goal_progress == 0.0 && !iterator_.isDone()) {
                    // Make sure we don't get stuck due to numerical issues when sampling dist is large relative to
                    // lookahead.
                    goal_progress = progress;
                }
                break;
            }
            goal_progress = progress;
            if (progress == remaining_progress) {
                break;
            }
        }
        iterator_.advance(goal_progress);
        return new UnwrappableTranslation2d(current_pose.getTranslation(), iterator_.getState().getTranslation());
    }

    public boolean isDone() {
        return done_;
    }

    protected static <S extends IUnwrappableTranslation2d<S>> double getDirection(UnwrappablePose2d pose, S point) {
        UnwrappableTranslation2d poseToPoint = new UnwrappableTranslation2d(pose.getTranslation(), point.getTranslation());
        UnwrappableTranslation2d robot = pose.getRotation().toTranslation();
        double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
        return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
    }

    public static class Arc<S extends IUnwrappableTranslation2d<S>> {
        public UnwrappableTranslation2d center;
        public double radius;
        public double length;

        public Arc(final UnwrappablePose2d pose, final S point) {
            center = findCenter(pose, point);
            radius = new UnwrappableTranslation2d(center, point.getTranslation()).norm();
            length = findLength(pose, point, center, radius);
            radius *= getDirection(pose, point);
        }

        protected UnwrappableTranslation2d findCenter(UnwrappablePose2d pose, S point) {
            final UnwrappableTranslation2d poseToPointHalfway = pose.getTranslation().interpolate(point.getTranslation(), 0.5);
            final UnwrappableRotation2d normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction()
                    .normal();
            final UnwrappablePose2d perpendicularBisector = new UnwrappablePose2d(poseToPointHalfway, normal);
            final UnwrappablePose2d normalFromPose = new UnwrappablePose2d(pose.getTranslation(),
                    pose.getRotation().normal());
            if (normalFromPose.isColinear(perpendicularBisector.normal())) {
                // Special case: center is poseToPointHalfway.
                return poseToPointHalfway;
            }
            return normalFromPose.intersection(perpendicularBisector);
        }

        protected double findLength(UnwrappablePose2d pose, S point, UnwrappableTranslation2d center, double radius) {
            if (radius < Double.MAX_VALUE) {
                final UnwrappableTranslation2d centerToPoint = new UnwrappableTranslation2d(center, point.getTranslation());
                final UnwrappableTranslation2d centerToPose = new UnwrappableTranslation2d(center, pose.getTranslation());
                // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
                // check the sign of the cross-product between the normal vector and the vector from pose to point.
                final boolean behind = Math.signum(
                        UnwrappableTranslation2d.cross(pose.getRotation().normal().toTranslation(),
                                new UnwrappableTranslation2d(pose.getTranslation(), point.getTranslation()))) > 0.0;
                final UnwrappableRotation2d angle = UnwrappableTranslation2d.getAngle(centerToPose, centerToPoint);
                return radius * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
            } else {
                return new UnwrappableTranslation2d(pose.getTranslation(), point.getTranslation()).norm();
            }
        }
    }
}
