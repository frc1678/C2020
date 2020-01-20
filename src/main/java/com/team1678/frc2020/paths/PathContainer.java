package com.team1678.frc2020.paths;

import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.spline.Spline;

/**
 * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
 * whether or not the robot should drive in reverse along the path.
 */
public interface PathContainer {
    List<Spline> buildPath();

    Pose2d getStartPose();

    boolean isReversed();
}
