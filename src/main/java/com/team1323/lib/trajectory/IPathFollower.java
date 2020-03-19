package com.team1323.lib.trajectory;

import com.team1323.lib.geometry.UnwrappablePose2d;
import com.team1323.lib.geometry.UnwrappableTwist2d;

public interface IPathFollower {
    public UnwrappableTwist2d steer(UnwrappablePose2d current_pose);

    public boolean isDone();
}
