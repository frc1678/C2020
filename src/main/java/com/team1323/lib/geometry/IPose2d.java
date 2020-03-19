package com.team1323.lib.geometry;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    public UnwrappablePose2d getPose();

    public S transformBy(UnwrappablePose2d transform);

    public S mirror();
}
