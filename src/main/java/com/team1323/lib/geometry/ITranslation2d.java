package com.team1323.lib.geometry;

import com.team254.lib.geometry.State;

public interface ITranslation2d<S> extends State<S> {
    public UnwrappableTranslation2d getTranslation();
}
