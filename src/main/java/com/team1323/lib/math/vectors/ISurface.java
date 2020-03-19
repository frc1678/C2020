package com.team1323.lib.math.vectors;

import java.util.function.Function;

import com.team1323.lib.geometry.UnwrappableTranslation2d;

public interface ISurface {
	public abstract Function<UnwrappableTranslation2d,Double> f();
	public abstract Function<UnwrappableTranslation2d,Double> dfdx();
	public abstract Function<UnwrappableTranslation2d,Double> dfdy();
}