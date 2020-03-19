package com.team1323.lib.math.vectors;

import com.team1323.lib.geometry.UnwrappableTranslation2d;

public interface IVectorField {
	public abstract UnwrappableTranslation2d getVector(UnwrappableTranslation2d here);
}