package com.team1323.lib.math.vectors;

import com.team1323.lib.geometry.UnwrappableTranslation2d;

public class RadialVectorField extends VectorField {
	// Default direction is toward point
	public RadialVectorField(UnwrappableTranslation2d where) {
		there = where;
	}
	public RadialVectorField(UnwrappableTranslation2d where, boolean away) {
		there = where;
		if(away) direction = -1;
	}
	
	protected int direction = 1;
	protected UnwrappableTranslation2d there;
	
	public UnwrappableTranslation2d getVector(UnwrappableTranslation2d here) {
		UnwrappableTranslation2d v = new UnwrappableTranslation2d(here,there);
		return v.scale(direction/v.norm());
	}
}