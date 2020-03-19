package com.team1323.lib.math.vectors;

import com.team1323.lib.geometry.UnwrappableTranslation2d;

public class ConstantVectorField extends VectorField {
	public ConstantVectorField(UnwrappableTranslation2d whichWay) {
		thatWay = whichWay.normalize();
	}
	
	protected UnwrappableTranslation2d thatWay;
	
	public UnwrappableTranslation2d getVector(UnwrappableTranslation2d here) {
		return thatWay;
	}
}