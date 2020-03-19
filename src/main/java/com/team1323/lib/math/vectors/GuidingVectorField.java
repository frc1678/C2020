package com.team1323.lib.math.vectors;

import java.util.function.Function;

import com.team1323.lib.geometry.UnwrappableTranslation2d;

public class GuidingVectorField extends VectorField {
	// Implement as e.g.
	// GuidingVectorField(UnwrappableTranslation2d here -> here.x()+here.y(), ... )
/*	public GuidingVectorField(Function<UnwrappableTranslation2d,Double> surface,
				  Function<UnwrappableTranslation2d,Double> dfdx,
				  Function<UnwrappableTranslation2d,Double> dfdy) {
		phi = surface;
		dfdx_ = dfdx;
		dfdy_ = dfdy;
	}
	public GuidingVectorField(Function<UnwrappableTranslation2d,Double> surface,
				  Function<UnwrappableTranslation2d,Double> dfdx,
				  Function<UnwrappableTranslation2d,Double> dfdy,
				  boolean isReversed) {
		phi = surface;
		dfdx_ = dfdx;
		dfdy_ = dfdy;
		if(isReversed) direction = -1;
	}*/
	public GuidingVectorField(Surface surface_, boolean isReversed, Function<UnwrappableTranslation2d,Double> k_) {
		surface = surface_;
		if(isReversed) direction = -1;
		k = k_;
	}
	public GuidingVectorField(Surface surface_) {
		this(surface_, false, here -> 1.0);
	}
	public GuidingVectorField(Surface surface_, boolean isReversed) {
		this(surface_, isReversed, here -> 1.0);
	}
	public GuidingVectorField(Surface surface_, Function<UnwrappableTranslation2d,Double> k_) {
		this(surface_, false, k_);
	}
	
		protected int direction = 1;
/*	protected Function<UnwrappableTranslation2d,Double> phi;
	protected Function<UnwrappableTranslation2d,Double> dfdx_;
	protected Function<UnwrappableTranslation2d,Double> dfdy_;*/
	protected Surface surface;
	protected UnwrappableTranslation2d n(UnwrappableTranslation2d here) {
		UnwrappableTranslation2d nv = new UnwrappableTranslation2d(surface.dfdx().apply(here),surface.dfdy().apply(here));
//		System.out.println("n" + here + " = " + nv);
		return nv;
	}
	// Is psi even necessary? Since k is a function now, idk
	protected double psi(double t) { return t; } // FIXME: pass this in constructor, definitely (type is known)
	protected double e(UnwrappableTranslation2d here) { return psi(surface.f().apply(here)); } // surface.f is phi from the paper
	protected Function<UnwrappableTranslation2d,Double> k;
	protected UnwrappableTranslation2d tau(UnwrappableTranslation2d here) {
		UnwrappableTranslation2d nv = n(here);
		UnwrappableTranslation2d tv = new UnwrappableTranslation2d(nv.y()*direction,-nv.x()*direction);
//		System.out.println("t" + here + " = " + tv);
		return tv;
	}
	public UnwrappableTranslation2d getVector(UnwrappableTranslation2d here) {
		return (new UnwrappableTranslation2d(n(here).scale(-k.apply(here)*e(here)),tau(here))).normalize();
	}
}