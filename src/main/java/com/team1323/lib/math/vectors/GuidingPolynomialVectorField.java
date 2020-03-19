package com.team1323.lib.math.vectors;

import java.util.function.Function;
import com.team1323.lib.geometry.UnwrappableTranslation2d;

public class GuidingPolynomialVectorField extends VectorField {

	// Create a directed line that contains the point (x,y) and has a given heading
/*	public GuidingPolynomialVectorField(UnwrappableTranslation2d where, Rotation2d heading) {
		fc = new double[]{where.y() - heading.tan()*where.x(),heading.tan()};
		setDfc(fc);
		if(heading.getRadians() >= Math.PI) direction = -1;
	}*/
	public GuidingPolynomialVectorField(double[] coeffs, boolean isReversed, Function<UnwrappableTranslation2d,Double> k_) {
		fc = coeffs;
		setDfc(fc);
		if(isReversed) direction = -1;
		k = k_;
	}
	public GuidingPolynomialVectorField(double[] coeffs) {
		this(coeffs, false, here -> 1.0);
	}
	public GuidingPolynomialVectorField(double[] coeffs, Function<UnwrappableTranslation2d,Double> k_) {
		this(coeffs, false, k_);
	}
	protected void setDfc(double[] fc) {
		dfc = new double[fc.length - 1];
		for(int i = 0; i < dfc.length; i++) {
			dfc[i] = fc[i+1]*(i+1);
		}
	}
	protected int direction = 1;
	protected double[] fc; // f[n] is coefficient for x^n term
	protected double[] dfc;
	protected double f(double[] cs, double x) {
		int deg = cs.length-1;
		double val = cs[deg];
		while(deg > 0) {
			val *= x;
			val += cs[--deg];
		}
		return val;
	}

	protected double phi(UnwrappableTranslation2d here) { return here.y() - f(fc,here.x()); }
	protected UnwrappableTranslation2d n(UnwrappableTranslation2d here) { return new UnwrappableTranslation2d(-f(dfc,here.x()),1); }
	protected double psi(double t) { return t; }
	protected double e(UnwrappableTranslation2d here) { return psi(phi(here)); }
	/** Lower values of k mean more following, less approaching */
	protected Function<UnwrappableTranslation2d,Double> k;
	protected UnwrappableTranslation2d tau(UnwrappableTranslation2d here) {
		UnwrappableTranslation2d nv = n(here);
		return new UnwrappableTranslation2d(nv.y()*direction,-nv.x()*direction);
	}
	public UnwrappableTranslation2d getVector(UnwrappableTranslation2d here) {
		return (new UnwrappableTranslation2d(n(here).scale(k.apply(here)*e(here)),tau(here))).normalize();
	}
}