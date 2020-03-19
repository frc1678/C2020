package com.team1323.lib.math.vectors;

import java.util.function.Function;
import com.team1323.lib.geometry.UnwrappableTranslation2d;

public class GeneralVectorField extends VectorField {
  public GeneralVectorField(Function<UnwrappableTranslation2d,UnwrappableTranslation2d> field) {
    field_ = field;
  }
  protected Function<UnwrappableTranslation2d,UnwrappableTranslation2d> field_;
  public UnwrappableTranslation2d getVector(UnwrappableTranslation2d here) {
	  return field_.apply(here).normalize();
  }
}