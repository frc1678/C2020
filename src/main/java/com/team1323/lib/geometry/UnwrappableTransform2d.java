/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.lib.geometry;

import java.util.Objects;

/**
 * Represents a transformation for a Pose2d.
 */
public class UnwrappableTransform2d {
  private final UnwrappableTranslation2d m_translation;
  private final UnwrappableRotation2d m_rotation;

  /**
   * Constructs the transform that maps the initial pose to the final pose.
   *
   * @param initial The initial pose for the transformation.
   * @param last    The final pose for the transformation.
   */
  public UnwrappableTransform2d(UnwrappablePose2d initial, UnwrappablePose2d last) {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    m_translation = last.getTranslation().translateBy(initial.getTranslation().inverse())
        .rotateBy(initial.getRotation().inverse());

    m_rotation = last.getRotation().rotateBy(initial.getRotation().inverse());
  }

  /**
   * Constructs a transform with the given translation and rotation components.
   *
   * @param translation Translational component of the transform.
   * @param rotation    Rotational component of the transform.
   */
  public UnwrappableTransform2d(UnwrappableTranslation2d translation, UnwrappableRotation2d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  /**
   * Constructs the identity transform -- maps an initial pose to itself.
   */
  public UnwrappableTransform2d() {
    m_translation = new UnwrappableTranslation2d();
    m_rotation = new UnwrappableRotation2d();
  }

  /**
   * Scales the transform by the scalar.
   *
   * @param scalar The scalar.
   * @return The scaled Transform2d.
   */
  public UnwrappableTransform2d times(double scalar) {
    return new UnwrappableTransform2d(m_translation.scale(scalar), m_rotation.times(scalar));
  }

  /**
   * Returns the translation component of the transformation.
   *
   * @return The translational component of the transform.
   */
  public UnwrappableTranslation2d getTranslation() {
    return m_translation;
  }

  /**
   * Returns the rotational component of the transformation.
   *
   * @return Reference to the rotational component of the transform.
   */
  public UnwrappableRotation2d getRotation() {
    return m_rotation;
  }

  @Override
  public String toString() {
    return String.format("Transform2d(%s, %s)", m_translation, m_rotation);
  }

  /**
   * Checks equality between this Transform2d and another object.
   *
   * @param obj The other object.
   * @return Whether the two objects are equal or not.
   */
  @Override
  public boolean equals(Object obj) {
    if (obj instanceof UnwrappableTransform2d) {
      return ((UnwrappableTransform2d) obj).m_translation.equals(m_translation)
          && ((UnwrappableTransform2d) obj).m_rotation.equals(m_rotation);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(m_translation, m_rotation);
  }
}
