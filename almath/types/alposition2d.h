/*
** Author(s):
**  - Chris Kilner
**  - Cyrille Collette
**  - David Gouaillier
**
** Copyright (C) 2011 Aldebaran Robotics
*/

#pragma once

#ifndef _LIB_ALMATH_ALMATH_ALPOSITION2D_H_
#define _LIB_ALMATH_ALMATH_ALPOSITION2D_H_

#include <vector>

namespace AL {
  namespace Math {

    /// <summary>
    /// Create and play with a Position2D.
    ///
    /// A Position2D is just defined by x and y.
    /// </summary>
    /// \ingroup Types
    struct Position2D
    {
      /// <summary> </summary>
      float x;
      /// <summary> </summary>
      float y;

      /// <summary>
      /// Create a Position2D initialized with 0.0f.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         x \\
       *         y
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         0.0 \\
       *         0.0
       *      \end{array}\right]\f$
       */
      /// </summary>
      Position2D();

      /// <summary>
      /// Create a Position2D initialize with the same float.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         x \\
       *         y
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pInit \\
       *         pInit
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pInit"> the float value for each member </param>
      /// </summary>
      explicit Position2D(float pInit);

      /// <summary>
      /// Create a Position2D initialized with explicit value.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         x \\
       *         y
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pX \\
       *         pY
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pX"> the float value for x </param>
      /// <param name="pY"> the float value for y </param>
      Position2D(float pX, float pY);

      /// <summary>
      /// Create a Position2D with an std::vector.
      /**
       *
       * \f$ \left[\begin{array}{c}
       *         x \\
       *         y
       *      \end{array}\right] =
       *      \left[\begin{array}{c}
       *         pFloats[0] \\
       *         pFloats[1]
       *      \end{array}\right]\f$
       */
      /// </summary>
      /// <param name="pFloats">
      /// An std::vector<float> of size 2 for respectively:
      /// x and y
      /// </param>
      Position2D(const std::vector<float>& pFloats);

      /// <summary>
      /// Overloading of operator + for Position2D.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      Position2D operator+ (const Position2D& pPos2) const;

      /// <summary>
      /// Overloading of operator - for Position2D.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      Position2D operator- (const Position2D& pPos2) const;

      /// <summary>
      /// Overloading of operator + for Position2D.
      /// </summary>
      Position2D operator+ () const;

      /// <summary>
      /// Overloading of operator - for Position2D.
      /// </summary>
      Position2D operator- () const;

      /// <summary>
      /// Overloading of operator += for Position2D.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      Position2D& operator+= (const Position2D& pPos2);

      /// <summary>
      /// Overloading of operator -= for Position2D.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      Position2D& operator-= (const Position2D& pPos2);

      /// <summary>
      /// Overloading of operator == for Position2D.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      /// <returns>
      /// true if each float of the two Position2D are equal
      /// </returns>
      bool operator==(const Position2D& pPos2) const;

      /// <summary>
      /// Overloading of operator != for Position2D.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      /// <returns>
      /// true if one of each float of the two Position2D are not equal
      /// </returns>
      bool operator!=(const Position2D& pPos2) const;

      /// <summary>
      /// Overloading of operator * for Position2D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Position2D operator* (float pVal) const;

      /// <summary>
      /// Overloading of operator / for Position2D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Position2D operator/ (float pVal) const;

      /// <summary>
      /// Overloading of operator *= for Position2D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Position2D& operator*= (float pVal);

      /// <summary>
      /// Overloading of operator /= for Position2D.
      /// </summary>
      /// <param name="pVal"> the float factor </param>
      Position2D& operator/= (float pVal);

      /// <summary>
      /// Compute the squared distance between the actual
      /// Position2D and the one give in argument.
      ///
      /// \f$(pPos1.x-pPos2.x)^2+(pPos1.y-pPos2.y)^2\f$
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      /// <returns>
      /// the float squared distance between the two Position2D
      /// </returns>
      float distanceSquared(const Position2D& pPos2) const;

      /// <summary>
      /// Compute the distance between the actual
      /// Position2D and the one give in argument.
      ///
      /// \f$\sqrt{(pPos1.x-pPos2.x)^2+(pPos1.y-pPos2.y)^2}\f$
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      /// <returns>
      /// the float distance between the two Position2D
      /// </returns>
      float distance(const Position2D& pPos2) const;

      /// <summary>
      /// Check if the actual Position2D is near the one
      /// give in argument.
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      /// <param name="pEpsilon"> an optionnal epsilon distance </param>
      /// <returns>
      /// true if the distance between the two Position2D is less than pEpsilon
      /// </returns>
      bool isNear(
        const Position2D& pPos2,
        const float&      pEpsilon=0.0001f) const;

      /// <summary>
      /// Compute the norm of the actual Position2D.
      ///
      /// \f$\sqrt{pPos.x^2+pPos.y^2}\f$
      /// </summary>
      /// <returns>
      /// the float norm of the Position2D
      /// </returns>
      float norm() const;

      /// <summary>
      /// Normalize the actual Position2D.
      ///
      /// \f$pRes = \frac{pPos}{norm(pPos)}\f$
      /// </summary>
      /// <returns>
      /// the Position2D normalized
      /// </returns>
      Position2D normalize() const;

      /// <summary>
      /// Compute the cross Product between the actual
      /// Position2D and the one give in argument.
      ///
      /// \f$pRes = (pPos1.x*pPos2.y - pPos1.y*pPos2.x)\f$
      /// </summary>
      /// <param name="pPos2"> the second Position2D </param>
      /// <returns>
      /// the float cross product between the two Position2D
      /// </returns>
      float crossProduct(const Position2D& pPos2) const;

      /// <summary>
      /// Return the Position2D as a vector of float [x, y].
      /// </summary>
      std::vector<float> toVector() const;
    };

    // TODO : Need this ?
    Position2D operator* (
      const float       pM,
      const Position2D& pPos1);

    /// <summary>
    /// Compute the squared distance between two Position2D.
    ///
    /// \f$(pPos1.x-pPos2.x)^2+(pPos1.y-pPos2.y)^2\f$
    /// </summary>
    /// <param name="pPos1"> the first Position2D </param>
    /// <param name="pPos2"> the second Position2D </param>
    /// <returns>
    /// the float squared distance between the two Position2D
    /// </returns>
    /// \ingroup Types
    float distanceSquared(
      const Position2D& pPos1,
      const Position2D& pPos2);

    /// <summary>
    /// Compute the distance between two Position2D \f$(pPos1,pPos2)\f$:
    ///
    /// \f$\sqrt{(pPos1.x-pPos2.x)^2+(pPos1.y-pPos2.y)^2}\f$
    /// </summary>
    /// <param name="pPos1"> the first Position2D </param>
    /// <param name="pPos2"> the second Position2D </param>
    /// <returns>
    /// the float distance between the two Position2D
    /// </returns>
    /// \ingroup Types
    float distance(
      const Position2D& pPos1,
      const Position2D& pPos2);

    /// <summary>
    /// Compute the norm of a Position2D.
    ///
    /// \f$\sqrt{(pPos.x)^2+(pPos.y)^2}\f$
    /// </summary>
    /// <param name="pPos"> the given Position2D </param>
    /// <returns>
    /// the float norm of the given Position2D
    /// </returns>
    /// \ingroup Types
    float norm(const Position2D& pPos);

    /// <summary>
    /// Normalize a Position2D.
    ///
    /// \f$pRes = \frac{pPos}{norm(pPos)}\f$
    /// </summary>
    /// <param name="pPos"> the given Position2D </param>
    /// <returns>
    /// the given Position2D normalized
    /// </returns>
    /// \ingroup Types
    Position2D normalize(const Position2D& pPos);

    /// <summary>
    /// Compute the cross Product of two Position2D.
    ///
    /// \f$pRes = (pPos1.x*pPos2.y - pPos1.y*pPos2.x)\f$
    /// </summary>
    /// <param name="pPos1"> the first Position2D </param>
    /// <param name="pPos2"> the second Position2D </param>
    /// <returns>
    /// the float cross product between the two Position2D
    /// </returns>
    /// \ingroup Types
    float crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2);

    /// <summary>
    /// Compute the cross Product of two Position2D.
    ///
    /// \f$pRes = (pPos1.x*pPos2.y - pPos1.y*pPos2.x)\f$
    /// </summary>
    /// <param name="pPos1"> the first Position2D </param>
    /// <param name="pPos2"> the second Position2D </param>
    /// <param name="pRes">
    /// the float cross product between the two Position2D </param>
    /// \ingroup Types
    void crossProduct(
      const Position2D& pPos1,
      const Position2D& pPos2,
      float&            pRes);

  } // end namespace math
} // end namespace al
#endif  // _LIB_ALMATH_ALMATH_ALPOSITION2D_H_
