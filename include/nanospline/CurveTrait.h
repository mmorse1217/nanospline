#pragma once

#include <Eigen/Core>

namespace nanospline {

template<typename _Scalar, int _dim>
class SplineBase;

template<typename _Scalar, int _dim, int _degree, bool _generic>
class Bezier;

template<typename _Scalar, int _dim, int _degree, bool _generic>
class RationalBezier;

template<typename _Scalar, int _dim, int _degree, bool _generic>
class BSpline;

template<typename _Scalar, int _dim, int _degree, bool _generic>
class NURBS;

template<typename CurveType>
struct CurveTrait {
};

template<typename _Scalar, int _dim, int _degree, bool _generic>
struct CurveTrait<Bezier<_Scalar, _dim, _degree, _generic> > {
    static_assert(_dim > 0, "Dimension must be positive.");
    static_assert(_degree>=0 || _generic,
            "Invalid degree for non-generic Bezier setting");

    using Scalar = _Scalar;
    using Point = Eigen::Matrix<Scalar, 1, _dim>;
    using ControlPoints = Eigen::Matrix<Scalar, _generic?Eigen::Dynamic:_degree+1, _dim>;
    using Base = SplineBase<_Scalar, _dim>;

    static constexpr int dim = _dim;
    static constexpr int degree = _degree;
    static constexpr bool generic = _generic;
};

template<typename _Scalar, int _dim, int _degree, bool _generic>
struct CurveTrait<RationalBezier<_Scalar, _dim, _degree, _generic> > {
    static_assert(_dim > 0, "Dimension must be positive.");
    static_assert(_degree>=0 || _generic,
            "Invalid degree for non-generic Bezier setting");

    using Scalar = _Scalar;
    using Point = Eigen::Matrix<Scalar, 1, _dim>;
    using ControlPoints = Eigen::Matrix<Scalar, _generic?Eigen::Dynamic:_degree+1, _dim>;
    using WeightVector = Eigen::Matrix<_Scalar, _generic?Eigen::Dynamic:_degree+1, 1>;
    using Base = SplineBase<_Scalar, _dim>;

    static constexpr int dim = _dim;
    static constexpr int degree = _degree;
    static constexpr bool generic = _generic;
};

template<typename _Scalar, int _dim, int _degree, bool _generic>
struct CurveTrait<BSpline<_Scalar, _dim, _degree, _generic> > {
    static_assert(_dim > 0, "Dimension must be positive.");
    static_assert(_degree>=0 || _generic,
            "Invalid degree for non-generic Bezier setting");

    using Scalar = _Scalar;
    using Point = Eigen::Matrix<Scalar, 1, _dim>;
    using ControlPoints = Eigen::Matrix<Scalar, _generic?Eigen::Dynamic:_degree+1, _dim>;
    using KnotVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using Base = SplineBase<_Scalar, _dim>;

    static constexpr int dim = _dim;
    static constexpr int degree = _degree;
    static constexpr bool generic = _generic;
};

template<typename _Scalar, int _dim, int _degree, bool _generic>
struct CurveTrait<NURBS<_Scalar, _dim, _degree, _generic> > {
    static_assert(_dim > 0, "Dimension must be positive.");
    static_assert(_degree>=0 || _generic,
            "Invalid degree for non-generic Bezier setting");

    using Scalar = _Scalar;
    using Point = Eigen::Matrix<Scalar, 1, _dim>;
    using ControlPoints = Eigen::Matrix<Scalar, _generic?Eigen::Dynamic:_degree+1, _dim>;
    using KnotVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using WeightVector = Eigen::Matrix<_Scalar, _generic?Eigen::Dynamic:_degree+1, 1>;
    using Base = SplineBase<_Scalar, _dim>;

    static constexpr int dim = _dim;
    static constexpr int degree = _degree;
    static constexpr bool generic = _generic;
};


}
