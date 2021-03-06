/**
 * This code is automatically generated by scripts/optimal_turning_angle.py
 */

#pragma once
#include <cassert>
#include <vector>

#include <nanospline/internal/auto_match_tangent_Bezier.h>
#include <nanospline/internal/auto_match_tangent_RationalBezier.h>

namespace nanospline {
namespace internal {


#if defined(HIGH_DEGREE_SUPPORT) || 2 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_RationalBezier_degree_2(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2,
        Scalar w0, Scalar w1, Scalar w2,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_RationalBezier_degree_2(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2,
        Scalar w0, Scalar w1, Scalar w2,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 3 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_RationalBezier_degree_3(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3,
        Scalar w0, Scalar w1, Scalar w2, Scalar w3,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_RationalBezier_degree_3(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3,
        Scalar w0, Scalar w1, Scalar w2, Scalar w3,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 4 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_RationalBezier_degree_4(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4,
        Scalar w0, Scalar w1, Scalar w2, Scalar w3, Scalar w4,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_RationalBezier_degree_4(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4,
        Scalar w0, Scalar w1, Scalar w2, Scalar w3, Scalar w4,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 2 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_2(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_2(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 3 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_3(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_3(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 4 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_4(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_4(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 5 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_5(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_5(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 6 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_6(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_6(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 7 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_7(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_7(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 8 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_8(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7, Scalar cx8, Scalar cy8,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_8(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7, Scalar cx8, Scalar cy8,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 9 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_9(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7, Scalar cx8, Scalar cy8, Scalar cx9, Scalar cy9,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_9(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7, Scalar cx8, Scalar cy8, Scalar cx9, Scalar cy9,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


#if defined(HIGH_DEGREE_SUPPORT) || 10 < 5
#define Scalar double
extern template
std::vector<Scalar> match_tangent_Bezier_degree_10(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7, Scalar cx8, Scalar cy8, Scalar cx9, Scalar cy9, Scalar cx10, Scalar cy10,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar

#define Scalar float
extern template
std::vector<Scalar> match_tangent_Bezier_degree_10(
        Scalar cx0, Scalar cy0, Scalar cx1, Scalar cy1, Scalar cx2, Scalar cy2, Scalar cx3, Scalar cy3, Scalar cx4, Scalar cy4, Scalar cx5, Scalar cy5, Scalar cx6, Scalar cy6, Scalar cx7, Scalar cy7, Scalar cx8, Scalar cy8, Scalar cx9, Scalar cy9, Scalar cx10, Scalar cy10,
        const Eigen::Matrix<Scalar, 2, 1>& tangent,
        Scalar t0,
        Scalar t1);
#undef Scalar
#endif


} // End internal namespace
} // End nanospline namespace

