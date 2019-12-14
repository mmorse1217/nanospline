#pragma once

#include <Eigen/Core>

#include <nanospline/CurveTrait.h>
#include <nanospline/SplineBase.h>

namespace nanospline {

template<typename CurveDerived>
class BezierBase : public CurveTrait<CurveDerived>::Base {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Trait = CurveTrait<CurveDerived>;
        using Base = typename Trait::Base;
        using Scalar = typename Trait::Scalar;
        using Point = typename Trait::Point;
        using ControlPoints = typename Trait::ControlPoints;

    public:
        virtual ~BezierBase()=default;
        virtual Point evaluate(Scalar t) const override =0;
        virtual Scalar inverse_evaluate(const Point& p) const override =0;
        virtual Point evaluate_derivative(Scalar t) const override =0;
        virtual Point evaluate_2nd_derivative(Scalar t) const override =0;

        virtual Scalar approximate_inverse_evaluate(const Point& p,
                const Scalar lower=0.0,
                const Scalar upper=1.0,
                const int level=3) const override {
            const int num_samples = 2 * (get_degree()+1);

            return Base::approximate_inverse_evaluate(
                    p, num_samples, lower, upper, level);
        }

        virtual void write(std::ostream &out) const override {
            out << "c:\n" << m_control_points << "\n";
        }

    public:
        CurveDerived& get_derived() {
            return *dynamic_cast<CurveDerived*>(this);
        }

        const CurveDerived& get_derived() const {
            return *dynamic_cast<CurveDerived*>(this);
        }

        const ControlPoints& get_control_points() const {
            return m_control_points;
        }

        template<typename Derived>
        void set_control_points(const Eigen::PlainObjectBase<Derived>& ctrl_pts) {
            m_control_points = ctrl_pts;
        }

        template<typename Derived>
        void set_control_points(Eigen::PlainObjectBase<Derived>&& ctrl_pts) {
            m_control_points.swap(ctrl_pts);
        }

        int get_degree() const {
            return Trait::generic ? static_cast<int>(m_control_points.rows())-1 : Trait::degree;
        }

        Scalar get_domain_lower_bound() const {
            return 0.0;
        }

        Scalar get_domain_upper_bound() const {
            return 1.0;
        }

    protected:
        ControlPoints m_control_points;
};

}
