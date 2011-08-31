#ifndef AUGMENTED_SOLVER_HPP
#define AUGMENTED_SOLVER_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/utilities/svd_HH.hpp>
#include <Eigen/LU>
#include <iostream>

namespace KDL
{
    /**
     * Implementation of a inverse velocity kinematics algorithm based
     * on the generalize pseudo inverse to calculate the velocity
     * transformation from Cartesian to joint space of a general
     * KDL::Chain. It uses a svd-calculation based on householders
     * rotations.
     *
     * @ingroup KinematicFamily
     */
    class augmented_solver
    {
    public:
        /**
         * Constructor of the solver
         *
         * @param chain the chain to calculate the inverse velocity
         * kinematics for
         * @param eps if a singular value is below this value, its
         * inverse is set to zero, default: 0.00001
         * @param maxiter maximum iterations for the svd calculation,
         * default: 150
         *
         */
    	augmented_solver(const Chain& chain,double eps=0.00001,int maxiter=150);
        ~augmented_solver();

        virtual int CartToJnt(const JntArray& q_in, Twist& v_in, JntArray& qdot_out, JntArray& qdot_base_out);
        /**
         * not (yet) implemented.
         *
         */
        virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out){return -1;};

        void setAugmentedJacobian(Eigen::Matrix<double,6,Eigen::Dynamic> _jac_augmented);
    private:
        const Chain chain;
        ChainJntToJacSolver jnt2jac;
        Jacobian jac;
        SVD_HH svd;
        std::vector<JntArray> U;
        JntArray S;
        std::vector<JntArray> V;
        JntArray tmp;
        double eps;
        int maxiter;
        bool base_is_actived_;
    };
}
#endif

