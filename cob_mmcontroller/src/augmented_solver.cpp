// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "augmented_solver.h"

namespace KDL
{
	augmented_solver::augmented_solver(const Chain& _chain,double _eps,int _maxiter):
        chain(_chain),
        jnt2jac(chain),
        jac(chain.getNrOfJoints()),
        svd(jac),
        U(6,JntArray(chain.getNrOfJoints())),
        S(chain.getNrOfJoints()),
        V(chain.getNrOfJoints(),JntArray(chain.getNrOfJoints())),
        tmp(chain.getNrOfJoints()),
        eps(_eps),
        maxiter(_maxiter)
    {
		base_is_actived_ = false;
    }

	augmented_solver::~augmented_solver()
    {
    }


    int augmented_solver::CartToJnt(const JntArray& q_in, const JntArray& q_in_base, Twist& v_in, JntArray& qdot_out)
    {
    	//testcase:
    	double velx = v_in.vel.x() + 0.1;
    	v_in.vel.x(velx);

        //Let the ChainJntToJacSolver calculate the jacobian "jac" for
        //the current joint positions "q_in"
        jnt2jac.JntToJac(q_in,jac);

        //Create standard platform jacobian
        Eigen::Matrix<double,6,3> jac_base;
        jac_base.setZero();
        if(base_is_actived_)
        {
        	jac_base(0,0) = 1.0;
        	jac_base(1,1) = 1.0;
        	jac_base(5,2) = 1.0;
        }

        //Put full jacobian matrix together
        Eigen::Matrix<double, 6, Eigen::Dynamic> jac_full;
        jac_full.resize(6,chain.getNrOfJoints() + jac_base.cols());
        jac_full << jac.data, jac_base;

        std::cout << "Combined jacobian:\n " << jac_full << "\n";

        //Weighting Matrices
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_v;
        W_v.resize(chain.getNrOfJoints() + jac_base.cols(),chain.getNrOfJoints() + jac_base.cols());
        W_v.setIdentity(chain.getNrOfJoints() + jac_base.cols(),chain.getNrOfJoints() + jac_base.cols());

        Eigen::Matrix<double, 6,6> W_e;
        W_e.setIdentity(6,6);

        //Inversion TODO: noch ohne augmented tasks just the infrastructure
        // qdot_out = (jac_full^T * W_e * jac_full + jac_augmented^T * W_c * jac_augmented + W_v)^-1(jac_full^T * W_e * v_in + jac_augmented^T * W_c * z_in)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> testmatrix;
        testmatrix.resize(chain.getNrOfJoints() + jac_base.cols(),chain.getNrOfJoints() + jac_base.cols());
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> testmatrix_inv;
        testmatrix_inv.resize(chain.getNrOfJoints() + jac_base.cols(),chain.getNrOfJoints() + jac_base.cols());
        testmatrix = (jac_full.transpose() * W_e * jac_full) + W_v;
        testmatrix_inv = testmatrix.inverse() ;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> testmatrix2, test2, test3;
        Eigen::Matrix<double, 6, 1> v_in_eigen;
        v_in_eigen.setZero();
        v_in_eigen(0,0) = v_in.vel.x();
        v_in_eigen(1,0) = v_in.vel.y();
        v_in_eigen(2,0) = v_in.vel.z();
        v_in_eigen(3,0) = v_in.rot.x();
        v_in_eigen(4,0) = v_in.rot.y();
        v_in_eigen(5,0) = v_in.rot.z();
        testmatrix2 = testmatrix_inv *  jac_full.transpose() * W_e ;
        test2 = testmatrix2 * v_in_eigen;

        std::cout << "TestMatrix:\n " << testmatrix2 << "\n ================\n";
        std::cout << "Endergebnis: \n" << test2 << "\n ====\n";

        //Do a singular value decomposition of "jac" with maximum
        //iterations "maxiter", put the results in "U", "S" and "V"
        //jac = U*S*Vt
        int ret = svd.calculate(jac,U,S,V,maxiter);
        /*test3 = V*S*U; // TODO: convert to eigen data types for comparison
        std::cout << "Out of SVD: \n" << test3 << "\n";*/
        double sum;
        unsigned int i,j;

        // We have to calculate qdot_out = jac_pinv*v_in
        // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
        // qdot_out = V*S_pinv*Ut*v_in

        //first we calculate Ut*v_in
        for (i=0;i<jac.columns();i++) {
            sum = 0.0;
            for (j=0;j<jac.rows();j++) {
                sum+= U[j](i)*v_in(j);
            }
            //If the singular value is too small (<eps), don't invert it but
            //set the inverted singular value to zero (truncated svd)
            //tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
            tmp(i) = sum*1.0/S(i);
        }
        //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
        //it with V to get qdot_out
        for (i=0;i<jac.columns();i++) {
            sum = 0.0;
            for (j=0;j<jac.columns();j++) {
                sum+=V[i](j)*tmp(j);
            }
            //Put the result in qdot_out
            qdot_out(i)=sum;
        }
        //return the return value of the svd decomposition
        return ret;
    }

}
