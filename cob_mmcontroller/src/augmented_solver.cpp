

#include "cob_mmcontroller/augmented_solver.h"

#define DEBUG false

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
		base_is_actived_ = true;
    }

	augmented_solver::~augmented_solver()
    {
    }


    int augmented_solver::CartToJnt(const JntArray& q_in, Twist& v_in, JntArray& qdot_out, JntArray& qdot_base_out)
    {
    	double damping_factor = 0.01;

        //Let the ChainJntToJacSolver calculate the jacobian "jac" for
        //the current joint positions "q_in"
        jnt2jac.JntToJac(q_in,jac);

		//v_in.vel.z(0.01);

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
        int num_dof = chain.getNrOfJoints() + jac_base.cols();
		if(DEBUG)
			std::cout << "Combined jacobian:\n " << jac_full << "\n";

        //Weighting Matrices
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_v;
        W_v.resize(num_dof,num_dof);
        W_v.setZero();
        for(int i=0 ; i<num_dof ; i++)
        	W_v(i,i) = damping_factor;


        Eigen::Matrix<double, 6,6> W_e;
        W_e.setZero();
        for(unsigned int i=0 ; i<6 ; i++)
        	W_e(i,i) = 1;

        if(DEBUG)
        	std::cout << "Weight matrix defined\n";
        //W_e.setIdentity(6,6);

        //Definition of additional task for platform


        //Inversion
        // qdot_out = (jac_full^T * W_e * jac_full + jac_augmented^T * W_c * jac_augmented + W_v)^-1(jac_full^T * W_e * v_in + jac_augmented^T * W_c * z_in)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> damped_inversion;
        damped_inversion.resize(num_dof,num_dof);

        damped_inversion = (jac_full.transpose() * W_e * jac_full) + W_v;  /* +  jac_augmented.transpose() * W_c * jac_augmented / + W_v;*/
        if(DEBUG)
        	std::cout << "Inversion done\n";

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> q_dot_conf_control;
        Eigen::Matrix<double, 6, 1> v_in_eigen;
        v_in_eigen.setZero();
        v_in_eigen(0,0) = v_in.vel.x();
        v_in_eigen(1,0) = v_in.vel.y();
        v_in_eigen(2,0) = v_in.vel.z();
        v_in_eigen(3,0) = v_in.rot.x();
        v_in_eigen(4,0) = v_in.rot.y();
        v_in_eigen(5,0) = v_in.rot.z();
        q_dot_conf_control = damped_inversion.inverse() * jac_full.transpose() * W_e * v_in_eigen;

        if(DEBUG)
        	std::cout << "Endergebnis: \n" << q_dot_conf_control << "\n";

        //Do a singular value decomposition of "jac" with maximum
        //iterations "maxiter", put the results in "U", "S" and "V"
        //jac = U*S*Vt
        int ret = svd.calculate(jac,U,S,V,maxiter);

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
            tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
            //tmp(i) = sum*1.0/S(i);
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
        if(DEBUG)
        	std::cout << "Solution SVD: " << qdot_out(0) << " " << qdot_out(1) << " " << qdot_out(2) << " " << qdot_out(3) << " " << qdot_out(4) << " " << qdot_out(5) << " " << qdot_out(6)  << "\n====\n";
        //return the return value of the svd decomposition
        //New calculation
        for(unsigned int i=0;i<7;i++)
        {
        	qdot_out(i)=q_dot_conf_control(i,0);
        }
        if(base_is_actived_)
        {
        	for(unsigned int i = 7; i<7+3; i++)
        		qdot_base_out(i-7) = q_dot_conf_control(i,0);
        }

        if(DEBUG)
        	std::cout << "Solution ConfControl: " << qdot_out(0) << " " << qdot_out(1) << " " << qdot_out(2) << " " << qdot_out(3) << " " << qdot_out(4) << " " << qdot_out(5) << " " << qdot_out(6)  << "\n====\n";
        return ret;
    }

}
