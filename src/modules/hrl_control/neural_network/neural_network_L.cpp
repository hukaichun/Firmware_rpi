#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "parameter.hpp"


/**
 *     ReLU activating function
 */

inline void truncate_zero(Eigen::RowVectorXf& vec) 
{
	for(int i=0; i<vec.size(); ++i)
		vec[i] = vec[i]>0? vec[i]:0;
}



extern "C"{




void neural_network_respond(std::vector<float>* state_v, std::vector<float>* output)
{
	Eigen::RowVectorXf vec(18);

	for(int i=0; i<18; ++i) vec[i] = (*state_v)[i];


	vec*=dense_kernel;
	vec+=dense_bias;
	truncate_zero(vec);

	vec*=dense_1_kernel;
	vec+=dense_1_bias;
	truncate_zero(vec);

	vec*=dense_2_kernel;
	vec+=dense_2_bias;

	output->resize(4,0);
	for(int i=0; i<4; ++i)
		(*output)[i] = sin(vec[i]);

	return;
}


char name[] = "FRPO_low";

}