#include <Eigen/Dense>
#include <array>
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


std::array<float,4> neural_network_respond(const std::array<float, 18>& state_v) {
	Eigen::RowVectorXf vec(18);
	std::array<float,4> output;

	for(int i=0; i<18; ++i) vec[i] = state_v[i];

	vec*=dense_kernel;
	vec+=dense_bias;
	truncate_zero(vec);

	vec*=dense_1_kernel;
	vec+=dense_1_bias;
	truncate_zero(vec);

	vec*=dense_2_kernel;
	vec+=dense_2_bias;

	for(int i=0; i<4; ++i)
		output[i] = sin(vec[i]);

	return output;
}


char name[] = "FRPO_low";

}