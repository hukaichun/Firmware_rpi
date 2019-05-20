#include <Eigen/Dense>

#include <array>

extern "C"{

std::array<float,3> neural_network_respond(const std::array<float,18>& state_v) {

	Eigen::RowVectorXf vec(18);
	std::array<float,3> output;

	for(int i=0; i<18; ++i) vec[i] = state_v[i];

	for(int i=0; i<3; ++i)
		output[i] = vec[i]*0;

	return output;
}

char name[] = "High-Level controller";

}