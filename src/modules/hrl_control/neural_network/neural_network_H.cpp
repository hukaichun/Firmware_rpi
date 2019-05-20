#include <Eigen/Dense>

#include <vector>



extern "C"{

void neural_network_respond(std::vector<float>* state_v, std::vector<float>* output)
{
	Eigen::Map<Eigen::RowVectorXf> state(state_v->data(), 18);

	output->resize(3);
	for(auto& v: *output) v=0;

	return;
}

char name[] = "High-Level controller";

}