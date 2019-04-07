import tensorflow as tf
import numpy as np

#env = env.unwrapped

def create_nn_variables(text_file):
	var_name_temp = []
	var_val_temp = []
	var_val_str_temp = []
	for v in tf.global_variables():
		if not "Actor" in v.name:
			continue
		if "Adam" in v.name:
			continue
		if "dense_3" in v.name:
			continue
		v_ = sess.run(v)
		var_name_temp.append(v.name)

		array_name = v.name.replace('/','').replace(':0','')
		matrix_name = v.name.replace('Agent','').replace('Actor','').replace('/','').replace(':0','')
	
		if len(v_.shape) > 1:

			text_file.write("const float {}[{}][{}] = \n".format(array_name, v_.shape[1], v_.shape[0]))
			temp_str = '{'
			for i in range(v_.shape[1]):
				temp_str += '{'
				for j in range(v_.shape[0]):
					temp_str += str(v_[j,i])
					temp_str += ','
				temp_str = temp_str[:-1]
				temp_str += '},\n'
			temp_str = temp_str[:-2]
			temp_str += '}'
			text_file.write(temp_str + ";\n\n")
			text_file.write("const Matrix<float, {}, {}> matrix_{}({});\n\n\n".format(v_.shape[1], v_.shape[0], array_name, array_name))

		else:

			text_file.write("const float {}[{}] = \n".format(array_name, v_.shape[0]))
			temp_str = '{'
			for i in range(v_.shape[0]):
				temp_str += str(v_[i])
				temp_str += ','
			temp_str = temp_str[:-1]
			temp_str += '}'
			text_file.write(temp_str + ";\n\n")
			text_file.write("const Matrix<float, {}, 1> matrix_{}({});\n\n\n".format(v_.shape[0], array_name, array_name))

def include_header(text_file):
	text_file.write("#include <iostream>\n#include <stdlib.h>\n#include <matrix/matrix/math.hpp>\nusing namespace std;\nusing namespace matrix;\n\n")

def policy_version(text_file, file_name):
	text_file.write("\nchar file_name[64] = \"{}\";\n\n".format(file_name))

def nn_api(text_file):
	text_file.write("Vector<float, 4> nn_controller(Vector<float, 18> &input_states)\n{\n\
	static Vector<float, 32> temp;\n	static Vector<float, 4> output;\n\
	temp = matrix_AgentActordensekernel*input_states + matrix_AgentActordensebias;\n	for (int i = 0; i < 32; ++i){\n		if (temp(i) < 0)\n			temp(i) = 0;\n	}\n\n\
	temp = matrix_AgentActordense_1kernel*temp + matrix_AgentActordense_1bias;\n	for (int i = 0; i < 32; ++i){\n		if (temp(i) < 0)\n			temp(i) = 0;\n	}\n\n 	output = matrix_AgentActordense_2kernel*temp;\n\
	for (int i = 0; i < 4; ++i)\n	{\n		output(i) = sin(output(i));\n	}\n\n	return output;\n}")

if __name__ == "__main__":
	with tf.Session() as sess:
		file_name = "FRPO_quad_323299000"
		saver = tf.train.import_meta_graph("./"+file_name+".meta")
		sess.run(tf.global_variables_initializer())
		saver.restore(sess, tf.train.latest_checkpoint("./"))
		graph = tf.get_default_graph()
		state = graph.get_tensor_by_name("env/obs0:0")
		AGENT = graph.get_tensor_by_name("Agent/Actor/Sin:0")

		with open("libnncontroller.cpp", "w") as text_file:
			include_header(text_file)
			text_file.write("extern \"C\" {\n")
			policy_version(text_file, file_name)
			create_nn_variables(text_file)
			nn_api(text_file)
			text_file.write("\n}")

		
