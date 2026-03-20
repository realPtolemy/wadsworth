Copyright 2026 Love Mitteregger
#include <mujoco/mujoco.h>

#include <iostream>

	int main() {
	// Load mesh decoder plugins (required for .stl files in MuJoCo 3.x)
	mj_loadAllPluginLibraries(MUJOCO_PLUGIN_DIR, nullptr);

	char error[1000] = "Could not load XML model";

	mjModel* m = mj_loadXML(SCENE_XML_PATH, nullptr, error, 1000);

	if (!m) {
		std::cerr << "Load model error: " << error << std::endl;
		return 1;
	}

	mjData* d = mj_makeData(m);

	std::cout << "Successfully loaded SO-ARM101!" << std::endl;
	std::cout << "Number of joints: " << m->njnt << std::endl;
	std::cout << "Number of bodies: " << m->nbody << std::endl;
	std::cout << "Number of actuators: " << m->nu << std::endl;

	mj_deleteData(d);
	mj_deleteModel(m);

	return 0;
}
