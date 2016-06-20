// ImGui - standalone example application for Glfw + OpenGL 2, using fixed pipeline
// If you are new to ImGui, see examples/README.txt and documentation at the top of imgui.cpp.

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include <stdio.h>
#include <GLFW/glfw3.h>
#include <time.h> 
#include <math.h>
#include <vector>

#include <iostream>
#include <cstdlib>

#include "dynamixel.h"

static void error_callback(int error, const char* description){
	fprintf(stderr, "Error %d: %s\n", error, description);
}

#define NB_MOTORS 1

struct Sample{
	Sample(int p_t){
		t = p_t;
		for (int i =0 ; i < NB_MOTORS; i++)value[i] = 512;
	}
	int t;
	int value[NB_MOTORS];
};

struct PeriodicPLfunc{
	int value(int motor_id, int t){
		if (pts.empty()) return 512;
		int i=0;
		pts.push_back(pts[0]);
		pts.back().t += anim_length;
		if (t < pts[0].t) t += anim_length; 
		while (pts[i+1].t < t) i++;
		assert(i+1<pts.size());
		float c = float(t - pts[i].t) / float(pts[i + 1].t - pts[i].t);
		pts.pop_back();
		//std::cerr << pts[i].value[motor_id] << " ";
		return (1. - c)*float(pts[i].value[motor_id]) + c*float(pts[(i + 1) % pts.size()].value[motor_id]);
		
		return 512;
	}

	void next_pt(float t, int playspeed, int *goal, float& dt){
		if (pts.empty()) { for (int i = 0; i < NB_MOTORS; i++)goal[i] = 512; dt = 1; return; }
		int i = 0;
		pts.push_back(pts[0]);
		pts.back().t += anim_length;
		if (t < pts[0].t) t += anim_length;
		while (pts[i + 1].t < t) i++;
		assert(i + 1<pts.size());
		if (playspeed > 0) {
			dt = (pts[i + 1].t - t) / double(playspeed);
			for (int i = 0; i < NB_MOTORS; i++) goal[i] = pts[i + 1].value[i];
		}
		else {
			dt = (t - pts[i].t) / double(-playspeed);
			for (int i = 0; i < NB_MOTORS; i++) goal[i] = pts[i].value[i];
		}
		pts.pop_back();
	}
	int cur_pt_i(float t){
		for (int i=	0;i<pts.size();i++)	if (t == pts[i].t) return i;
		return -1;
	}

	int prev_pt_i(float t){
		if (pts.empty()) { return -1; }
		int i = 0;
		pts.push_back(pts[0]);
		pts.back().t += anim_length;
		if (t < pts[0].t) t += anim_length;
		while (pts[i + 1].t < t) i++;
		pts.pop_back();
		return i;
	}
	int next_pt_i(float t){
		int prev = prev_pt_i(t);
		if (pts[(prev + 1) % pts.size()].t == t) 		return (prev + 2) % pts.size();

		return ( prev+ 1) % pts.size();
	}
	void remove_sample(int id){
		for (int i = id; i < pts.size() - 1; i++)
			pts[i] = pts[i + 1];
		pts.pop_back();
	}
	int add_sample(Sample s){
		if (cur_pt_i(s.t) != -1) return cur_pt_i(s.t);
		int prev = prev_pt_i(s.t);
		pts.push_back(s);
		for (int i = pts.size() - 2; i > prev; i--)
			std::swap(pts[i],pts[i + 1]);
	}

	std::vector<Sample> pts;
	int anim_length;
};

enum EditState{
	GOTO_POS,
	WAIT,
	EDITING
} currentstate;

int main(int, char**){

	static int anim_length = 10000;


	Dynamixel dxl(1);
	if (!dxl.open_serial("\\\\.\\COM3")) {
		std::cerr << "Can not open serial device" << std::endl;
		return -1;
	}

	std::cerr << dxl.set_moving_speed(1, 0) << std::endl;
	std::cerr << dxl.set_max_torque(1, 256) << std::endl;
	
	// Setup window
	glfwSetErrorCallback(error_callback);
	if (!glfwInit()) return 1;
	GLFWwindow* window = glfwCreateWindow(1280, 720, "SpiderGigot", NULL, NULL);
	glfwMakeContextCurrent(window);

	// Setup ImGui binding
	ImGui_ImplGlfw_Init(window, true);


	// Main loop
	clock_t last_t = clock();
	PeriodicPLfunc func;
	func.anim_length = 10000;
	
	Sample samples[4] = { Sample(1000), Sample(2000), Sample(3000), Sample(4000) };
	int samples_val[4] = { 250,750,320,750};

	for (int s = 0; s < 4; s++) for (int i = 0; i < NB_MOTORS; i++) samples[s].value[i] = samples_val[s];

	for (int s = 0; s < 4; s++) func.add_sample(samples[s]);

	int motormap[NB_MOTORS];
	for (int i = 0; i < NB_MOTORS; i++) motormap[i] = i+1;

	
	std::vector<float> keypoints(func.anim_length / 10, 0);
	std::vector<float> plot(func.anim_length / 10, 0);
	int time = 0.0f;
	bool ok;


	while (!glfwWindowShouldClose(window))
	{
		
		glfwPollEvents();
		ImGui_ImplGlfw_NewFrame();
		ImGui::SetNextWindowPos(ImVec2(100, 10));
		ImGui::SetNextWindowSize(ImVec2(1000, 500));
		if (!ImGui::Begin("Example: Fixed Overlay", &ok, ImVec2(0, 0), 0.3f, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings)){
			ImGui::End();
			return -1;
		}

		static int animate_speed = 0;
		ImGui::SliderInt("animate_speed", &animate_speed, -2, 2);
		if (animate_speed != 0) currentstate = GOTO_POS;

		
		clock_t t = clock();
		int dt = 1000.*float(t-last_t) / float(CLOCKS_PER_SEC);
		last_t = t;
		

		time += double(animate_speed)*dt;
		while (time >= func.anim_length) time -= func.anim_length;
		while (time <0) time += func.anim_length;


		
		if (ImGui::SmallButton("Prev")) { animate_speed = 0; time = func.pts[func.prev_pt_i(time)].t; currentstate = GOTO_POS; }

		bool editing = (animate_speed==0) ;


		int pos[NB_MOTORS];
		for (int i = 0; i < NB_MOTORS; i++) dxl.get_present_position(motormap[i], pos[i]);

		int wished_pos[NB_MOTORS];
		for (int i = 0; i < NB_MOTORS; i++)wished_pos [i] = func.value(i, time);

		if (editing){
			if (currentstate == GOTO_POS){
				currentstate = WAIT;
				for (int i = 0; i < NB_MOTORS; i++)
				if (std::abs(pos[i] - wished_pos[i]) > 5)
					currentstate = GOTO_POS;
			}
			if (currentstate == WAIT){
				for (int i = 0; i < NB_MOTORS; i++)if (std::abs(pos[i] - wished_pos[i]) > 100)currentstate = EDITING;
			}
			if (currentstate == EDITING) {
				if (func.cur_pt_i(time) == -1)
					func.add_sample(Sample(time));
				for (int i = 0; i < NB_MOTORS; i++)
					func.pts[func.cur_pt_i(time)].value[i] = pos[i];
			}
		}
		 editing = (animate_speed == 0) && (func.cur_pt_i(time) != -1);
		 std::cerr << " currentstate " << currentstate << "  " << pos[0] << "  " << wished_pos[0] << std::endl;
		if (editing ){

			ImGui::SameLine(100);
			if (ImGui::SmallButton("<==="))  if (func.cur_pt_i(time) != -1) {
				func.pts[func.cur_pt_i(time)].t -= 100;
				time -= 100;
			}
			ImGui::SameLine(140);
			if (ImGui::SmallButton("<--"))  if (func.cur_pt_i(time) != -1) {
				func.pts[func.cur_pt_i(time)].t -= 10;
				time -= 10;
			}
			ImGui::SameLine(180);
			if (ImGui::SmallButton("Delete")) {
				if (func.cur_pt_i(time) != -1){
					func.remove_sample(func.cur_pt_i(time));
					currentstate = GOTO_POS;
				}
		}
			ImGui::SameLine(250);
			if (ImGui::SmallButton("-->"))  if (func.cur_pt_i(time) != -1) {
				func.pts[func.cur_pt_i(time)].t += 10;
				time += 10;
			}
			ImGui::SameLine(290);
			if (ImGui::SmallButton("===>"))  if (func.cur_pt_i(time) != -1) {
				func.pts[func.cur_pt_i(time)].t += 100;
				time += 100;
			}
		}
		ImGui::SameLine(350);
		if (ImGui::SmallButton("Next")) { animate_speed = 0; time = func.pts[func.next_pt_i(time)].t; currentstate = GOTO_POS; }


		ImGui::PushItemWidth(-1);

		int gettime = time;
		ImGui::SliderInt("Time", &gettime, 0.0f, func.anim_length);
		if (time != gettime) currentstate = GOTO_POS;
		time = gettime;

		// control goal position
		for (int i = 0; i < NB_MOTORS; i++)
			dxl.set_goal_position(motormap[i], func.value(i,time)) ;


		// control goal + speed
		/*if (iter % 1 == 0){
			float dt2goal;
			int goal;
			func.next_pt(time, animate_speed, goal, dt2goal);
			if (animate_speed == 0)
				dxl.set_goal_position(1, func.value(time));
			else {
				std::cerr << std::abs((float(goal - pos)) / 1024.) / dt2goal;
				std::cerr << dxl.set_moving_speed(1, std::abs((float(goal - pos)) ) / dt2goal) << std::endl;
				std::cerr << dxl.set_goal_position(1, goal) << std::endl;
			}
		}*/



		for (int i = 0; i < keypoints.size(); i++)	keypoints[i] = .3;
		for (int i = 0; i < func.pts.size(); i++)	for (int di = -1; di < 2; di++)	{
			int ii = func.pts[i].t/10 + di; 
			if (ii >= keypoints.size()) ii = keypoints.size() - 1;
			if (ii < 0) ii = 0;
			keypoints[ii] = 0;
		}

		for (int i = 0; i < plot.size(); i++)	plot[i] = func.value(0,10 * i);


		int curtime_pt = time / 10;
		if (curtime_pt < 0)curtime_pt = 0;
		if (curtime_pt >= keypoints.size())curtime_pt = keypoints.size()-1;
		keypoints[curtime_pt] += .7;
		plot[curtime_pt] += 50;
		ImGui::PlotLines("keypoints", keypoints.data(), keypoints.size(), 0, "gna", 0, 1, ImVec2(1000, 100));
		ImGui::PlotLines("plot", plot.data(), plot.size(), 0, "objective", 0, 1024, ImVec2(1000, 300));

		plot[curtime_pt] -= 50;
		keypoints[curtime_pt] -= .7;




		ImGui::End();
		
		// Rendering
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0,0,0,0);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui::Render();
		glfwSwapBuffers(window);
		
	}

	// Cleanup
	ImGui_ImplGlfw_Shutdown();
	glfwTerminate();

	return 0;
}
