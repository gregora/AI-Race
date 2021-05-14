#include "wrapper.h"
#include "network.h"
#include <random>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <thread>
#include <time.h>
#include <limits>

using namespace std;

const float RAY_LENGTH = 20;
const int NUMBER_OF_RAYS = 9;
bool FIRST_CAR_HUMAN_CONTROLLED = false;
bool DISPLAY = false;
bool RECORD = false;

const bool COLLISIONS = false;

const float FLOAT_MIN = - numeric_limits<float>::max();

double randDouble() {

	std::random_device rd{};
	std::mt19937 engine{rd()};
	std::uniform_real_distribution<double> dist{0.0, 1.0};
	double ret = dist(engine);
	return ret;

}

void CycleArray(float * array, Network ** networks, int length) {

	float tmp_array[length];
	Network* tmp_networks[length];

	tmp_array[length - 1] = array[0];
	tmp_networks[length - 1] = networks[0];


	for (int i = 0; i < length - 1; i++) {
			tmp_array[i] = array[i + 1];
			tmp_networks[i] = networks[i + 1];
	}

	for (int i = 0; i < length; i++) {
		array[i] = tmp_array[i];
		networks[i] = tmp_networks[i];
	}

}

void ShuffleArray(float * array, Network ** networks, int length) {

		float tmp_array[length];
		Network* tmp_networks[length];

		for (int i = 0; i < length; i++) {
			tmp_array[i] = array[i];
			tmp_networks[i] = networks[i];
		}

		for (int i = 0; i < length / 2; i++) {
			int rand1 = (int) (randDouble() * length);
			int rand2 = (int) (randDouble() * length);

			Network* tmp_network = tmp_networks[rand1];
			tmp_networks[rand1] = tmp_networks[rand2];
			tmp_networks[rand2] = tmp_network;


			float tmp_score = tmp_array[rand1];
			tmp_array[rand1] = tmp_array[rand2];
			tmp_array[rand2] = tmp_score;

		}

		for (int i = 0; i < length; i++) {
			array[i] = tmp_array[i];
			networks[i] = tmp_networks[i];
		}

}

std::vector<std::string> SplitString(std::string string, std::string split_by) {

	std::vector<std::string> ret;

	while (string.find(split_by) != std::string::npos) {
		int spacepos = string.find(split_by);
		ret.push_back(string.substr(0, spacepos));
		string = string.substr(spacepos + split_by.size(), string.size());
	}

	ret.push_back(string.substr(0, string.find(split_by)));

	return ret;

}


b2Vec2 * fileToChain(std::string path, int * number_of_edges = nullptr) {

	int edges = 0;

	std::ifstream inFile;
	inFile.open(path); // open the input file

	std::stringstream strStream;
	strStream << inFile.rdbuf(); // read the file
	std::string str = strStream.str(); // str holds the content of the file

	std::vector<std::string> file_split = SplitString(str, "\n");

	b2Vec2 * box = new b2Vec2[file_split.size()];

	int i = 0;
	for (std::string line : file_split) {
		if (line != "") {
			std::vector<std::string> coords = SplitString(line, ",");
			box[i].Set(std::stof(coords[0]), std::stof(coords[1]));
			i++;
			edges += 1;
		}
	}

	if (number_of_edges != nullptr)
		(*number_of_edges) = edges;

	return box;

}









class Car {

	public:
		float rays[2*NUMBER_OF_RAYS];
		PhysicsObject * car;
		b2World* world = nullptr;
		bool human_controlled = false;

		float score = 0;
		bool crashed = false;
		Network* network = nullptr;

		Car(b2World* world, Display * display, float x = 0, float y = 0) {

			this -> world = world;

			car = new PhysicsObject(world, x, y, 2, 4.3);
			car -> body -> SetLinearDamping(0.5f);
			car -> body -> SetAngularDamping(20);
			car -> createTexture("img/car.png", display -> renderer, 0.045, 0.045);

		}

		void move() {

			const Uint8 *state = SDL_GetKeyboardState(NULL);

			float *pptr = car -> getPosition();
			float ppos[] = {*pptr, *(pptr + 1), *(pptr + 2)};
			float rot = ppos[2]; // player rotation

			float engine_power = 400;
			float turning_coef = 1000;

			b2Vec2 f(engine_power * sin(-rot), engine_power * cos(-rot));

			b2Vec2 v = car -> body -> GetLinearVelocity();
			float velocity = v.Length();


			if (human_controlled) {
				if (state[SDL_SCANCODE_W]) {
						car -> body -> ApplyForceToCenter(f, true);
				}
				if (state[SDL_SCANCODE_S]) {
					car -> body -> ApplyForceToCenter(-0.5*f, true);
				}

				if (velocity > 1) {
					if (state[SDL_SCANCODE_A]) {
						car -> body -> ApplyTorque(turning_coef, true);
					}
					if (state[SDL_SCANCODE_D]) {
						car -> body -> ApplyTorque(-turning_coef, true);
					}
				}
			} else if (network != nullptr) {

				float angle = rot;
				float x = ppos[0];
				float y = ppos[1];

				double input[2*NUMBER_OF_RAYS + 1];

				for (int i = 0; i < 2*NUMBER_OF_RAYS; i++) {
					input[i] = rays[i];
				}


				input[2*NUMBER_OF_RAYS] = angle;

				network -> set_input_nodes(input);
				network -> forward_propagate();

				if (network -> output_nodes[0] > 0.5) {
					car -> body -> ApplyForceToCenter(f, true);
					if (!crashed)
						score += 1.0 / 60.0;

				}

				if (network -> output_nodes[1] > 0.5) {
					car -> body -> ApplyForceToCenter(-0.5*f, true);
					if (!crashed)
						score -= 1.0 / 60.0;
				}

				if (velocity > 1) {
					if (network -> output_nodes[2] > 0) {
						car -> body -> ApplyTorque(turning_coef, true);
					}

					if (network -> output_nodes[3] > 0) {
						car -> body -> ApplyTorque(-turning_coef, true);
					}
				}

			}

			// turn
			b2Vec2 currentRightNormal = car -> body -> GetWorldVector(b2Vec2(1,0));
			b2Vec2 lateralVel = b2Dot(currentRightNormal, car -> body->GetLinearVelocity()) * currentRightNormal;

			b2Vec2 impulse = car->body->GetMass() * -lateralVel;
			float maxLateralImpulse = 15;

			if (impulse.Length() > maxLateralImpulse) {
				impulse *= maxLateralImpulse / impulse.Length(); // drift
			}

			car->body->ApplyLinearImpulse(impulse, car->body->GetWorldCenter(), true);

		}

		void updateRays() {
			b2RayCastInput input;
			input.p1 = car -> body -> GetPosition();
			input.maxFraction = 1;

			float angle_fraction = 3.14 / (NUMBER_OF_RAYS - 1);

			for (int i = 0; i < NUMBER_OF_RAYS; i++) {
				float car_angle = car -> body -> GetAngle();

				b2Vec2 p2 = b2Vec2(cos(car_angle + angle_fraction*i) * RAY_LENGTH, sin(car_angle + angle_fraction*i) * RAY_LENGTH);

				input.p2 = input.p1 + p2;
				float max_fraction_1 = 1;
				float max_fraction_2 = 1;

				for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
					for (b2Fixture* f = b->GetFixtureList(); f; f = f->GetNext()) {

							for (int j = 0; j < f -> GetShape() -> GetChildCount(); j++) {
								b2RayCastOutput output;
								output.fraction = 10;
								f -> RayCast(&output, input, j);

								if (f -> GetFilterData().categoryBits == (uint16) 1){
									if (max_fraction_1 > output.fraction && output.fraction > 0) {
										max_fraction_1 = output.fraction;
									}
								}else{
									if (max_fraction_2 > output.fraction && output.fraction > 0) {
										max_fraction_2 = output.fraction;
									}
								}


						}
					}
				}

				rays[i] = max_fraction_1;
				rays[i + NUMBER_OF_RAYS] = max_fraction_2;
			}

		}

		void addScore() {
			float addition = 0;

			b2Vec2 v = car -> body -> GetLinearVelocity();
			float velocity = v.Length();

			b2ContactEdge* ce = car -> body -> GetContactList();

			while (ce != NULL) {
					if (ce -> contact -> IsTouching()) {
						crashed = true;
						// score = -1000;
						break;
					}
					ce = ce -> next;
			}



		}

};

void CallUpdate(Car * car) {
	car -> updateRays();
	car -> move();
	car -> addScore();
}









void race(int number_of_cars, Network* networks[], float * scores, int race_length, bool record = false) {

	b2Vec2 gravity(0.0f, 0.0f);
	b2World world(gravity);

	float starting_positions[number_of_cars][2];

	for (int i = 0; i < number_of_cars; i++) {
		float x, y;
		if (i % 2 == 0) {
			x = 232;
		} else {
			x = 244;
		}

		y = 15*i - 150;


		/*if (!COLLISIONS) {
			x = 238;
			y = -150;
		}*/

		starting_positions[i][0] = x;
		starting_positions[i][1] = y;

	}

	int display_height, display_width;

	if (record) {
		display_width = 1920;
		display_height = 1080;
	} else {
		display_width = 1000;
		display_height = 600;
	}

	Display* display = new Display(display_width, display_height, "A.I. Race", 10, DISPLAY);
	// display -> setFullscreen(true);
	SDL_SetRenderDrawBlendMode(display -> renderer, SDL_BLENDMODE_BLEND);


	// add field
	PhysicsObject groundBody(&world, 0, 0);
	groundBody.setType(b2_staticBody);
	b2Fixture * fix = groundBody.body -> GetFixtureList();
	groundBody.body -> DestroyFixture(fix);

	b2Filter filt;

	filt.categoryBits = (uint16) 1;
	filt.maskBits = (uint16) -1;



	int edges;
	b2ChainShape chain;
	b2Vec2 * box = fileToChain("track/track-outer.ch", &edges);
	chain.CreateLoop(box, edges);
	b2Fixture * f1 = groundBody.body -> CreateFixture(&chain, 1);

	int edges2;
	b2ChainShape chain2;
	b2Vec2 * box2 = fileToChain("track/track-inner.ch", &edges2);
	chain2.CreateLoop(box2, edges2);
	b2Fixture * f2 = groundBody.body -> CreateFixture(&chain2, 1);

	if (!COLLISIONS) {
		f1 -> SetFilterData(filt);
		f2 -> SetFilterData(filt);
	}


	if (DISPLAY) {
		// render ring
		int tex_w = 15000;
		groundBody.texture = SDL_CreateTexture(display -> renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, tex_w, tex_w);

		SDL_SetRenderTarget(display -> renderer, groundBody.texture);
		SDL_SetTextureBlendMode(groundBody.texture, SDL_BLENDMODE_BLEND);
		SDL_SetRenderDrawColor(display -> renderer, 0, 0, 0, 0);
		SDL_RenderClear(display -> renderer);
		SDL_SetRenderDrawColor(display -> renderer, 255, 255, 255, 255);

		for (int i = 0; i < edges; i++) {
			if (i < edges - 1) {
				SDL_RenderDrawLine((*display).renderer, tex_w/2 + box[i].x*10, tex_w/2 - box[i].y*10, tex_w/2 + box[i + 1].x*10, tex_w/2 - box[i + 1].y*10);
			} else {
				SDL_RenderDrawLine((*display).renderer, tex_w/2 + box[i].x*10, tex_w/2 - box[i].y*10, tex_w/2 + box[0].x*10, tex_w/2 - box[0].y*10);
			}
		}

		for (int i = 0; i < edges2; i++) {
			if (i < edges2 - 1) {
				SDL_RenderDrawLine((*display).renderer, tex_w/2 + box2[i].x*10, tex_w/2 - box2[i].y*10, tex_w/2 + box2[i + 1].x*10, tex_w/2 - box2[i + 1].y*10);
			} else {
				SDL_RenderDrawLine((*display).renderer, tex_w/2 + box2[i].x*10, tex_w/2 - box2[i].y*10, tex_w/2 + box2[0].x*10, tex_w/2 - box2[0].y*10);
			}
		}


		SDL_SetRenderTarget(display -> renderer, NULL);
	}


		DisplayTexture * rays_texture = new DisplayTexture("", display -> renderer);
		delete rays_texture -> texture;
		rays_texture -> texture = SDL_CreateTexture(display -> renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 1000, 1000);


		// add objects to display
		display -> addObject(&groundBody);
		display -> addTexture(rays_texture, true);


	Car * cars[number_of_cars];

	for (int i = 0; i < number_of_cars; i++) {
		cars[i] = new Car(&world, display, starting_positions[i][0], starting_positions[i][1]);
		cars[i] -> network = networks[i];


		if (!COLLISIONS) {
			b2Filter * car_filt = new b2Filter();
			car_filt -> maskBits = (uint16) 1;
			car_filt -> categoryBits = (uint16) pow(2, i+2);
			cars[i] -> car -> body -> GetFixtureList() -> SetFilterData(*car_filt);
		}


		display -> addObject(cars[i] -> car);
	}

	cars[0] -> human_controlled = FIRST_CAR_HUMAN_CONTROLLED;

	int frame = 0;

	while (true) {
		frame++;

		thread threads[number_of_cars];

		for (int i = 0; i < number_of_cars; i++) {
			// CallUpdate(cars[i]);
			threads[i] = thread(CallUpdate, cars[i]);
		}

		bool allcrashed = true;

		for (int i = 0; i < number_of_cars; i++) {
			threads[i].join();
			if (!cars[i] -> crashed)
				allcrashed = false;
		}

		if (allcrashed)
			break;



		if (DISPLAY) {

			SDL_SetRenderTarget((*display).renderer, rays_texture -> texture);
			SDL_SetTextureBlendMode(rays_texture -> texture, SDL_BLENDMODE_BLEND);
			SDL_SetRenderDrawColor((*display).renderer, 0, 0, 0, 0);
			SDL_RenderClear(display -> renderer);
			SDL_SetRenderDrawColor((*display).renderer, 255, 255, 255, 255);

			float angle_fraction = 3.14 / (NUMBER_OF_RAYS - 1);

			for (int i = 0; i < NUMBER_OF_RAYS; i++) {
				float car_angle = cars[0] -> car -> body -> GetAngle();
				b2Vec2 p2 = b2Vec2(cos(car_angle + angle_fraction*i) * RAY_LENGTH * 10, sin(car_angle + angle_fraction*i) * RAY_LENGTH * 10);

				SDL_RenderDrawLine((*display).renderer, 500, 500, 500 + p2.x*(cars[0] -> rays[i]), 500 - p2.y*cars[0] -> rays[i]);

			}

			SDL_SetRenderTarget((*display).renderer, NULL);

		}

		b2Vec2 position = cars[0] -> car -> body -> GetPosition();

		/*if (DISPLAY) {
			cout << cars[0] -> score << endl;
		}*/

		display -> camerax = position.x;
		display -> cameray = position.y;

		world.Step(1.0f/60.0f, 6, 2);
		display -> render(DISPLAY);

		if (record)
			display -> saveFrame("recording/" + to_string(frame) + ".png");

		if (frame >= race_length*60) {
			break;
		}

	}

	SDL_DestroyRenderer(display -> renderer);

	for (int i = 0; i < number_of_cars; i++) {
		scores[i] += cars[i] -> score;
		SDL_DestroyTexture(cars[i] -> car -> texture);
		delete cars[i];
	}

	SDL_DestroyTexture(groundBody.texture);
	SDL_DestroyTexture(rays_texture -> texture);
	SDL_DestroyWindow(display -> window);
	delete display;

}







int main() {

	const int number_of_cars = 10; // has to be even

	cout << "Command (learn || #) >";

	string command;

	cin >> command;

	Network* networks[number_of_cars];
	float * scores = new float[number_of_cars]();

	int race_index = 0;

	if (command == "learn") {

		// initial mutation
		for (int i = 0; i < number_of_cars; i++) {
			networks[i] = new Network(2*NUMBER_OF_RAYS + 1, 2, 5, 4);
			networks[i] -> randomize_edges(0.6, 0.7, 0.8);

		}
	} else {


		for (int i = 0; i < number_of_cars; i++) {

			networks[i] = new Network(2*NUMBER_OF_RAYS + 1, 2, 5, 4);
			networks[i] -> load("saves/race" + to_string(stoi(command)) + "/"+to_string(i + 1) + ".AI");
		}

		race_index = stoi(command);
	}

	string disp_str;
	cout << "Display? (yes || no) >";
	cin >> disp_str;

	int number_of_races = 1000;

	if (disp_str == "yes" || disp_str == "y" || disp_str == "record") {
		DISPLAY = true;
		number_of_races = race_index + 1;
	}

	if (disp_str == "record")
		RECORD = true;


	for (race_index; race_index < number_of_races; race_index++) {


		// score initialization
		for (int i = 0; i < number_of_cars; i++)
			scores[i] = 0;

		time_t start = time(0);


		// evaluation
		int evaluation_steps = number_of_cars;

		/*if (!COLLISIONS) {
			evaluation_steps = 1;
		}*/

		for (int r = 0; r < evaluation_steps; r++) {


			// evaluation is slow and ineffective
			race(number_of_cars, networks, scores, 40, RECORD);
			CycleArray(scores, networks, number_of_cars);
			// ShuffleArray(scores, networks, number_of_cars);

		}



		// sorting
		for (int i = 0; i < number_of_cars; i++) {
			float max_score = FLOAT_MIN;
			int max_index = -1;
			for (int j = number_of_cars - 1; j >= 0; j--) {
				if (scores[j] > max_score) {
					max_score = scores[j];
					max_index = j;
				}
			}

			cout << i + 1 << ": " << scores[max_index] / evaluation_steps << "(" << max_index + 1 << ")" << endl;

			networks[max_index] -> save("saves/race" + to_string(race_index) + "/"+to_string(i + 1) + ".AI");
			scores[max_index] = FLOAT_MIN;
			delete networks[max_index];

		}




		// selection
		for (int i = 0; i < number_of_cars; i++) {

			if (i < number_of_cars / 2) {
				// keep the old ones
				networks[i] = new Network(2*NUMBER_OF_RAYS + 1, 2, 5, 4);
				networks[i] -> load("saves/race" + to_string(race_index) + "/"+to_string(i + 1) + ".AI");


			} else {
				// mutate
				networks[i] = new Network(2*NUMBER_OF_RAYS + 1, 2, 5, 4);
				networks[i] -> load("saves/race" + to_string(race_index) + "/"+to_string(i + 1 - number_of_cars / 2) + ".AI");

				for (int j = 0; j < 200*randDouble(); j++) {
					networks[i] -> mutate(1);
				}


			}

		}

		time_t end = time(0);
		double time = difftime(end, start);

		cout << "Finished race #" << race_index << " in " << time << " sec" << endl;
		cout << "============\n";

	}

}
