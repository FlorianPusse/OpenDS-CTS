#include"ped_pomdp.h"
#include "solver/despot.h"
#include "connector.h"
#include "HybridDespot.h"
#include "math_utils.h"
#include<random>
#include<ctime>

using namespace std;

int n_sim = 5;
int n_peds = -1; // should be smaller than ModelParams::N_PED_IN

class Simulator {
public:
    typedef pair<float, Pedestrian> PedDistPair;

    int observation_size = 4 + 2*NUM_PEDESTRIANS;

    double in_front_angle_cos = cos(ModelParams::IN_FRONT_ANGLE_DEG / 180.0 * M_PI);

    bool inFront(Path& path, COORD ped_pos, int car) const {
        if(ModelParams::IN_FRONT_ANGLE_DEG >= 180.0) {
            // inFront check is disabled
            return true;
        }

        const COORD& car_pos = path[car];
        const COORD& forward_pos = path[path.forward(car, 1.0)];

        double d0 = COORD::EuclideanDistance(car_pos, ped_pos);
        if(d0 <= 0.7) return true;

        double d1 = COORD::EuclideanDistance(car_pos, forward_pos);
        if(d1<=0) return false;

        double dot = DotProduct(forward_pos.x - car_pos.x, forward_pos.y - car_pos.y,
                ped_pos.x - car_pos.x, ped_pos.y - car_pos.y);
        double cosa = dot / (d0 * d1);

        if(!(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8)){
            cout << "\nCosa value: " << cosa << "\n";
            cout << "Dot value: " << dot << "\n";
            cout << "carpos value: " << car_pos.x << ", " << car_pos.y << "\n";
            cout << "pedpos value: " << ped_pos.x << ", " << ped_pos.y << "\n";
            cout << "d0 value: " << d0 << "\n";
            cout << "d1 value: " << d1 << "\n";
            assert(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8);
        }

        return cosa > in_front_angle_cos;
    }

    int policy(Path& path, PomdpStateWorld* state) {
        double mindist = numeric_limits<double>::infinity();
        auto& carpos = path[state->car.pos];
        double carvel = state->car.vel + 2;

        double mindist_all = numeric_limits<double>::infinity();

        // Closest pedestrian in front
        for (int i=0; i<state->num; i++) {
    		auto& p = state->peds[i];

    		double d = COORD::EuclideanDistance(carpos, p.pos);
            if (d >= 0 && d < mindist_all)
                mindist_all = d;

    		if(!inFront(path, p.pos, state->car.pos))
    		    continue;

            d = COORD::EuclideanDistance(carpos, p.pos);
            if (d >= 0 && d < mindist)
    			mindist = d;
        }

        mindist -= 4;
        mindist_all -= 1.5;

        double brakingDistance = ((carvel*3.6)*(carvel*3.6)) / (250*0.8);

        cout << "Min dist all: " << mindist_all << " Min dist: " << mindist << " Braking dist: " << brakingDistance << " ";

        if(brakingDistance > mindist || (mindist_all < 2 && state->car.vel > 0)){
            cout << "Braking\n";
            return PedPomdp::ACT_DEC;
        }

        double nextBrakingDistance = (((carvel+ModelParams::AccSpeed)*3.6)*((carvel+ModelParams::AccSpeed)*3.6)) / (250*0.8);
        if((state->car.vel < ModelParams::VEL_MAX) && nextBrakingDistance < (mindist)){
            cout << "Accelerating\n";
            return PedPomdp::ACT_ACC;
        }

        cout << "Maintaining\n";
        return PedPomdp::ACT_CUR;
    }


    void run(connector& conn) {
        worldModel = WorldModel();

        WorldStateTracker stateTracker(worldModel);
        WorldBeliefTracker beliefTracker(worldModel, stateTracker);
        PedPomdp pomdp = PedPomdp(worldModel);

        ScenarioLowerBound *lower_bound = pomdp.CreateScenarioLowerBound("SMART");
        ScenarioUpperBound *upper_bound = pomdp.CreateScenarioUpperBound("SMART", "SMART");

        //HybridDESPOT solver = HybridDESPOT(&pomdp, lower_bound, upper_bound, &python_conn);
        //DESPOT solver = DESPOT(&pomdp, lower_bound, upper_bound, python_conn);
        //memset(&DESPOT::initial_history[0], 0, (history_size + 4 + 2*NUM_PEDESTRIANS)*sizeof(float));

        // for pomdp planning and print world info
        PomdpState s;
        // for tracking world state
        PomdpStateWorld world_state;

        message *m = conn.receiveMessage();


        cout << "Initial message received\n";

        n_peds =  (int) m->pedestrianPositions.size();
        pomdp.num = n_peds;

        cout << "Set number of pedestrians in scene = " << n_peds << "\n";


        world_state.car.pos = 0;
        world_state.car.vel = 0;
        world_state.num = n_peds;

        // TODO generate initial n_peds peds -> fetch data from server
        for(int i=0; i<n_peds; i++) {
            pair<float,float> &ped = m->pedestrianPositions[i];
			world_state.peds[i].goal = -1;
			world_state.peds[i].pos = COORD(ped.first,ped.second);
            world_state.peds[i].id = i;
        }

        int num_of_peds_world = n_peds;

		delete m;

		conn.sendMessage("START\n");

        cout << "Starting...\n";

        for(int step=0; step < ModelParams::MAX_EPISODE_LENGTH; step++) {
            //ModelParams::numUpperBound = 0;
            //ModelParams::numLowerBound = 0;
            //ModelParams::numDefault = 0;


            //cout << "====================" << "step= " << step << endl;

			// receive current observation from server
			m = conn.receiveMessage();

			if (m->terminal) {
			    cout << worldModel.inCollision(world_state) << "\n";
				cout << "Reached the goal!\n";
                conn.sendMessage("RESET\n");
				break;
			}

			if(step + 1 >= ModelParams::MAX_EPISODE_LENGTH){
				cout << "Did not reach goal. Ran out of time!\n";
                conn.sendMessage("RESET\n");
				break;
			}

			// extract current path
			Path path;
			for (auto &p : m->path) {
				path.push_back(COORD(get<0>(p),get<1>(p),get<2>(p)));
			}
            worldModel.path = path;
            this->path = path;
            DESPOT::path = &path;

            //image_conn.sendMessage(path[path.size() - 1], path);

			// set current position to be 0 (will probably always stay that way)
			world_state.car.pos = 0;
			world_state.car.vel = m->carSpeed;

            for(int i=0; i<n_peds; i++) {
                pair<float,float> &ped = m->pedestrianPositions[i];
                world_state.peds[i].goal = -1;
                world_state.peds[i].pos = COORD(ped.first,ped.second);
                world_state.peds[i].id = i;
            }

            COORD &pathPos = worldModel.path[world_state.car.pos];
            //if(worldModel.inCollision(world_state)){
            //    cout << "(" << pathPos.x << ", " << pathPos.y << ", " << pathPos.theta << ") COLLISION STATE !\n";
            //}else{
            //    cout << "(" << pathPos.x << ", " << pathPos.y << ", " << pathPos.theta << ")NO COLLISION STATE!\n";
            //}

			stateTracker.updateCar(path[world_state.car.pos]);
			stateTracker.updateVel(world_state.car.vel);

			// TODO ... update the peds in stateTracker and the pedestrians
			for (int i = 0; i<num_of_peds_world; i++) {
				Pedestrian p(world_state.peds[i].pos.x, world_state.peds[i].pos.y, world_state.peds[i].id);
				stateTracker.updatePed(p);
			}

			delete m;

			// set current position to be 0 (will probably always stay that way)
			world_state.car.pos = 0;
			world_state.car.vel = m->carSpeed;

            for(int i=0; i<n_peds; i++) {
                pair<float,float> &ped = m->pedestrianPositions[i];
                world_state.peds[i].goal = -1;
                world_state.peds[i].pos = COORD(ped.first,ped.second);
                world_state.peds[i].id = i;
            }

			stateTracker.updateCar(path[world_state.car.pos]);
			stateTracker.updateVel(world_state.car.vel);

			// TODO ... update the peds in stateTracker and the pedestrians
			for (int i = 0; i<num_of_peds_world; i++) {
				Pedestrian p(world_state.peds[i].pos.x, world_state.peds[i].pos.y, world_state.peds[i].id);
				stateTracker.updatePed(p);
			}

            // TODO: Create "local" POMDP from world state? This one only contains the x closest pedestrians
            s.car.pos = world_state.car.pos;
            s.car.vel = world_state.car.vel;
            s.car.dist_travelled = world_state.car.dist_travelled;
            s.num = n_peds;
            std::vector<PedDistPair> sorted_peds = stateTracker.getSortedPeds();

            //update s.peds to the nearest n_peds peds
            for(int i=0; i<n_peds; i++) {
                s.peds[i] = world_state.peds[sorted_peds[i].second.id];
            }

            // TODO Update the belief
            beliefTracker.update();

            /*
             * 	enum {
		     *      ACT_CUR,
		     *      ACT_ACC,
		     *      ACT_DEC
	         *  };
             *
             */

            std::string goalString;

            std::map<int, PedBelief> &beliefs = beliefTracker.peds;
            for(auto it = beliefs.begin(); it != beliefs.end(); ++it){
                PedBelief &belief = it->second;

                goalString += std::to_string(belief.id) + ",";
                for(double prob: belief.prob_goals){
                    goalString += std::to_string(prob) + ",";
                }

                if(next(it) != beliefs.end()){
                    goalString += ";";
                }else{
                    goalString += "\n";
                }
            }

            int act = policy(path, &world_state);

            std::string message;

            switch (act){
                case PedPomdp::ACT_CUR:
                    message = "1;";
                    break;
                case PedPomdp::ACT_ACC:
                    message = "0;";
                    break;
                case PedPomdp::ACT_DEC:
                    message = "2;";
                    break;
                default:
                    cerr << "Unknown action!\n";
                    throw "Unknown action!\n";
            }

            message += goalString;

            conn.sendMessage(message.c_str());
        }

        //delete solver;

        /*
        cout << "final_state=[[" << endl;
        pomdp.PrintState(s);
        cout << "]]" << endl;
        cout << "total_reward= " << total_reward << endl;
        */
    }
    std::default_random_engine generator;

    COORD start, goal;

    Path path;
    WorldModel worldModel;
};

int main(int argc, char** argv) {
    int port = 1245;
    int python_port = 1246;

	if (argc >= 2){
        port = atoi(argv[1]);
	}

    Globals::config.max_policy_sim_len = 50;
    Globals::config.no_importance_sampling = true;
    Globals::config.discount = 0.990;
    Globals::config.search_depth = 50;
	Globals::config.num_scenarios = 120;
	Globals::config.time_per_move = (1.0 / ModelParams::control_freq);

	cout << "Maximum search_depth: " << Globals::config.search_depth
	    << ", num_scenarios" << Globals::config.num_scenarios << "\n";

    Seeds::root_seed(get_time_second());
    double seed = Seeds::Next();
    Random::RANDOM = Random(seed);
    cerr << "Initialized global random generator with seed " << seed << endl;

    cout << "Connect to server ...\n";

    connector conn;
    conn.establish_connection(port);
    conn.sendMessage("RESET\n");

    /*

    cout << "Connect to python server ...";

    // TODO: CHANGE
    python_connector python_conn[3];
    for(int i = 0; i < 3; ++i){
        python_conn[i].establish_connection(0);
    }

    cout << "done\n";

    cout << "Connect to train server ...";
    connector train_connector;
    train_connector.establish_connection(python_port);
    cout << "done\n";

    cout << "Connect to image server ...";
    image_connector image_conn;
    image_conn.establish_connection();
    cout << "done\n";

    */

    Simulator sim;
    for(long i=0;; i++){
        cout<<"++++++++++++++++++++++ ROUND "<<i<<" ++++++++++++++++++++"<<endl;
        sim.run(conn);
    }
}
