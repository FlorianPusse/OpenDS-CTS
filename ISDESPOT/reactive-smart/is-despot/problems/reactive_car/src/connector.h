#ifndef CONNECTOR_DESPOT
#define CONNECTOR_DESPOT

#include <sys/socket.h>
#include <sys/un.h>

#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <string>
#include <arpa/inet.h>
#include <errno.h>
#include <utility>
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include <tuple>
#include "state.h"
#include "Path.h"
#include "assert.h"
#include "../../../include/core/policy.h"

#define NUM_PEDESTRIANS 4

using namespace std;

class message {
public:
	// pythonConnector.sendMessage(r.terminal + ";" + reward + ";" + converted_angle + ";" + cast_obs + ";" + String.join(",",convertedPath) + "\n");
	bool terminal;
	float reward;
	float angle;
	pair<float, float> carPos;
	float carSpeed;
	vector<pair<float, float> > pedestrianPositions;
	vector<tuple<float, float, float> > path;

	void print() {
		cout << "Terminal: " << terminal << "\n";
		cout << "Reward: " << reward << "\n";
		cout << "Angle: " << angle << "\n";
		cout << "Car: (" << carPos.first << ", " << carPos.second << "), " << carSpeed;
		cout << "Pedestrians: ";
		for (auto &p : pedestrianPositions) {
			cout << "(" << p.first << ", " << p.second << ") ";
		}
		cout << "\n";
		cout << "Path: ";
		for (auto &p : path) {
			cout << "(" << get<0>(p) << ", " << get<1>(p) << ", " << get<2>(p) << ") ";
		}
		cout << "\n";
	}
};

class image_connector {
	int sockfd;
	string total;
public:
	void sendMessage(COORD& goal_pos, Path& path) {
	    string message;
	    message += to_string(goal_pos.x) + "," + to_string(goal_pos.y) + ";";

	    for(int i = 0; i < path.size(); i++){
            COORD& c = path[i];
            message += to_string(c.x) + "," + to_string(c.y) + ",";
	    }

	    message.pop_back();
	    message += "\n";

		send(sockfd, message.c_str(), strlen(message.c_str()), 0);
	}

	int establish_connection() {
        if ((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
            printf("\n Socket creation error \n");
            return -1;
        }

        struct sockaddr_un saddr;
        memset(&saddr, 0, sizeof(struct sockaddr_un));
        saddr.sun_family = AF_UNIX;
        strcpy(saddr.sun_path, "/tmp/python_unix_sockets_image");

        int tries = 0;
        int res;
        while ((res = connect(sockfd, (struct sockaddr *) &saddr, sizeof(struct sockaddr_un))) < 0) {
            std::this_thread::sleep_for (std::chrono::seconds(1));
            tries++;

            if(tries == 5){
                printf("\nConnection Failed: %s\n", strerror(errno));
                exit(-1);
            }
        }

        return sockfd;
	}

};

class connector {
	int sockfd;
	string total;
public:

	void split(string s, string delimiter, vector<string> &out) {
		size_t pos = 0;
		std::string token;
		while ((pos = s.find(delimiter)) != std::string::npos) {
			token = s.substr(0, pos);
			out.push_back(token);
			s.erase(0, pos + delimiter.length());
		}
		out.push_back(s);
	}

	message* receiveMessage() {
		int receivedBytes = 0;
		char buffer[1024] = { 0 };

		while (total.find("\n") == string::npos) {
			receivedBytes = recv(sockfd, buffer, 1024, 0);
			string convertedBytes = string(buffer, receivedBytes);
			total += convertedBytes;
		}

		int pos = total.find("\n");
		string row = total.substr(0, pos);
		total.erase(0, pos + 1);

		message *m = new message();

		vector<string> tokens;
		split(row, ";", tokens);

		if (tokens[0] == "true") {
			m->terminal = true;
		}
		else {
			m->terminal = false;
		}
		m->reward = stof(tokens[1]);
		m->angle = stof(tokens[2]);

		m->carPos = pair<float, float>(stof(tokens[3]), stof(tokens[4]));
		m->carSpeed = stof(tokens[5]);

		for (int i = 0; i < NUM_PEDESTRIANS; ++i) {
			float x = stof(tokens[6 + i * 2]);
			float z = stof(tokens[7 + i * 2]);
			pair<float, float> ped_pos(x, z);
			m->pedestrianPositions.push_back(ped_pos);
		}

		string pathString = tokens[6 + NUM_PEDESTRIANS * 2];

		tokens.clear();
		split(pathString, ",", tokens);
		int pathTokenLength = tokens.size();

		for (int i = 0; i < pathTokenLength; i+= 3) {
			float x = stof(tokens[i]);
			float z = stof(tokens[i + 1]);
			float theta = stof(tokens[i + 2]);
			tuple<float, float, float> path_entry(x, z, theta);
			m->path.push_back(path_entry);
		}

		return m;
	}

	void sendMessage(const char* m) {
		send(sockfd, m, strlen(m), 0);
	}

	int establish_connection(int port) {

        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("\n Socket creation error \n");
            return -1;
        }

        struct sockaddr_in *serv_addr = new struct sockaddr_in();
        memset(serv_addr, '0', sizeof(struct sockaddr_in));
        serv_addr->sin_family = AF_INET;
        serv_addr->sin_port = htons(port);

        cout << "Connect to port: " << port << "\n";

        // Convert IPv4 and IPv6 addresses from text to binary form
        if (inet_pton(AF_INET, "127.0.0.1", &serv_addr->sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }

        int tries = 0;
        int res;
        while ((res = connect(sockfd, (struct sockaddr *) serv_addr, sizeof(struct sockaddr_in))) < 0) {
            std::this_thread::sleep_for (std::chrono::seconds(1));
            tries++;

            if(tries == 5){
                printf("\nConnection Failed: %s\n", strerror(errno));
                exit(-1);
            }
        }

		return sockfd;
	}

};

class python_connector {
	int sockfd;
	string total;
	string path;
public:
    int num_elements = history_size + 4 + 2*NUM_PEDESTRIANS;
    int num_peds = 4;

	void receiveBinaryMessage(float* history, ValuedAction& action) {
	    size_t len = (history_size + 2)*sizeof(float);

		int receivedBytes = 0;
		int totalReceivedBytes = 0;

		while (totalReceivedBytes < len) {
			receivedBytes = recv(sockfd, history + (totalReceivedBytes / sizeof(float)), len - totalReceivedBytes, 0);
			totalReceivedBytes += receivedBytes;
		}

        action.action = (int) history[history_size];
        action.value = (double) history[history_size + 1];
	}

	void receiveBinaryMessageAll(vector<float*>& newHistories, vector<ValuedAction>& valuedActions){
        size_t len = (history_size + 2)*sizeof(float);
        size_t num_elements = newHistories.size();

        for(int i = 0; i < num_elements; ++i){
            int receivedBytes = 0;
            int totalReceivedBytes = 0;

            while (totalReceivedBytes < len) {
                receivedBytes = recv(sockfd, &newHistories[i][totalReceivedBytes / sizeof(float)], len - totalReceivedBytes, 0);
                totalReceivedBytes += receivedBytes;
            }

            ValuedAction val;
            val.action = (int) newHistories[i][history_size];
            val.value = (double) newHistories[i][history_size + 1];
            valuedActions.push_back(val);
        }
	}

    int observation_size = 12;

	float* createBinaryMessageAll(Path& path, float* hist, vector<State*>& states, size_t &len){
        assert(path.size() != 0);

        size_t buf_size = 1 + history_size + observation_size*states.size();

        // history + (observation_size)*num_states
        float* buf = new float[buf_size];
        buf[0] = states.size();

        memcpy(&buf[1], hist, sizeof(float)*history_size);

        for(int i = 0; i < states.size(); i++){
            PomdpState* state = dynamic_cast<PomdpState*>(states[i]);

            COORD carPos = path.at(state->car.pos);

            buf[1 + history_size + (i*observation_size)] = carPos.x;
            buf[1 + history_size + (i*observation_size) + 1] = carPos.y;
            buf[1 + history_size + (i*observation_size) + 2] = carPos.theta;
            buf[1 + history_size + (i*observation_size) + 3] = state->car.vel;

            for(int j = 0; j < state->num; ++j){
                PedStruct& ped = state->peds[j];
                buf[1 + history_size + (i*observation_size) + 4 + (2*j)] = ped.pos.x;
                buf[1 + history_size + (i*observation_size) + 4 + (2*j) + 1] = ped.pos.y;
            }

            if(NUM_PEDESTRIANS - state->num > 0){
                memset(&buf[1 + history_size + (i*observation_size) + 4 + 2*state->num], 0, sizeof(float)*2*(NUM_PEDESTRIANS - state->num));
            }
        }


        len = sizeof(float) * buf_size;

        return buf;
	}

	float* createBinaryMessage(Path& path, float* hist, PomdpState* state, size_t &len){
        assert(path.size() != 0);
        COORD carPos = path.at(state->car.pos);

        hist[history_size] = carPos.x;
        hist[history_size + 1] = carPos.y;
        hist[history_size + 2] = carPos.theta;
        hist[history_size + 3] = state->car.vel;

        for(int i = 0; i < state->num; ++i){
            PedStruct& ped = state->peds[i];
            hist[history_size + 4 + (2*i)] = ped.pos.x;
            hist[history_size + 4 + (2*i) + 1] = ped.pos.y;
        }

        if(NUM_PEDESTRIANS - state->num > 0){
            memset(&hist[history_size + 4 + 2*state->num],0, sizeof(float)*2*(NUM_PEDESTRIANS - state->num));
        }

        len = sizeof(float) * num_elements;

        return hist;
	}

	void sendMessage(const float* m, size_t len) {
    	send(sockfd, m, len, 0);
    }

	int establish_connection(int number) {
	    path = "/tmp/python_unix_sockets_example";
        path += to_string(number);

	    if ((sockfd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
            printf("\n Socket creation error \n");
            return -1;
        }

        struct sockaddr_un saddr;
        memset(&saddr, 0, sizeof(struct sockaddr_un));
        saddr.sun_family = AF_UNIX;
        strcpy(saddr.sun_path,path.c_str());

        int res;
        if ((res = connect(sockfd, (struct sockaddr *) &saddr, sizeof(struct sockaddr_un))) < 0) {
            printf("\nUnix Connection Failed %d: %s\n", errno, strerror(errno));
            return -1;
        }

		return sockfd;
	}
};

#endif