#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
using namespace std;
using namespace json;
using json = nlohmann::json;

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main(int argc, char* argv[]){
    uWS::Hub h;

    PID pid_steer, pid_throttle;

    //Initialize the pid variable for steering
    double init_Kp = -0.15;
    double init_Ki = -0.00001;
    double init_Kd = -3;

    // pass arguments override defacult pid gains
    if(argc>1){
        init_Kp = atof(argv[1]);
        init_Ki = atof(argv[2]);
        init_Kd = atof(argv[3]);
    }

    cout << "Steering PID Values " << init_Kp <<", "<< init_Ki <<", "<< init_Kd <<endl;
    pid_steer.Init(init_Kp, init_Ki, init_Kd);

    // Initialize pid values for throttle
    double init_Kp_throttle = -0.5;
    double init_Ki_throttle = 0;
    double init_Kd_throttle = 0;
    // When high cte reduce acceleration propotionally

    cout << "Throttle PID Values " << init_Kp_throttle <<", "<< init_Ki_throttle <<", "<< init_Kd_throttle <<endl;
    pid_throttle.Init(init_Kp_throttle, init_Ki_throttle, init_Kd_throttle);

    h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" means there's a websocket message event. 4-websocket message. 2-websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2'){
            auto s = hasData(string(data).substr(0, length));

            if (s != "") {
                auto j = parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {

                    // j[1] is the data JSON object
                    double cte = stod(j[1]["cte"].get<string>());
                    double speed = stod(j[1]["speed"].get<string>());
                    double angle = stod(j[1]["steering_angle"].get<string>());
                    double steer_value, throttle_value;
                    
                    pid_steer.UpdateError(cte);
                    steer_value = pid_steer.TotalError();

                    if (steer_value>1) {
                        steer_value = 1;
                    } else if (steer_value<-1) {
                        steer_value = -1;
                    }

                    pid_throttle.UpdateError(fabs(cte));

                    // 0.55 is a Bias term to map the throttle vales appropriately
                    throttle_value = 0.55 + pid_throttle.TotalError();

                    // speed limiter
                    if (throttle_value > 0.7) {
                        throttle_value = 0.7;
                    } else if (throttle_value < 0.05) {
                        throttle_value = 0.05;
                    }

                    // cout << "CTE: " << cte << ", Steering: " << steer_value << ", Throttle: " << throttle_value << endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });


    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
            res->end(s.data(), s.length());
        else
            res->end(nullptr, 0);
        
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        cout << "Connected!!!" << endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        cout << "Disconnected" << endl;
    });

    int port = 4567;
    if (h.listen(port)){
        cout << "Listening to port " << port << endl;
    }
    else{
        cerr << "Failed to listen to port" << endl;
        return -1;
    }
    h.run();
}
