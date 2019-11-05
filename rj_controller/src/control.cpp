#include "CombinedNodeException.h"
#include "CombinedROSNode.h"
#include "CarConstants.h"
#include "LateralFeedforward.h"
#include "LateralGains.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <map>


#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <map>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

int main(int argc, char **argv) 
{   
    ros::init(argc, argv, "controller");
    CombinedROSNode *LateralNode = new CombinedROSNode();

    //Dynamic reconfigure setup
    dynamic_reconfigure::Server<combined_controller::long_gainsConfig> server;
    dynamic_reconfigure::Server<combined_controller::long_gainsConfig>::CallbackType f;
    f = boost::bind(&CombinedROSNode::gainsCallback, LateralNode,_1,_2);
    server.setCallback(f);
    
    float speed, k_lat, k_head, k_o;
    string input;
    ifstream file_input;
    bool error;
    
    LateralNode->getGains("/home/autodrive/Desktop/catkin_ws/src/combined_controller/src/gains.txt");

    try {
        LateralNode->Initialize(argc, argv);
        LateralNode->UpdateLoop();
    }
    catch (const CombinedNodeException e) {
        cout << "\n" << e.what() << endl;
        cout << "Combined Node Exception Recieved - Stopping lateral_controller node..." << endl;
    }

    LateralNode->Cleanup();
    return 0;
}
