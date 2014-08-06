#include "Driver.hpp"
#include <iostream>
#include <ros/ros.h>
#include "dvl_driver/DVL.h"

using namespace dvl_teledyne;

void usage()
{
    std::cerr << "dvl_teledyne_read DEVICE" << std::endl;
}

int main(int argc, char **argv)
{
 /*   if (argc != 2)
    {
        usage();
        return 1;
    }
*/
    //Set up Ros
    ros::init(argc, argv, "DVL");
    ros::NodeHandle n;

    ros::Publisher dvl_message = n.advertise<dvl_driver::DVL>("DVL", 1000);

    dvl_teledyne::Driver driver;
    driver.open(argv[1]);
    driver.setReadTimeout(base::Time::fromSeconds(5));
    driver.read();

    char const* coord_systems[4] = { "BEAM", "INSTRUMENT", "SHIP", "EARTH" };
    std::cout << "Device outputs its data in the " << coord_systems[driver.outputConf.coordinate_system] << " coordinate system" << std::endl;


    std::cout << "Time Seq ";
    for (int beam = 0; beam < 4; ++beam)
        std::cout << " range[" << beam << "] velocity[" << beam << "] evaluation[" << beam << "]";
    std::cout << std::endl;

    while(true)
    {
        driver.read();

        dvl_driver::DVL msg;
        BottomTracking const& tracking = driver.bottomTracking;
        std::cout << tracking.time.toString() << " " << driver.status.seq;
        msg.time = tracking.time.toString();
        msg.seq = driver.status.seq;
        for (int beam = 0; beam < 4; ++beam){
            std::cout << " " << tracking.range[beam] << " " << tracking.velocity[beam] << " " << tracking.evaluation[beam];
            msg.range[beam] = tracking.range[beam];
            msg.velocity[beam]=tracking.velocity[beam];
            msg.evaluation[beam]=tracking.evaluation[beam];
        }

        dvl_message.publish(msg);
        std::cout << std::endl;
    }
}


