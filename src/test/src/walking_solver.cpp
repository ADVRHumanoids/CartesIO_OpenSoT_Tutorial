#include <cartesian_interface/ros/RosClient.h>
using namespace XBot::Cartesian;

#include <cartesian_interface/ros/RosClient.h>

using namespace XBot::Cartesian;
#define MAX_STEP 10

int main(int argc, char ** argv)
{
    RosClient cli;
    auto lfoot = cli.getTask("left_foot");
 
    auto rfoot = cli.getTask("right_foot");
    

     // ..use task
     lfoot->setLambda(0.5);
     rfoot->setLambda(0.5);

    auto lfoot_cartesian = std::dynamic_pointer_cast<CartesianTask>(lfoot);
    auto rfoot_cartesian = std::dynamic_pointer_cast<CartesianTask>(rfoot);
      
    int steps=0;
    bool turn=0;
    // if conversion was successful...
    if((lfoot_cartesian)&&(rfoot_cartesian))
    {
        Eigen::Affine3d Ttgt;
        double time = 5.0;

        while(steps!=MAX_STEP)
        {
            if(!turn)
            {
                // fill Ttgt...
                lfoot_cartesian->getPoseReference(Ttgt);
                Ttgt.translation().x() += 0.3;

                // command reaching motion
                lfoot_cartesian->setPoseTarget(Ttgt, time);

                // sleep time
                const double sleep_dt = 0.1;
            

                // wait until motion started
                while(lfoot_cartesian->getTaskState() == State::Online)
                {
                    cli.update(0, 0);
                    ros::Duration(sleep_dt).sleep();
                }

                std::cout << "Motion started" << std::endl;

                // wait until motion completed
                while(lfoot_cartesian->getTaskState() == State::Reaching)
                {
                    cli.update(0, 0);
                    ros::Duration(sleep_dt).sleep();
                }

                std::cout << "Motion completed" << std::endl;
                turn=1;
            }
            else
            {
                // fill Ttgt...
                rfoot_cartesian->getPoseReference(Ttgt);
                Ttgt.translation().x() += 0.3;

                // command reaching motion
                rfoot_cartesian->setPoseTarget(Ttgt, time);

                // sleep time
                const double sleep_dt = 0.1;
            

                // wait until motion started
                while(rfoot_cartesian->getTaskState() == State::Online)
                {
                    cli.update(0, 0);
                    ros::Duration(sleep_dt).sleep();
                }

                std::cout << "Motion started" << std::endl;

                // wait until motion completed
                while(rfoot_cartesian->getTaskState() == State::Reaching)
                {
                    cli.update(0, 0);
                    ros::Duration(sleep_dt).sleep();
                }

                std::cout << "Motion completed" << std::endl;
                turn=0;
            }
            steps++;
        }
     }
}