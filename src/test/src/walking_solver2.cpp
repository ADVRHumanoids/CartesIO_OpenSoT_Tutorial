#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <XBotInterface/RobotInterface.h>
#include <thread>
#define NWALK 10

using namespace XBot::Cartesian;

int main(int argc, char **argv)
{
    /* Part 0: contructing the solver object */

    // an option structure which is needed to make a model
    XBot::ConfigOptions xbot_cfg;

    // set the urdf and srdf path..
    xbot_cfg.set_urdf_path(URDF_PATH);
    xbot_cfg.set_srdf_path(SRDF_PATH);

    // the following call is needed to generate some default joint IDs
    xbot_cfg.generate_jidmap();

    // some additional parameters..
    xbot_cfg.set_parameter("is_model_floating_base", true);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(xbot_cfg);
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    XBot::RobotInterface::Ptr robot = XBot::RobotInterface::getRobot ( path_to_config_file ); // returns a shared pointer to a robot object

    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    

    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    const double dt = 0.01;
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile(IK_PB_PATH);
    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );



    // inspect properties of "left_foot" task
    std::string task_name = "left_foot";
    auto lfoot = solver->getTask(task_name);
    
    // inspect properties of "right_foot" task
    task_name = "right_foot";
    auto rfoot = solver->getTask(task_name);



    // check that "left_foot" is actually a Cartesian type task
    auto lfoot_cartesian = std::dynamic_pointer_cast<CartesianTask>(lfoot);
    auto rfoot_cartesian = std::dynamic_pointer_cast<CartesianTask>(rfoot);
    
    if((!lfoot_cartesian)||(!rfoot_cartesian))
    {
        throw std::runtime_error("Unexpected task type!");
    }


    Eigen::Affine3d Trefr,Trefl;
    int current_state = 0; // hand-crafted finite state machine!
    double time = 0;
    Eigen::VectorXd q, qdot, qddot;
    Eigen::VectorXd qr, qdotr;
    bool turn=1; // 0 means Left 1 right
    int walk=0;
    rfoot_cartesian->getPoseReference(Trefr);
    lfoot_cartesian->getPoseReference(Trefl);
    while(true)
    {
        if(current_state == 0) // here we command a reaching motion
        {
            if(turn)
            {
                std::cout << "Commanding right foot forward 0.3m in 3.0 secs" << std::endl;

                Trefr.translation()[0] += 0.3;
                double target_time = 3.0;
                rfoot_cartesian->setPoseTarget(Trefr, target_time);
            }
            else
            {
                std::cout << "Commanding left foot forward 0.3m in 3.0 secs" << std::endl;

                Trefl.translation()[0] += 0.3;
                double target_time = 3.0;
                lfoot_cartesian->setPoseTarget(Trefl, target_time);
            }

            current_state++;
        }

        if(current_state == 1) // here we check that the reaching started
        {
            if(turn)
            {
                if(rfoot_cartesian->getTaskState() == State::Reaching)
                {
                    std::cout << "Motion started!" << std::endl;
                    current_state++;
                }
            }
            else
            {
                if(lfoot_cartesian->getTaskState() == State::Reaching)
                {
                    std::cout << "Motion started!" << std::endl;
                    current_state++;
                }   
            }
        }

        if(current_state == 2) // here we wait for it to be completed
        {
            if(turn)
            {
                if(rfoot_cartesian->getTaskState() == State::Online)
                {
                    Eigen::Affine3d T;
                    rfoot_cartesian->getCurrentPose(T);

                    std::cout << "Motion completed, final error is " <<
                                (T.inverse()*Trefr).translation().norm() << std::endl;
                turn=0;
                walk++;
                if(walk==NWALK)    
                    current_state++;
                else
                    current_state=0;
                
                } 
            }
            else
            {
                if(lfoot_cartesian->getTaskState() == State::Online)
                {
                    Eigen::Affine3d T;
                    lfoot_cartesian->getCurrentPose(T);

                    std::cout << "Motion completed, final error is " <<
                                (T.inverse()*Trefl).translation().norm() << std::endl;
                turn=1;
                walk++;
                if(walk==NWALK)    
                    current_state++;
                else
                    current_state=0;
                }   
                
            }

        }


        if(current_state == 3) break;

        // update and integrate model state
        solver->update(time, dt);

        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        model->getJointAcceleration(qddot);

        q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
        qdot += dt * qddot;

        model->setJointPosition(q);
        model->setJointVelocity(qdot);
        model->update();
        
        robot->setReferenceFrom(*model);
        robot->getPositionReference(qr);
        robot->getVelocityReference(qdotr);
        
        robot->setPositionReference(qr);
        robot->setVelocityReference(qdotr);
        robot->move();
        

        // roughly loop at 100 Hz
        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
        time += dt;
    }


}
