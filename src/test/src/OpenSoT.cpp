#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>
#include <XBotInterface/RobotInterface.h>
#include <thread>

using namespace OpenSoT;

int main(int argc, char ** argv)
{
    tasks::velocity::Cartesian::Ptr _left_arm, _right_arm;
    tasks::velocity::Postural::Ptr _posture;
    constraints::velocity::JointLimits::Ptr _joint_limits;
    constraints::velocity::VelocityLimits::Ptr _vel_limits;
     
    AutoStack::Ptr _ik_problem;
    solvers::iHQP::Ptr _solver;
     
    
    std::string path_to_config_file = XBot::Utils::getXBotConfig();
    XBot::ModelInterface::Ptr _model = XBot::ModelInterface::getModel(path_to_config_file);
    XBot::RobotInterface::Ptr robot = XBot::RobotInterface::getRobot ( path_to_config_file ); // returns a shared pointer to a robot object
    
    double dT=0.01; 
     
    Eigen::VectorXd q,dq,qr;
    
    
     q.setZero(_model->getJointNum());
     dq.setZero(_model->getJointNum());

    _model->getRobotState("home", q);
    _model->setJointPosition(q);
    _model->update();


    // Initialization of Cartesian tasks including task ids, joint position, model,
    // end-effector name, base_link from where the tasks are controlled

    _left_arm = boost::make_shared<tasks::velocity::Cartesian>("larm", q, *_model,
                                                          _model->chain("left_arm").getTipLinkName(),
                                                          _model->chain("torso").getBaseLinkName());
    // Proportional gain for the task error
    _left_arm->setLambda(0.1);
    
    _right_arm = boost::make_shared<tasks::velocity::Cartesian>("rarm", q, *_model,
                                                          _model->chain("right_arm").getTipLinkName(),
                                                          _model->chain("torso").getBaseLinkName());
    
    _right_arm->setLambda(0.1);
    
    
    
    // Ids to select rows of the right arm, in particular we are considering just the position part
    std::list<uint> pos_idx = {0, 1, 2};
    auto right_arm_pos = _right_arm % pos_idx;

    // Postural task initialization
    _posture = boost::make_shared<tasks::velocity::Postural>(q);

    // Proportional gain for the task error
    _posture->setLambda(0.01);

    Eigen::VectorXd qmax, qmin;
    // Initialization of joint limits, joint limits are taken from the model
    _model->getJointLimits(qmin, qmax);
    _joint_limits = boost::make_shared<constraints::velocity::JointLimits>(q, qmax, qmin);

    // Initialization of joint velocity limits
    _vel_limits = boost::make_shared<constraints::velocity::VelocityLimits>(M_PI, dT, q.size());
    
    /*
     *  The Math of Tasks
     */
    _ik_problem = ( (_left_arm + right_arm_pos) / _posture) << _joint_limits << _vel_limits;
    
    _solver = boost::make_shared<solvers::iHQP>(_ik_problem->getStack(), _ik_problem->getBounds(), 1e8);
    
    Eigen::Affine3d TtgtR,TtgtL;
    
    
    _right_arm->getReference(TtgtR);
    TtgtR.translation().x() += 0.5;
    _right_arm->setReference(TtgtR);
    
    _left_arm->getReference(TtgtL);
    TtgtL.translation().x() += 0.5;
    _left_arm->setReference(TtgtL);
    
    double time=0;
    while(time<3)
    {
        _model->setJointPosition(q);
        _model->update();
        
        robot->setReferenceFrom(*_model);
        robot->getPositionReference(qr);
        
        robot->setPositionReference(qr);
        robot->move();
        
        _ik_problem->update(q);
        if(_solver->solve(dq))
            q+=dq;
        else
            std::cout << "OpenSoT can not solve!" << std::endl;
        
       std::this_thread::sleep_for(std::chrono::duration<double>(dT));
       time+=dT;     
    }
}
