/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez, Jonathan Gammell */

/* MPNet Authors: Ahmed Qureshi, Anthony Simeonov */
/* BASED ON OMPL RRT* SOURCE CODE, BEAR WITH ME */

// #include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/mpnet/MPNet.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

//torch
#include <torch/torch.h>
#include <torch/script.h>

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <ompl/base/goals/GoalStates.h>
#include <fstream>

void ompl::geometric::MPNet::testJointRange(){
    std::cout << "\n\n\n";
    si_->printSettings();
    std::cout << "\n\n\n";

    const float baxterJointRange[7] = {3.4033, 3.194, 6.117, 3.6647, 6.117, 6.1083, 2.67};
    
    const int dim = sizeof(baxterJointRange) / sizeof(baxterJointRange[0]);
    for (int i = 0; i < dim; i++)
    {
        std::cout << baxterJointRange[i] << "  ";
    }
    std::cout << "\n\n\n";
}

torch::Tensor ompl::geometric::MPNet::loadNormalizedPointCloud(std::string fname){
    std::ifstream infile;
    infile.open("/mpnet_data/pcd.csv");

    std::string line;
    std::vector<float> tt;

    //load into 3xN array 
    while (getline(infile, line)){
        tt.push_back(std::atof(line.c_str()));
    }

    torch::Tensor torch_tensor = torch::from_blob(tt.data(), {1, 16053});
    return torch_tensor;
}

void ompl::geometric::MPNet::baxterToMoveIt(torch::Tensor baxter_state_tensor, ompl::base::State *moveit_state){
    //swap indices    
    auto tensor_a = baxter_state_tensor.accessor<float,2>(); //accessor for the tensor
    std::vector<double> baxter_state_vec;

    for (int i = 0; i < tensor_a.size(1); i++){
        baxter_state_vec.push_back((double)tensor_a[0][i]);
    }

    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = baxter_state_vec[0];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = baxter_state_vec[1];

    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = baxter_state_vec[5];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = baxter_state_vec[6];

    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = baxter_state_vec[2];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = baxter_state_vec[3];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = baxter_state_vec[4];

    #ifdef DEBUG 
        std::cout << "Baxter state: \n" << baxter_state_vec << "\nMoveit State: \n";
        si_->printState(moveit_state);
    #endif
}

void ompl::geometric::MPNet::baxterToMoveItAndUnnormalize(torch::Tensor baxter_state_tensor, ompl::base::State *moveit_state, const float jointRange[], int dim){
    auto tensor_a = baxter_state_tensor.accessor<float,2>(); //accessor for the tensor

    std::vector<double> baxter_state_vec;
    for (int i = 0; i < dim; i++){
        baxter_state_vec.push_back((double)(tensor_a[0][i] * jointRange[i]));
    }

    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = baxter_state_vec[0];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = baxter_state_vec[1];

    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = baxter_state_vec[5];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = baxter_state_vec[6];

    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = baxter_state_vec[2];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = baxter_state_vec[3];
    moveit_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = baxter_state_vec[4];


    #ifdef DEBUG
        std::cout << "Baxter state: \n"
                << baxter_state_vec << "\nMoveit State: \n";
        si_->printState(moveit_state);
    #endif   
}

std::vector<float> ompl::geometric::MPNet::moveItToBaxter(std::vector<float> moveit_state_vector)
{
    std::vector<float> baxter_state_vector;

    for (uint i = 0; i < moveit_state_vector.size(); i++)
        baxter_state_vector.push_back(moveit_state_vector[i]);

    baxter_state_vector[0] = moveit_state_vector[0];
    baxter_state_vector[1] = moveit_state_vector[1];

    baxter_state_vector[2] = moveit_state_vector[4];
    baxter_state_vector[3] = moveit_state_vector[5];
    baxter_state_vector[4] = moveit_state_vector[6];

    baxter_state_vector[5] = moveit_state_vector[2];
    baxter_state_vector[6] = moveit_state_vector[3];

    torch::Tensor baxter_state_tensor = torch::from_blob(baxter_state_vector.data(), {1, (int)(baxter_state_vector.size())});

    #ifdef DEBUG
        std::cout << "Moveit: \n" << moveit_state_vector 
                << "\n Baxter: \n" << baxter_state_vector << "\n";
    #endif

    return baxter_state_vector;
}

std::vector<float> ompl::geometric::MPNet::normalizeVector(std::vector<float> unnormalized_vec, const float jointRange[])
{
    std::vector<float> normalized_vec;

    for (uint i = 0; i < si_->getStateDimension(); i++)
    {
        normalized_vec.push_back(unnormalized_vec[i] / jointRange[i]);
    }

    return normalized_vec;
}

std::vector<float> ompl::geometric::MPNet::moveItToBaxterAndNormalize(std::vector<float> moveit_state_vector, const float jointRange[], int dim)
{
    std::vector<float> baxter_state_vector;

    for (int i = 0; i < dim; i++)
        baxter_state_vector.push_back(moveit_state_vector[i]);

    baxter_state_vector[0] = moveit_state_vector[0];
    baxter_state_vector[1] = moveit_state_vector[1];

    baxter_state_vector[2] = moveit_state_vector[4];
    baxter_state_vector[3] = moveit_state_vector[5];
    baxter_state_vector[4] = moveit_state_vector[6];

    baxter_state_vector[5] = moveit_state_vector[2];
    baxter_state_vector[6] = moveit_state_vector[3];

    for (int i = 0; i < dim; i++)
    {
        baxter_state_vector[i] = baxter_state_vector[i] / jointRange[i];
    }

    return baxter_state_vector;
}

at::Tensor ompl::geometric::MPNet::getEncoding(){
    // create tensor for model inputs 
    std::vector<torch::jit::IValue> inputs;

    // variable for loading file 
    std::ifstream infile;
    std::string pcd_fname = "/mpnet_data/trainEnv_4_pcd_normalized.csv";
    std::cout << "PCD file: " << pcd_fname << "\n\n\n";
    infile.open(pcd_fname);

    std::string line;
    std::vector<float> tt;

    while (getline(infile, line)){
        tt.push_back(std::atof(line.c_str()));
    }

    torch::Tensor torch_tensor = torch::from_blob(tt.data(), {1, 16053});
    inputs.push_back(torch_tensor);

    // std::shared_ptr<torch::jit::script::Module> module = torch::jit::load("/mpnet_data/pytorch_load/encoder_annotated_.pt");
    std::shared_ptr<torch::jit::script::Module> module = torch::jit::load("/mpnet_data/pytorch_models/encoder_annotated_test_cpu.pt");
    assert(module != nullptr);

    at::Tensor output1 = module->forward(inputs).toTensor();

    return output1;
}

torch::Tensor ompl::geometric::MPNet::getStartGoalTensor(ompl::base::State *start_state, const ompl::base::State *goal_state, bool start_first, const float jointRange[], int dim){
    //convert to torch tensor by getting data from states
    std::vector<float> goal_vec;
    std::vector<float> start_vec;

    for (int i = 0; i < dim; i++){
        goal_vec.push_back((float)goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
        start_vec.push_back((float)start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }


    std::vector<float> normalized_start_vec = moveItToBaxterAndNormalize(start_vec, jointRange, dim);
    std::vector<float> normalized_goal_vec = moveItToBaxterAndNormalize(goal_vec, jointRange, dim);

    torch::Tensor start_tensor = torch::from_blob(normalized_start_vec.data(), {1, dim});
    torch::Tensor goal_tensor = torch::from_blob(normalized_goal_vec.data(), {1, dim});

    #ifdef DEBUG
        std::cout << "Start Vec: \n" << start_vec << "\n";
        std::cout << "Goal Vec: \n" << goal_vec << "\n";



        std::cout << "Start Vec: " << start_vec << "\n"
                << "Start Tensor: " << start_tensor << "\n"
                << "Goal Vec: " << goal_vec << "\n"
                << "Goal Tensor: " << goal_tensor << "\n";
    #endif
    
    torch::Tensor sg_cat;

    if (start_first){
        sg_cat = torch::cat({start_tensor, goal_tensor}, 1);
    }
    else {
        sg_cat = torch::cat({goal_tensor, start_tensor}, 1);
    }

    #ifdef DEBUG
        std::cout << "\n\n\nCONCATENATED START/GOAL\n\n\n" << sg_cat << "\n\n\n";  
    #endif

    return sg_cat;
}

torch::Tensor ompl::geometric::MPNet::getStartStartTensor(ompl::base::State *start_state, ompl::base::State *goal_state, bool start_first, const float jointRange[], int dim)
{
    //convert to torch tensor by getting data from states
    std::vector<float> goal_vec;
    std::vector<float> start_vec;

    for (int i = 0; i < dim; i++)
    {
        goal_vec.push_back((float)goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
        start_vec.push_back((float)start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
    std::vector<float> normalized_start_vec = moveItToBaxterAndNormalize(start_vec, jointRange, dim);
    std::vector<float> normalized_goal_vec = moveItToBaxterAndNormalize(goal_vec, jointRange, dim);

    torch::Tensor start_tensor = torch::from_blob(normalized_start_vec.data(), {1, dim});
    torch::Tensor goal_tensor = torch::from_blob(normalized_goal_vec.data(), {1, dim});

    #ifdef DEBUG
        std::cout << "Start Vec: \n"
                << start_vec << "\n";
        std::cout << "Goal Vec: \n"
                << goal_vec << "\n";


        std::cout << "Start Vec: " << start_vec << "\n"
                << "Start Tensor: " << start_tensor << "\n"
                << "Goal Vec: " << goal_vec << "\n"
                << "Goal Tensor: " << goal_tensor << "\n";
    #endif
    
    torch::Tensor sg_cat;
    if (start_first)
    {
        sg_cat = torch::cat({start_tensor, goal_tensor}, 1);
    }
    else
    {
        sg_cat = torch::cat({goal_tensor, start_tensor}, 1);
    }

    #ifdef DEBUG 
        std::cout << "\n\n\nCONCATENATED START/GOAL\n\n\n"
                << sg_cat << "\n\n\n";
    #endif

    return sg_cat;
}

void ompl::geometric::MPNet::copyGoalState(const base::State *const_goal_state, base::State *goal_state, int dim){
    for (int i = 0; i < dim; i++)
    {
        goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = const_goal_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }
}

bool ompl::geometric::MPNet::steerTo(ompl::base::State *start_state, ompl::base::State *end_state, float step_size, int dim){

    std::vector<float> start_vec;
    std::vector<float> end_vec;

    float start_val;
    float end_val;

    std::vector<float> dists;
    float distTotal = 0.0;
    float diff;

    for (int i = 0; i < dim; i++)
    {
        start_val = (float)(start_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
        start_vec.push_back(start_val);

        end_val = (float)(end_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

        diff = end_val - start_val;
        dists.push_back(diff);
        distTotal = distTotal + diff*diff;
    }

    si_->printState(start_state);
    si_->printState(end_state);
    std::cout << "dists: " << dists << "\n";
    std::cout << "distTotal: " << distTotal << "\n";

    distTotal = std::sqrt(distTotal);

    if (distTotal > 0.0){
        float incrementTotal = distTotal/(step_size/6.117);

        for (int i = 0; i < dim; i++){
            dists[i] = dists[i]/incrementTotal;
        }

        int numSegments = (int)(std::floor(incrementTotal));

        std::cout << "incrementTotal: " << incrementTotal << "\n";
        std::cout << "numSegments: " << numSegments << "\n";
        std::cout << "dists, after divide: " << dists << "\n";

        std::vector<float> state_current_vec;
        ompl::base::State *state_current = si_->allocState();

        for (int i = 0; i < dim; i++){
            state_current_vec.push_back(start_vec[i]);
            state_current->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_vec[i];
        }

        // bool valid;
        for (int i = 0; i < numSegments; i++){
            if (i%500 == 0){
                std::cout << "i: " << i << ", valid: " << si_->isValid(state_current) << "\n";
                si_->printState(state_current);

            }

            if (!si_->isValid(state_current)){
                return false;
            }

            for (int j = 0; j < dim; j++){
                state_current_vec[j] = state_current_vec[j] + dists[j];
                state_current->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = state_current_vec[j];
            }
        }

        if (!si_->isValid(end_state)){
            return false;
        }
    }

    std::cout << "Apparently edge is collision free,    ";
    bool ompl_edge_valid = si_->checkMotion(start_state, end_state);
    std::cout << "OMPL edge check validity: " << ompl_edge_valid << "\n";

    return true; //edge is valid 
}

std::vector<ompl::base::State *> ompl::geometric::MPNet::lazyStateContraction(std::vector<ompl::base::State *> path){
    for (size_t i = 0; i < (path.size() - 1); i++){
        for (size_t j = (path.size() - 1); j > i + 1; j--){
            bool ind = 0;
            ind = si_->checkMotion(path[i], path[j]);

            #ifdef DEBUG
                std::cout << "i: " << i << ", j: " << j << " ind: " << ind << "\n";
                std::cout << "path length: " << path.size() << "\n";
            #endif

            if (ind){

                #ifdef DEBUG
                    std::cout << "calling LSC again... \n";
                #endif

                std::vector<base::State *> pc;
                for (size_t k = 0; k < i+1; k++){
                    pc.push_back(path[k]);
                }
                for (size_t k = j; k < path.size(); k++){
                    pc.push_back(path[k]);
                }

                return lazyStateContraction(pc);
            }
        }
    }

    return path;
}

bool ompl::geometric::MPNet::feasibilityCheck(std::vector<base::State *> path){
    for (size_t i = 0; i < (path.size() - 1); i++){
        if (!si_->checkMotion(path[i], path[i+1])){
            return false;
        }
    }

    return true;
}

bool ompl::geometric::MPNet::feasibilityCheckOMPL(std::vector<base::State *> path)
{
    for (size_t i = 0; i < (path.size() - 1); i++){
        if (!si_->checkMotion(path[i], path[i+1])){
            return false;
        }
    }

    return true;
}

std::vector<ompl::base::State *> ompl::geometric::MPNet::replanPath(std::vector<ompl::base::State *> path, ompl::base::State *goal,
                                                                        std::shared_ptr<torch::jit::script::Module> MLP, at::Tensor obs_enc, const float jointRange[], int dim)
{    
    std::vector<ompl::base::State *> new_path;

    for (size_t i = 0; i < path.size() - 1; i++){
        if (si_->isValid(path[i])){
            new_path.push_back(path[i]);
        }
    }

    new_path.push_back(goal);
    std::vector<ompl::base::State *> return_path;

    for (size_t i = 0; i < new_path.size() - 1; i++){


        if (si_->checkMotion(new_path[i], new_path[i+1])){
            return_path.push_back(new_path[i]);
            return_path.push_back(new_path[i+1]);
        }

        else {

            int itr = 0;
            bool target_reached = false;

            torch::Tensor sg_new = getStartStartTensor(new_path[i], new_path[i+1], true, jointRange, dim);
            torch::Tensor goal_tens = sg_new.narrow(1, 7, 7);
            torch::Tensor start_tens = sg_new.narrow(1, 0, 7);

            base::State *start1_state = si_->allocState();
            base::State *start2_state = si_->allocState();
            baxterToMoveItAndUnnormalize(start_tens, start1_state, jointRange, dim);
            baxterToMoveItAndUnnormalize(goal_tens, start2_state, jointRange, dim);

            std::vector<base::State *> pA;
            std::vector<base::State *> pB;

            pA.push_back(start1_state);
            pB.push_back(start2_state);

            std::vector<torch::jit::IValue> mlp_input_1;
            std::vector<torch::jit::IValue> mlp_input_2;

            torch::Tensor mlp_input_tensor;
            bool tree = 0;

            while (!target_reached && itr < 3000)
            {
                itr = itr + 1;

                if (tree == 0)
                {
                    // concat and fill input
                    mlp_input_tensor = torch::cat({start_tens, goal_tens, obs_enc}, 1).to(at::kCUDA);
                    mlp_input_1.push_back(mlp_input_tensor);

                    // forward pass and convert to OMPL state
                    start_tens = MLP->forward(mlp_input_1).toTensor().to(at::kCPU);
                    base::State *new_state_1 = si_->allocState();
                    baxterToMoveItAndUnnormalize(start_tens, new_state_1, jointRange, dim);

                    // append path
                    pA.push_back(new_state_1);

                    // clear input
                    mlp_input_1.clear();

                    // switch to goal
                    tree = 1;
                }

                else
                {
                    // concat and fill input
                    mlp_input_tensor = torch::cat({goal_tens, start_tens, obs_enc}, 1).to(at::kCUDA);
                    mlp_input_2.push_back(mlp_input_tensor);

                    // forward pass and convert to OMPL state
                    goal_tens = MLP->forward(mlp_input_2).toTensor().to(at::kCPU);
                    base::State *new_state_2 = si_->allocState();
                    baxterToMoveItAndUnnormalize(goal_tens, new_state_2, jointRange, dim);

                    // append path
                    pB.push_back(new_state_2);

                    // clear input
                    mlp_input_2.clear();

                    // switch to start
                    tree = 0;
                }

                target_reached = si_->checkMotion(pA.back(), pB.back());
            }

            if (!target_reached){
                std::cout << "Failed to replan\n";
            }

            else {
                for (size_t i = 0; i < pA.size(); i ++){
                    return_path.push_back(pA[i]);
                }
                for (int j = ((int)(pB.size()) - 1); j > -1; j--){
                    return_path.push_back(pB[j]);
                }
            }
        }    
    }
    return return_path;
}

std::vector<ompl::base::State *> ompl::geometric::MPNet::MPNetSolve()
{

    int dim = si_->getStateDimension();
    const float baxterJointRange[7] = {3.4033, 3.194, 6.117, 3.6647, 6.117, 6.1083, 2.67};

    // get obstacle encoding
    at::Tensor obs_enc = getEncoding();

    // get start and goal tensors
    base::Goal *goal = pdef_->getGoal().get();

    const base::State *const_goal_state = goal->as<ompl::base::GoalStates>()->getState(0);
    base::State *goal_state = si_->allocState();
    copyGoalState(const_goal_state, goal_state, dim);

    base::State *start_state = pdef_->getStartState(0);

    // get start, goal in tensor form
    bool start_first = true;
    torch::Tensor sg = getStartGoalTensor(start_state, goal_state, start_first, baxterJointRange, dim);
    torch::Tensor gs = getStartGoalTensor(start_state, goal_state, !start_first, baxterJointRange, dim);

    torch::Tensor start_only = sg.narrow(1, 0, 7);
    torch::Tensor goal_only = sg.narrow(1, 7, 7);

    torch::Tensor start1 = sg.narrow(1, 0, 7); // path start
    torch::Tensor start2 = sg.narrow(1, 7, 7); // path goal

    torch::Tensor mlp_input_tensor;

    bool target_reached = false;
    int step = 0;
    bool tree = 0;

    float default_step = 0.01;
    float feas_step = 0.01;
    float step_size = default_step;
    si_->setStateValidityCheckingResolution(step_size);

    std::vector<torch::jit::IValue> mlp_input_1;
    std::vector<torch::jit::IValue> mlp_input_2;
    
    std::shared_ptr<torch::jit::script::Module> MLP;
    // MLP = torch::jit::load("/mpnet_data/pytorch_gpu_load/mlp_annotated_test_gpu.pt");
    // MLP = torch::jit::load("/gpu_model_load_test/mlp_annotated_test_gpu.pt");
    MLP = torch::jit::load("/mpnet_data/pytorch_models/mlp_annotated_test_gpu.pt");
    MLP->to(at::kCUDA);
    assert(module != nullptr);

    std::vector<base::State *> path1;
    path1.push_back(start_state);

    std::vector<base::State *> path2;
    path2.push_back(goal_state);

    while (!target_reached && step < 3000){
        step = step + 1;

        if (tree == 0){

            mlp_input_tensor = torch::cat({start1, start2, obs_enc}, 1).to(at::kCUDA);

            mlp_input_1.push_back(mlp_input_tensor);

            auto mlp_output = MLP->forward(mlp_input_1);

            start1 = mlp_output.toTensor().to(at::kCPU);

            base::State *new_state_1 = si_->allocState();
            baxterToMoveItAndUnnormalize(start1, new_state_1, baxterJointRange, dim);

            // append path 
            path1.push_back(new_state_1);

            // clear input 
            mlp_input_1.clear();

            // switch to goal 
            tree = 1;
        }

        else {
            // concat and fill input 
            // mlp_input_tensor = torch::cat({start2, start1, obs_enc}, 1);
            mlp_input_tensor = torch::cat({start2, start1, obs_enc}, 1).to(at::kCUDA);
            mlp_input_2.push_back(mlp_input_tensor);

            // forward pass and convert to OMPL state
            start2 = MLP->forward(mlp_input_2).toTensor().to(at::kCPU);
            base::State *new_state_2 = si_->allocState();
            baxterToMoveItAndUnnormalize(start2, new_state_2, baxterJointRange, dim);

            // append path 
            path2.push_back(new_state_2);

            // clear input 
            mlp_input_2.clear();

            // switch to start
            tree = 0;
        }
        target_reached = si_->checkMotion(path1.back(), path2.back());
    }

    #ifdef DEBUG
        std::cout << "Checking start and goal from path 1 and 2: \n";
        si_->printState(path1[0]);
        si_->printState(path2[0]);

        std::cout << "Path 1 length: " << path1.size() << ", Path 2 length: " << path2.size() << "\n";
    #endif

    std::vector<base::State *> path;
    if (target_reached){

        for (size_t i = 0; i < path1.size(); i++){
            path.push_back(path1[i]);
        }

        for (int j = ((int)(path2.size()) - 1); j > -1; j--){
            path.push_back(path2[j]);
        }
    }
    else{
        std::cout << "\n\n\nTARGET NOT REACHED\n\n\n";
    }

    #ifdef DEBUG
        std::cout << "Checking start and goal from overall path: \n";
        si_->printState(path[0]);
        si_->printState(path.back());
    #endif

    path = lazyStateContraction(path);

    si_->setStateValidityCheckingResolution(feas_step);

    bool is_feasible = feasibilityCheck(path);

    si_->setStateValidityCheckingResolution(step_size);


    if (is_feasible){
        std::cout << "We're done, let's go home\n";
        return path;
    }

    else {
        int sp = 0;

        while (!is_feasible && sp < 10){

            if (sp == 0){
                step_size = default_step * 0.8;
            }
            else if (sp == 1){
                step_size = default_step * 0.6;
            }
            else if (sp >= 2){
                step_size = default_step * 0.4;
            }

            si_->setStateValidityCheckingResolution(step_size);

            sp = sp + 1;
            path = replanPath(path, goal_state, MLP, obs_enc, baxterJointRange, dim);

            if (path.size() > 0){

                path = lazyStateContraction(path);

                si_->setStateValidityCheckingResolution(feas_step);

                is_feasible = feasibilityCheck(path);

                if (is_feasible){
                    std::cout << "Replanning successful, we're done, let's go home\n";
                    return path;
                }
            }
        }

        // we failed, return path with only start state 
        path.clear();
        path.push_back(start_state);
        std::cout << "Failed to plan, returning path with only start state\n";
        std::cout << "\n\n\n\n";
        return path; 
    }
}

ompl::geometric::MPNet::MPNet(const base::SpaceInformationPtr &si)
  : base::Planner(si, "MPNet")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &MPNet::setRange, &MPNet::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &MPNet::setGoalBias, &MPNet::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &MPNet::setRewireFactor, &MPNet::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &MPNet::setKNearest, &MPNet::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &MPNet::setDelayCC, &MPNet::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &MPNet::setTreePruning, &MPNet::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &MPNet::setPruneThreshold, &MPNet::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &MPNet::setPrunedMeasure, &MPNet::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &MPNet::setInformedSampling, &MPNet::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &MPNet::setSampleRejection, &MPNet::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &MPNet::setNewStateRejection,
                                &MPNet::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &MPNet::setAdmissibleCostToCome,
                                &MPNet::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &MPNet::setOrderedSampling, &MPNet::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &MPNet::setBatchSize, &MPNet::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &MPNet::setFocusSearch, &MPNet::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &MPNet::setNumSamplingAttempts,
                                        &MPNet::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::MPNet::~MPNet()
{
    freeMemory();
}

void ompl::geometric::MPNet::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();

    // Calculate some constants:
    calculateRewiringLowerBounds();

    // baxterJointRange = {3.4033, 3.194, 6.1083, 2.67, 6.117, 3.6647, 6.117};
    // setBaxterJointRange();
}

void ompl::geometric::MPNet::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;

}


ompl::base::PlannerStatus ompl::geometric::MPNet::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    bool symCost = opt_->isSymmetric();

    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->cost = opt_->identityCost();
            nn_->add(motion);
            startMotions_.push_back(motion);
        }

        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    // No else

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> nbh;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    std::vector<int> valid;
    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    if (bestGoalMotion_)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_);

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);

    while (ptc == false)
    {
        iterations_++;

        // sample random state (with goal biasing)
        // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
        // states.
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        {
            // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
            // loop and return to try again
            if (!sampleUniform(rstate))
                continue;
        }

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

            // Find nearby neighbors of the new motion
            getNeighbors(motion, nbh);

            rewireTest += nbh.size();
            ++statesGenerated;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (valid.size() < nbh.size())
                valid.resize(nbh.size());
            std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            if (delayCC_)
            {
                // calculate all costs and distances
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

                // collision check until a valid motion is found
                //
                // ASYMMETRIC CASE: it's possible that none of these
                // neighbors are valid. This is fine, because motion
                // already has a connection to the tree through
                // nmotion (with populated cost fields!).
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin() + nbh.size(); ++i)
                {
                    if (nbh[*i] == nmotion ||
                        ((!useKNearest_ || si_->distance(nbh[*i]->state, motion->state) < maxDistance_) &&
                         si_->checkMotion(nbh[*i]->state, motion->state)))
                    {
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];
                        valid[*i] = 1;
                        break;
                    }
                    else
                        valid[*i] = -1;
                }
            }
            else  // if not delayCC
            {
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            if ((!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                si_->checkMotion(nbh[i]->state, motion->state))
                            {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];
                                valid[i] = 1;
                            }
                            else
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                        valid[i] = 1;
                    }
                }
            }

            if (useNewStateRejection_)
            {
                if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
                {
                    nn_->add(motion);
                    motion->parent->children.push_back(motion);
                }
                else  // If the new motion does not improve the best cost it is ignored.
                {
                    si_->freeState(motion->state);
                    delete motion;
                    continue;
                }
            }
            else
            {
                // add motion to the tree
                nn_->add(motion);
                motion->parent->children.push_back(motion);
            }

            bool checkForSolution = false;
            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {
                    base::Cost nbhIncCost;
                    if (symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                        if (valid[i] == 0)
                        {
                            motionValid =
                                (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                                si_->checkMotion(motion->state, nbh[i]->state);
                        }
                        else
                        {
                            motionValid = (valid[i] == 1);
                        }

                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent(nbh[i]);

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->incCost = nbhIncCost;
                            nbh[i]->cost = nbhNewCost;
                            nbh[i]->parent->children.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);

                            checkForSolution = true;
                        }
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                motion->inGoal = true;
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                bool updatedSolution = false;
                if (!bestGoalMotion_ && !goalMotions_.empty())
                {
                    // We have found our first solution, store it as the best. We only add one
                    // vertex at a time, so there can only be one goal vertex at this moment.
                    bestGoalMotion_ = goalMotions_.front();
                    bestCost_ = bestGoalMotion_->cost;
                    updatedSolution = true;

                    OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                                "vertices in the graph)",
                                getName().c_str(), bestCost_, iterations_, nn_->size());
                }
                else
                {
                    // We already have a solution, iterate through the list of goal vertices
                    // and see if there's any improvement.
                    for (auto &goalMotion : goalMotions_)
                    {
                        // Is this goal motion better than the (current) best?
                        if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                        {
                            bestGoalMotion_ = goalMotion;
                            bestCost_ = bestGoalMotion_->cost;
                            updatedSolution = true;

                            // Check if it satisfies the optimization objective, if it does, break the for loop
                            if (opt_->isSatisfied(bestCost_))
                            {
                                break;
                            }
                        }
                    }
                }

                if (updatedSolution)
                {
                    if (useTreePruning_)
                    {
                        pruneTree(bestCost_);
                    }

                    if (intermediateSolutionCallback)
                    {
                        std::vector<const base::State *> spath;
                        Motion *intermediate_solution =
                            bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                        // Push back until we find the start, but not the start itself
                        while (intermediate_solution->parent != nullptr)
                        {
                            spath.push_back(intermediate_solution->state);
                            intermediate_solution = intermediate_solution->parent;
                        }

                        intermediateSolutionCallback(this, spath, bestCost_);
                    }
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approxDist)
            {
                approxGoalMotion = motion;
                approxDist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
            break;
    }

    // Add our solution (if it exists)
    Motion *newSolution = nullptr;
    if (bestGoalMotion_)
    {
        // We have an exact solution
        newSolution = bestGoalMotion_;
    }
    else if (approxGoalMotion)
    {
        // We don't have a solution, but we do have an approximate solution
        newSolution = approxGoalMotion;
    }
    // No else, we have nothing

    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        // set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
            psol.setApproximate(approxDist);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        // pdef_->addSolutionPath(psol);
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;
    // No else, we have nothing

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

    // MPNET BELOW HERE: 

    std::vector<base::State *> mpnet_path;

    auto start = std::chrono::high_resolution_clock::now();

    mpnet_path = MPNetSolve();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    bool is_feasible = false;
    if (mpnet_path.size() > 0){
        is_feasible = true;
        std::cout << "\n\n\nMPNet took " << duration.count() << " ms to find feasible path\n\n\n";
        std::cout << "Planned path with : " << mpnet_path.size() << " states\n";
    }

// UNCOMMENT THIS BLOCK BELOW TO RETURN PATH FROM MPNET TO MOVEIT
///////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool solved = false;
    bool approximate = false; //not sure what to do with this for MPNet

    if (is_feasible)
    {
        /* set the solution path */
        auto solution_path(std::make_shared<PathGeometric>(si_));
        // for (int i = path.size() - 1; i >= 0; --i)
        for (size_t i = 0; i < mpnet_path.size(); i++)
        {
            solution_path->append(mpnet_path[i]);
        }
        // base::PlannerSolution psol(solution_path);
        // psol.setApproximate(-1.0);
        // psol.setPlannerName(getName());
        pdef_->addSolutionPath(solution_path, approximate, 0.0, getName());
        // pdef_->addSolutionPath(psol);
        solved = true;
    }

    return base::PlannerStatus(solved, approximate);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////



    // // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false
    // return base::PlannerStatus(newSolution != nullptr, bestGoalMotion_ == nullptr);
}

void ompl::geometric::MPNet::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}

void ompl::geometric::MPNet::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::MPNet::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::MPNet::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::MPNet::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

int ompl::geometric::MPNet::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            nn_->add(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                nn_->add(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        nn_->add(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // Remove it
                    goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                       goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
            // Add the motion back to the NN struct:
            nn_->add(r);

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void ompl::geometric::MPNet::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::geometric::MPNet::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::geometric::MPNet::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::geometric::MPNet::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::geometric::MPNet::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::geometric::MPNet::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::MPNet::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::MPNet::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::MPNet::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}

bool ompl::geometric::MPNet::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::MPNet::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}
