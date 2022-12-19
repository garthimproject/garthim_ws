#include <rl_metrics/rl_metrics.h>
namespace rl_metrics {
    string folderExper = "src/rl_missions/results/";
    int index_experiment;
    MeasureQ::MeasureQ(){
        ros::NodeHandle nh_private("~");
        /*
        ros::NodeHandle nh;
        bool start=false;
        while (ros::ok() && !start) {
            nh.param("rl_metrics_start",start,start);
            ros::Duration(60).sleep();
        }
        */
        nh_private.param("total_experiments", this->total_experiments, 1000);
        double DISTANCE_MAX = 1.5;      //Distancia maxima al target dist_max
        int DISTANCE_LEVELS = 8;        //Niveles de disatncia al target (estado 0)
        int ORIENTATION_LEVELS = 8;     //Niveles de diferencia de orientación con el target (estado 1)
        double ACTION_TIME =1;//0.1;  
        int num_actions = 6;
        int STATE_SIZE = 3;
        std::vector<double> time_options = {0.5,1,2};
        this->physical_handle = std::unique_ptr<PhysicalHandle>(
            new PhysicalHandle(DISTANCE_MAX, DISTANCE_LEVELS,
            ORIENTATION_LEVELS, time_options.size(), ACTION_TIME, 
            num_actions, STATE_SIZE, time_options)
        );    
        int _numStates= time_options.size()*DISTANCE_LEVELS*ORIENTATION_LEVELS; 
        std::vector<std::vector<double>> Q;
        Q.assign( _numStates ,vector<double>(num_actions,0));
        std::vector<double> V;
        V.assign( _numStates,0);

        string folderOwned = "src/rl_metrics/results";
        chdir("../../..");
        system("pwd");
        ifstream filefiles_i, filefiles_i_Exper;
        ofstream filefiles_o;
        string folderfilescount = folderOwned + "/.files";
        filefiles_i.open(folderfilescount.c_str());
        string str_aux;
        filefiles_i >> str_aux;
        filefiles_i >> id_experiment_;
        if (str_aux == "") {
            str_aux = "0";
            id_experiment_ = "0";
            cout << "couldnt read\n";
            index_experiment = 0;
        } else {
            cout << "Files: "<< str_aux << endl;
            index_experiment = atoi(str_aux.c_str());
        }
        filefiles_i.close();

        filefiles_i_Exper.open((folderExper  + "/.files").c_str());
        string str_aux_Exper;
        filefiles_i_Exper >> str_aux_Exper;
        if (str_aux_Exper == "") {
            str_aux_Exper = "0";
            cout << "couldnt read\n";
        } else {
            cout << "Files: "<< str_aux_Exper << endl;
        }
        filefiles_i_Exper.close();
        if (atoi(str_aux.c_str()) >= atoi(str_aux_Exper.c_str())) {
            ROS_INFO("EXPERIMENTS DONE. PLEASE PRESS CTRL+C");
            while(ros::ok()) {
                ros::Duration(3600).sleep();
            }
            exit(0);
        }
        filefiles_o.open(folderfilescount.c_str());
        filefiles_o.clear();
        filefiles_o << (atoi(str_aux.c_str()) + 1);
        filefiles_o.close();
        cout << str_aux << endl;
        /*folderExper += (str_aux);
        folderExper +=  "/";*/







        rl_pkg::readExternalTab(folderExper+"QFinal"+str_aux+".txt", Q);
        std::unique_ptr<IMovementExplotation> mv(new MaxMovement(Q));
        this->master_handle = std::unique_ptr<MasterHandle>(
            new MasterHandle(Q,mv)
        );
    }

    void MeasureQ::runExperiments() {
        const int max_distance = this->physical_handle->getDistanceMax();
        std::vector<double> target = this->physical_handle->getNewTarget(max_distance);
        this->physical_handle->setTarget(target);
        ROS_INFO("[MeasureQ::runExperiments()] Total experiments: %d", this->total_experiments);
        ROS_INFO("[MeasureQ::runExperiments()] ROS_OK: %d", ros::ok());
        //rl_pkg::FileManager fm;
        std::vector<std::vector<double>> all_rewards;
        int count = 0;

        std::vector<double> rewards;
        std::fstream fs, fs_eps;
        fs.open(folderExper+"Explotation"+id_experiment_+".txt", fstream::out);
        fs_eps.open(folderExper+"ExplotationEndEpisodes"+id_experiment_+".txt", fstream::out);
        int step = 0, episode = 0, step_in_episode= 0;
        while(ros::ok() && count < this->total_experiments) {

            ROS_INFO("[MeasureQ::runExperiments] Index: %d, step %d, episode %d",index_experiment, step, episode);
            auto state_vector = this->physical_handle->lookState();
            int current_state = this->physical_handle->stateToInt(state_vector);
            int current_action = this->master_handle->actionSelection(current_state);
            if (current_action >= 0) {

                double needed_time = this->physical_handle->actionComand(current_action);
                //rewards.push_back(getReward(state_vector));
                fs << rl_pkg::limitNum(getReward(state_vector)) << " ";
                bool achieved = this->physical_handle->isTargetAchieved();
                if ( achieved || step_in_episode > 249) {
                    target = this->physical_handle->getNewTarget(max_distance);
                    this->physical_handle->setTarget(target);
                    fs_eps << step_in_episode << " " << (int) achieved << std::endl;
                    count++;
                    ROS_INFO("[MeasureQ::runExperiments] Target achieved.");
                    episode++;
                    step_in_episode = 0;
            //     all_rewards.push_back(rewards);
            //     rewards = std::vector<double>();
                } else {
                    step_in_episode++;
                }
                step++;
            } else {
                
                ros::Duration(0.01).sleep();
            }
        }
        fs.close();
        fs_eps.close();
        //rl_pkg::writeFilesMatrix(folderExper+"Explotation.txt", all_rewards);
    }
}
