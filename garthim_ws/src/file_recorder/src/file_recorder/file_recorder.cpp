#include <file_recorder/file_recorder.h>

namespace file_recorder {
    FileRecorder::FileRecorder(bool write) {
        ros::NodeHandle nh_global, nh("~");
        nh_global.param("decrement_collision_reward_each_step", decrement_each_exp_, 1);
        write_read_file_ = nh.hasParam("url_initial_value_function") && nh.hasParam("write_file_read");
        setEnvironment(write);
        filename_ += nh.param("filename", std::string("RecordTab")) + iteration_ + ".txt";
        file_episodes_ += nh.param("filename_end", std::string("EpisodeCompletation")) + iteration_ + ".txt";
        if (nh.hasParam("filename_scaffolding")) {
            filename_scaffolding_ += nh.param("filename_scaffolding", std::string("Scaffolding")) + iteration_ + ".txt";
            
        }
        if (nh.hasParam("filename_Q_scaffolding")) {
            filename_Q_scaffolding_ += nh.param("filename_Q_scaffolding", std::string("QScaf")) + iteration_ + ".txt";
        }
        steps_ = 1;
        if (nh.hasParam("url_initial_value_function")) {
            if (write_read_file_) {
                int it_read = std::stoi(iteration_read_write_)/decrement_each_exp_;
                iteration_read_ = std::to_string(it_read);
            }
            url_initial_value_function_ += nh.param("url_initial_value_function", std::string("QIni")) + iteration_read_ + ".txt";
            ROS_INFO("%s", url_initial_value_function_.c_str());
        }
        if (nh.hasParam("url_transition_model")) {
            url_transition_model_ += nh.param("url_transition_model", std::string("Model")) + ".mat";
        }
        first_step_done_ = false;
    }
    FileRecorder::~FileRecorder(){}
    void FileRecorder::recordScaffold(const std_msgs::UInt64& episode) {
        std::vector<double> data = {(double) episode.data}; 
        // Possible inaccuracy because of a conflict with FileRecorder::record
        writeFiles(filename_scaffolding_, data);
    }
    void FileRecorder::record(const rl_msgs::TransitionResult& transition) {
        if (first_step_done_) {
            const std::vector<double> data = {
                (double)transition.state.content.as_integer[0],
                (double)transition.action.as_integer[0],
                (double)transition.next_state.content.as_integer[0],
                transition.reward,
                transition.time_elapsed,
                0
                };
            writeFiles(filename_,data);
            if (transition.next_state.type) {
                ROS_INFO("[FileRecorder::record] End of episode saved");
                const std::vector<double> data = {
                    (double) steps_,
                    (double) transition.next_state.type - 1
                };
                writeFiles(file_episodes_, data);
            }
            steps_++;
        }
        first_step_done_ = true;
    }
    void FileRecorder::readValueFunction(std::vector<std::vector<double>>& Q, int end_row, int repeat) {
        int begin_row = 0, inc = end_row;
        for (int i = 0; i < repeat; i++) {
            readExternalTab(url_initial_value_function_,begin_row, end_row,Q);
            begin_row = end_row;
            end_row += inc;
        }
    }
    void FileRecorder::readModel(std::vector<std::vector<std::vector<double>>>& M, int end_row) {
        /*
        int begin_row = 0, inc = end_row;
        ROS_INFO("[FileRecorder::readModel] Reading model from url: %s", url_transition_model_.c_str());
        MATFile* mf = matOpen(url_transition_model_.c_str(), "r");
        std::vector<double> v;
        if (mf != NULL) {
            mxArray *arr = matGetVariable(mf, "Visits");
            if (arr != NULL && mxIsDouble(arr) &&  !mxIsEmpty(arr)) {
                mwSize nums = mxGetNumberOfElements(arr);
                const mwSize *dims = mxGetDimensions(arr);
                double *pr = mxGetPr(arr);
                auto it = pr;
                for (int i = 0; i < *dims; i++) {
                    M.emplace_back();
                    for (int j =0; j < *(dims+1); j++) {
                        if (it != NULL) {
                            M.back().emplace_back();
                            M.back().back().reserve(*(dims+2));
                            // Comment
                            ROS_INFO("Copying vector");
                            std::stringstream ss("[");
                            for (auto it_aux = it; it_aux < it + *(dims+2); it_aux++) {
                                ss << *it_aux << " ";
                            }
                            ss << "]";
                            ROS_INFO("VECTOR: %s", ss.str().c_str());
                            // comment
                            M.back().back().assign(it, it + *(dims+2));
                            it += *(dims+2);
                        }
                    }
                }
                for (int j = 0; j < M[0].size(); j++) {
                    for (int i = 0; i < M.size(); i++) {
                        for (int k = i; k <M[i][j].size(); k++) {
                            double aux = M[i][j][k];
                            M[i][j][k] = M[k][j][i];
                            M[k][j][i] = aux;
                        }
                    }
                } 
                int state_example = M.size() > 700? 2279: 645;
                ROS_INFO("Copying vector FOR STATE %d, ACTION 2", state_example);
                std::stringstream ss;
                double sumi = 0;
                for (auto it = M[state_example][2].begin(); it < M[state_example][2].end(); it++) {
                    if ((*it) > 0) {
                        sumi+= *it;
                        ss << "state: " <<std::distance(M[state_example][2].begin(),it) <<" "<<*it << " " << std::endl;
                    }
                }
                ROS_INFO("VECTOR: %s", ss.str().c_str());
                ROS_INFO("SUM %f", sumi);
                if (pr != NULL) {
                    v.reserve(nums);
                    v.assign(pr, pr+nums);
                }
            }
            mxDestroyArray(arr);
            matClose(mf);


        } else {
            ROS_ERROR("Couldn't open or find %s file. ", url_transition_model_.c_str());
        }*/
        ROS_ERROR("[FileRecorder::readModel] Not implemented");
    }
    void FileRecorder::setEnvironment(bool write) {
        using namespace std;
        std::string rosmasteruri = getenv("ROS_MASTER_URI");
        rosmasteruri[5] =rosmasteruri[6] = '_';
        filename_ = "src/rl_missions/results/";
        filename_ += rosmasteruri + "/";
        file_episodes_ = "src/rl_missions/results/";
        file_episodes_ += rosmasteruri + "/";
        url_initial_value_function_ = "src/rl_missions/results/";
        url_transition_model_ = "src/rl_missions/results/";
        //url_initial_value_function_ += rosmasteruri + "/";
        filename_scaffolding_ = "src/rl_missions/results/";
        filename_scaffolding_ += rosmasteruri + "/";
        filename_Q_scaffolding_ = "src/rl_missions/results/";
        filename_Q_scaffolding_ += rosmasteruri + "/";
        url_folder_ = "src/rl_missions/results/";
        url_folder_ += rosmasteruri + "/";
        chdir("../../..");
        ifstream filefiles_i;
        ofstream filefiles_o;

        string folderfilescount = filename_ + ".file_to_read";
        filefiles_i.open(folderfilescount.c_str());
        if (write_read_file_) {
            ROS_ERROR("HEY");
            filefiles_i >> iteration_read_write_;
            if (iteration_read_write_ == "") {
                iteration_read_write_ = "0";
                cout << "couldnt read\n";
            } else {
                cout << "file_to_read: "<< iteration_read_write_ << endl;
            }
            filefiles_o.open(folderfilescount.c_str());
            filefiles_o.clear();
            filefiles_o << (atoi(iteration_read_write_.c_str()) + 1);
            filefiles_o.close();
            ROS_ERROR("[FileRecorder::setEnvironment] Lock folder removed.");
        } else {
            ROS_ERROR("NO HEY");
            filefiles_i >> iteration_read_;
            if (iteration_read_ == "") {
                iteration_read_ = "0";
                cout << "couldnt read\n";
            } else {
                cout << "file_to_read: "<< iteration_read_ << endl;
            }
        }
        filefiles_i.close();
        /*std::string lockFileName = filename_+".files.lock";
        struct stat sb;
        std::string url_rosmasterlockfile = lockFileName+'/'+rosmasteruri;
        ROS_INFO("Stat. %d", stat(url_rosmasterlockfile.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode));
        while (mkdir(lockFileName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) &&
            !(stat(url_rosmasterlockfile.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
            ROS_ERROR("[FileRecorder::setEnvironment] Lock folder exists: Waiting for Lock");
            ros::WallDuration(0.5).sleep();
        }
        ROS_ERROR("[FileRecorder::setEnvironment] Lock folder created, lock acquired");
        if (mkdir(url_rosmasterlockfile.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)) {
            ROS_ERROR("ERROR");
        }*/

        folderfilescount = filename_ + ".files";
        ROS_INFO("FOLDER %s", folderfilescount.c_str());
        filefiles_i.open(folderfilescount.c_str());
        filefiles_i >> iteration_;
        if (iteration_ == "") {
            iteration_ = "0";
            cout << "couldnt read\n";
        } else {
            cout << "Files: "<< iteration_ << endl;
        }
        filefiles_i.close();
        if (write) {
            filefiles_o.open(folderfilescount.c_str());
            filefiles_o.clear();
            filefiles_o << (atoi(iteration_.c_str()) + 1);
            filefiles_o.close();
            ROS_ERROR("[FileRecorder::setEnvironment] Lock folder removed.");
        }

        //rmdir(lockFileName.c_str());
        cout << iteration_ << endl;
        
    }
}