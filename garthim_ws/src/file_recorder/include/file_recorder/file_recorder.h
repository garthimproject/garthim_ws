#ifndef __FILE_RECORDER_FILE_RECORDER_H__
#define __FILE_RECORDER_FILE_RECORDER_H__
#include <ros/ros.h>
#include <rl_msgs/TransitionResult.h>
#include <file_recorder/file_handler.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include <sys/stat.h>
//#include <mat.h>
namespace file_recorder {
    /**
     * @brief A wrapper object used in a rosnode to execute read/write operations.
     * 
     */
    class FileRecorder {
        private:
            bool write_read_file_;
            int decrement_each_exp_;
            std::string url_transition_model_;
            /**
             * @brief An url of the folder used to store files.
             * 
             */
            std::string url_folder_;
            /**
             * @brief An url of the filename to store values. Ex: Each transition result.
             * 
             */
            std::string filename_;
            /**
             * @brief Filename of the info obtained at the end of each episode.
             * 
             */
            std::string file_episodes_;
            /**
             * @brief String containing an index to append to filenames used for writing.
             * 
             */
            std::string iteration_;
            /**
             * @brief String containing an index to append to filenames used for reading.
             * 
             */
            std::string iteration_read_, iteration_read_write_;
            /**
             * @brief Filename of initial value function.
             * 
             */
            std::string url_initial_value_function_;
            /**
             * @brief Filename for storing scaffolding steps.
             * 
             */
            std::string filename_scaffolding_;

            /**
             * @brief Filename for storing value-function when scaffolding level is risen.
             * 
             */
            std::string filename_Q_scaffolding_;
            /**
             * @brief Number of times a value is written. i.e. a step is performed by the agent.
             * 
             */
            uint64_t steps_;
            /**
             * @brief Flag indicating if first step has been skipped.
             * 
             */
            bool first_step_done_;
            /**
             * @brief Private function used when setting the folder where values are going to be written.
             * 
             * @param write A flag indicating whether writting operations are performed at all. This 
             * method does not enables or restrict the possibility of writting in a file, rather, it
             * must be called with the correct flag if there is files that are going to be written using 
             * this object or not.
             */
            void setEnvironment(bool write);
        public:
            FileRecorder(bool write);
            ~FileRecorder();
            /**
             * @brief A method that writes a rl_msgs::TransitionResult message in a file. Used as a callback.
             * 
             * @param transition message to be written.
             */
            void record(const rl_msgs::TransitionResult& transition);
            /**
             * @brief A method that writes the step when scaffolding is risen.Used as a callback.
             * 
             */
            void recordScaffold(const std_msgs::UInt64&);
            /**
             * @brief A method to call when value-function is written at the end of execution.
             * 
             * @param Q Value-function in tabular form in order to be written.
             */
            inline void saveValueFunctionEnd(const std::vector<std::vector<double>>& Q) {
                writeFilesMatrix(file_episodes_, Q);
            }
            /**
             * @brief A method to call when value-function is written during execution.
             * 
             * @param Q Value-function in tabular form in order to be written.
             * @param last_endl A flag indicating whether an endl must be written at the end of the stream.
             */
            inline void saveValueFunction(const std::vector<std::vector<double>>& Q, const bool last_endl=false) {
                writeFilesMatrix(filename_, Q, last_endl);
            }
            /**
             * @brief A method to write a collection of numeric values in a file.
             * 
             * @param name_file Name of the file.
             * @param values Values in a double vector form
             * @param is_column Whether values are treated as columns or rows.
             */
            inline void saveGenericVector(std::string name_file,const std::vector<double>& values, bool is_column = false) {
                std::string url = url_folder_ + name_file + iteration_ + ".txt";
                writeFiles(url, values, is_column);
            }
            /**
             * @brief A method to write value-function when scaffolding is performed.
             * 
             * @param Q Value function in tabular form.
             * @param last_endl Whether to put an endl at the end of the file.
             */
            inline void saveValueFunctionScaffolding(const std::vector<std::vector<double>>& Q, const bool last_endl=false) {
                writeFilesMatrix(filename_Q_scaffolding_, Q, last_endl);
            }
            /**
             * @brief A wrapped method to read a value function from inner parameter.
             * 
             * @param Q out parameter to store value-function.
             */
            inline void readValueFunction(std::vector<std::vector<double>>& Q) {
                readExternalTab(url_initial_value_function_,0, Q.size(),Q);
            }
            /**
             * @brief A method to read a value function and to create a new one based on repetitions of the previously written one.
             * 
             * @param Q out parameter to store value-function.
             * @param end_row position of end row to be read.
             * @param repeat number of times the read value-function is repeated and stored inside Q.
             */
            void readValueFunction(std::vector<std::vector<double>>& Q, int end_row, int repeat);
            void readModel(std::vector<std::vector<std::vector<double>>>& M, int end_row);
    };
}

#endif