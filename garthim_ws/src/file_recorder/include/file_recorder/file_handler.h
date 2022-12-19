#ifndef __FILE_RECORDER_FILE_HANDLER_H__
#define __FILE_RECORDER_FILE_HANDLER_H__
#include<string>
#include<vector>
#include<map>
#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
namespace file_recorder {
    /**
     * @brief Method used to load in out parameter Q a value function.
     * 
     * @param urlQStartFile Url of file containing value-function.
     * @param br Position indicating the start of a new row.
     * @param er Position indicating the end of a row.
     * @param Q Out parameter containing value-function in tabular form.
     */
    void readExternalTab(const std::string& urlQStartFile, int br, int er,
        std::vector< std::vector<double> >& Q);
    /**
     * @brief Method used to load in out parameter a transition-model.
     * 
     * @param urlQStartFile Url of file containing transition-model.
     * @param br Position indicating the start of a new row.
     * @param er Position indicating the end of a row.
     * @param Q Out parameter containing value-function in tabular form.
     */
    void readExternalTab(const std::string& urlModelFile, int br, int er, int br2,
        std::vector< std::vector< std::vector<double> > >& M);
    /**
     * @brief A function that lowers a number if it can't be written in a stringstream.
     * 
     * @param num Number to be analized.
     * @return double. Reduced number.
     */
    double limitNum(double num);
    /**
     * @brief A method that writes a vector in a file as a row or as a column.
     * 
     * @param urlQStartFile Name of the file for storing values.
     * @param values Vector containing containing the numbers to be written. (double)
     * @param is_column (default false) wheter vector is written as a column or as a row.
     */
    void writeFiles(const std::string& urlQStartFile, const std::vector<double>& values,bool is_column = false);
    /**
     * @brief A method that writes a matrix in a file and may put an end line parameter at the end.
     * 
     * @param urlQStartFile Name of the file to write.
     * @param values Matrix containing the numbers to be written. (double)
     * @param endl_at_end (default false) whether an end line parameter is put at the end of the written stream.
     */
    void writeFilesMatrix(const std::string& urlQStartFile, const std::vector< std::vector<double> >& values, const bool endl_at_end=false);
}
#endif