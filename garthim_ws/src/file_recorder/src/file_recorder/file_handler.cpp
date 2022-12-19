
#include <file_recorder/file_handler.h>

namespace file_recorder {

    void writeFiles(const std::string& file_name,
        const std::vector<double>& values, const bool is_column){
        std::ofstream stream(file_name.c_str(), std::ios_base::app);
        std::ostringstream ss;
        for (auto it = values.begin(); it != values.end(); ++it) {
            ss << *it;
            stream << (ss.str().size() >= 10 ? limitNum(*it) : *it) << " ";
            if (is_column) {
                stream << std::endl;
            }
        }
        if (!is_column) {
            stream << std::endl;
        }
        stream.close();
    }
    void writeFilesMatrix(const std::string& file_name,
     const std::vector< std::vector<double> >& Q, const bool endl_at_end) {
        for (auto it = Q.begin(); it != Q.end(); ++it) {
            writeFiles(file_name, *it);
        }
        if (endl_at_end) {
            std::ofstream stream(file_name.c_str(), std::ios_base::app);
            stream << std::endl;
            stream.close();
        }
     }
    void readExternalTab(const std::string& urlQStartFile, int br, int er,
        std::vector< std::vector<double> >& Q){
        //Método que inicializa la tabla Q desde un archivo externo llamado QIni.txt
        std::ifstream tabQ;

        tabQ.open(urlQStartFile);
        if(!tabQ) {
            std::cout << "\n Error abriendo la tabla Q inicializadora: " << urlQStartFile << "\n";
        }

        double aux =0;
        for(size_t i=br;i < er;i++) {
            for(size_t j=0;j<Q[i].size();j++) {
                tabQ >> aux;
                Q.at(i).at(j) = aux;
            }
        }
        tabQ.close();
    }

    void readExternalTab(const std::string& urlModelFile, int br, int er, int br2,
        std::vector< std::vector< std::vector<double> > >& M) {

        std::ifstream tabM;

        tabM.open(urlModelFile);
        if (!tabM) {
            std::cout << "\n Error abriendo la tabla M inicializadora: " << urlModelFile << "\n";
        }
        double aux = 0;
        for (size_t i = br; i < er; i++) {
            for (size_t j = 0; j < M[i].size(); i++) {
                for (size_t k = 0; k < M[i][j].size(); k++) {
                    tabM >> aux;
                    M[i][j][k] = aux;
                }
            }
        }
        tabM.close();
    }

    double limitNum(double num) {
        //método que limita el número de caracteres a 10

        std::ostringstream ssCheck;
        ssCheck.clear();
        ssCheck << num;
        double val; //valor que se devolverá
        if (ssCheck.str().size() >= 10) {


        std::ostringstream ss;
        int integer = round(num); //parte entera 
        ss << integer;
        int integerNum = ss.str().size(); //nuemro de caracteres de parte entera
        int decimalNum = 9-integerNum; //numero de caracteres que se van a permitir en parte decimal

        //si el númeor ya es mayor de 10 en parte entera, truncamos
        if(integerNum >= 10) {
            // si el bit más significativo que quedaría es 0, lo ponemos a 9
            if(ss.str().at(integerNum-10)=='0') {
                ss.str().at(integerNum-10)='9';
            }
            //numero truncado
            val = atof(ss.str().substr(integerNum-10,integerNum).c_str());
        }
        else {
            //redondeamos a parte decimal permitida
            val = round(num*pow(10,decimalNum))/pow(10,decimalNum);
        }
        } else {
            val = num;
        }
        return val;
    }
}