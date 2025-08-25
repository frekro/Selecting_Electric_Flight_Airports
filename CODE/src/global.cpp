#include "global.h"
#include <fstream>

boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
date todaysDateInLocalTZ = day_clock::local_day();
std::ostringstream date_namebuf(static_cast<std::ostringstream &&>(std::ostringstream() << todaysDateInLocalTZ.day()
                                                                                        << "_" << todaysDateInLocalTZ.month()
                                                                                        << "_" << todaysDateInLocalTZ.year()
                                                                                        << "_at_" << now.time_of_day().hours()
                                                                                        << "_" << now.time_of_day().minutes()
                                                                                        << "_" << now.time_of_day().seconds()));

string output_path;
ofstream pFilepriced;
ofstream pBranchDec;
ofstream pPenaltyCuts;
ofstream input_data;

void prepare_output(string outputgroupname)
{

    boost::filesystem::path dir_zero(outputgroupname);
    if (!(boost::filesystem::exists(dir_zero)))
    {
        if (!boost::filesystem::create_directory(dir_zero))
        {
            std::cout << "folder output not created!\n";
        }
    }

    boost::filesystem::path dir_one = dir_zero / date_namebuf.str();
    int cnt = 1;
    while (boost::filesystem::exists(dir_one))
    {
        dir_one = dir_zero / (date_namebuf.str() + "_(" + std::to_string(cnt++) + ")");
    }

    if (!boost::filesystem::create_directory(dir_one))
    {
        std::cout << "folder " + date_namebuf.str() + " not created!\n";
    }

    boost::filesystem::path dir_three = dir_one / "scip_data";
    if (!boost::filesystem::create_directory(dir_three))
    {
        std::cout << "folder " + date_namebuf.str() + "/scip_data/ not created!\n";
    }

    output_path = dir_one.string();
    pFilepriced.open(output_path + std::string("/scip_data/priced_variables.txt").c_str(), ios::app);
    pFilepriced << "Variable\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t|\tValue\n";
    pFilepriced.flush();
    pBranchDec.open(output_path + std::string("/scip_data/branching_decisions.txt").c_str(), ios::app);
    pBranchDec << "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t|\tBrachningDecision\n";
    pBranchDec.flush();
    input_data.open(output_path + std::string("/input_data.txt").c_str(), ios::app);
    input_data << "Instance Data\n";
    input_data.flush();
    pPenaltyCuts.open(output_path + std::string("/scip_data/penalty_cuts.txt").c_str(), ios::app);
    pPenaltyCuts << "Penalty Cuts:\n";
    pPenaltyCuts.flush();
}
