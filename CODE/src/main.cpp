
#include <string>
#include <iostream>
#include "seaport_program/scip_instance/model/scip_exception.h"
#include "global.h"
#include "global_parameter.h"
#include "seaport_program/seaport_class.h"

#include <boost/program_options.hpp>

// #include "dump.h"

using namespace std;
using namespace boost;

int main(int argc, const char *argv[])
{

    std::filesystem::path file = __FILE__;
    std::string path = file.parent_path().parent_path().parent_path().string();

    Input_AirportGraph input{};
    input.path_dataset = path + "/DATA";
    Input_SeaportProgram input_seaport = {};

    boost::program_options::options_description desc{"Options"};
    desc.add_options()("outputfouldername", boost::program_options::value<string>(&input.outputfouldername)->default_value(""), "the outputfouldername");
    desc.add_options()("overall_budget", boost::program_options::value<double>(&input_seaport.budget)->default_value(100000000), "the overall budget limit");
    desc.add_options()("elec_range", boost::program_options::value<double>(&input.elec_range)->default_value(300), "the considered range of the electric aircraft");
    desc.add_options()("name_dataset", boost::program_options::value<string>(&input.dataset_name)->default_value("german_5_1"), "the considered dataset");
    desc.add_options()("eleccost_dataset", boost::program_options::value<string>(&input.eleccost_dataset)->default_value("electrification_costs_szenario1"), "the considered elec costs");
    desc.add_options()("electricity_price", boost::program_options::value<double>(&input.electricity_price)->default_value(0.13), "the considered price for electricity");
    desc.add_options()("fuel_price", boost::program_options::value<double>(&input.fuel_price)->default_value(0.672), "the considered price for fuel");
    boost::program_options::variables_map vm;
    boost::program_options::positional_options_description positional_options;
    positional_options.add("outputfouldername", 1);
    positional_options.add("elec_range", 1);
    positional_options.add("overall_budget", 1);
    positional_options.add("name_dataset", 1);
    positional_options.add("eleccost_dataset", 1);
    positional_options.add("fuel_price", 1);
    positional_options.add("electricity_price", 1);
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                      .options(desc)
                                      .positional(positional_options)
                                      .run(),
                                  vm);
    if (vm.count("help"))
    {
        std::cout << "Usage: "
                  << argv[0]
                  << " [options] <input> <output>"
                  << std::endl;

        std::cout << desc << std::endl;
    }
    boost::program_options::notify(vm);

    input.electricity_price = 0.13; // (price pro kWh)
    input.fuel_price = 0.67;        // (price pro Kg)

    AirportGraph *airportgraph = new AirportGraph(input);
    input_seaport.airportgraph = airportgraph;
    SeaportProgram seaport_program{input_seaport};

    prepare_output(path + "/OUTPUT" + input.outputfouldername);

    input_data << "dataset_name: " << input.dataset_name << std::endl;
    input_data << "aircraft_range: " << input.elec_range << std::endl;
    input_data << "overall_budget: " << input_seaport.budget << std::endl;
    input_data << "electricity_price: " << input.electricity_price << std::endl;
    input_data << "fuel_price: " << input.fuel_price << std::endl;
    input_data.flush();

    FILE *pFile_primal = fopen((output_path + std::string("/model.lp")).c_str(), "w");
    SCIPprintOrigProblem(seaport_program.model, pFile_primal, "lp", false);
    fclose(pFile_primal);

    SCIPsetMessagehdlrLogfile(seaport_program.model, (output_path + "/scip_data/log.csv").c_str());

    // solve Model :
    SCIP_CALL_EXC(SCIPsolve(seaport_program.model));

    FILE *pFile_sol = fopen((output_path + std::string("/solution.txt")).c_str(), "w");
    SCIPprintBestSol(seaport_program.model, pFile_sol, true);
    fclose(pFile_sol);

    FILE *pFile_stats = fopen((output_path + std::string("/statistics.txt")).c_str(), "w");
    SCIPprintStatistics(seaport_program.model, pFile_stats);
    fclose(pFile_stats);

    FILE *pFile_pricer = fopen((output_path + std::string("/pricer_stats.txt")).c_str(), "w");
    SCIPprintPricerStatistics(seaport_program.model, pFile_pricer);
    fclose(pFile_pricer);

    FILE *pFile_branch = fopen((output_path + std::string("/branching_stats.txt")).c_str(), "w");
    SCIPprintBranchingStatistics(seaport_program.model, pFile_branch);
    fclose(pFile_branch);

    return 0;
}
