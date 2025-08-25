#include <boost/date_time/posix_time/posix_time.hpp>
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/filesystem.hpp"

using namespace std;
using namespace boost::gregorian;

extern boost::posix_time::ptime now;
extern date todaysDateInLocalTZ;
extern std::ostringstream date_namebuf;
extern string output_path;
extern ofstream pFilepriced;
extern ofstream pBranchDec;
extern ofstream input_data;
extern ofstream pPenaltyCuts;

void prepare_output(string outputgroupname);