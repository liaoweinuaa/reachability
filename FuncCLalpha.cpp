#include "FuncCLalpha.h"
#include <algorithm>
using namespace std;







static double alphalist[16] = {
-0.349,
- 0.262,
- 0.175,
- 0.087,
0,
0.07,
0.14,
0.209,
0.279,
0.349,
0.419,
0.489,
0.559,
0.698,
0.785,
0.873
};

static double CLalphalist[16] = {
-0.93,
- 0.86,
- 0.58,
- 0.26,
0.06,
0.3,
0.58,
0.78,
0.88,
1.01,
1.08,
1.16,
1.19,
1.17,
1.09,
1.05

};

double Func_CLalpha(double alpha)
{
    int index_upper = upper_bound(alphalist, alphalist + 6, alpha) - alphalist;
    int index_lower = index_upper - 1;

    double alpha_upper = alphalist[index_upper];
    double alpha_lower = alphalist[index_lower];
    double deltaalpha = alpha_upper - alpha_lower;
    return ((alpha_upper - alpha) * CLalphalist[index_lower] + (alpha - alpha_lower) * CLalphalist[index_upper]) / deltaalpha;
}


