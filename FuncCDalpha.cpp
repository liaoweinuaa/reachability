#include "FuncCDalpha.h"
#include <algorithm>
using namespace std;


static double alphalist[11] = {
    -0.419,
    -0.279,
    -0.14,
    -0.07,
    0,
    0.07,
    0.14,
    0.279,
    0.419,
    0.559,
    0.698
};

static double CDalphalist[11] = {
    0.38,
    0.22,
    0.07,
    0.04,
    0.0147,
    0.04,
    0.07,
    0.22,
    0.38,
    0.8,
    1.01
};

double Func_CDalpha(double alpha)
{
    int index_upper = upper_bound(alphalist, alphalist + 6, alpha) - alphalist;
    int index_lower = index_upper - 1;

    double alpha_upper = alphalist[index_upper];
    double alpha_lower = alphalist[index_lower];
    double deltaalpha = alpha_upper - alpha_lower;
    return ((alpha_upper - alpha) * CDalphalist[index_lower] + (alpha - alpha_lower) * CDalphalist[index_upper])/deltaalpha;
}


