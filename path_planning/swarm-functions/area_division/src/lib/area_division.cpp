#include "lib/area_division.h"


area_division::area_division ()
{
    // Initialize the ROS node handle and read optimization parameters from the parameter server.
    // Parameters include:
    // - max_iter: Maximum number of iterations (default 1000)
    // - variate_weight: Weight variation parameter (default 0.01)
    // - discr: Discrepancy parameter (default 30)
    NodeHandle nh;
    nh.param(this_node::getName() + "/optimizer/iterations", max_iter, 1000);
    nh.param(this_node::getName() + "/optimizer/variate_weight", variate_weight, 0.01);
    nh.param(this_node::getName() + "/optimizer/discrepancy", discr, 30);
}

void area_division::divide ()
{   

    // Perform area division for fair allocation of grid cells to robots:
    // - Initialize the necessary parameters and variables.
    // - Compute the distances of cells from the robots.
    // - Enter the main loop to attempt the division, adjusting the metrics based on connectivity and fairness.
    // - Adjust the discrepancy threshold and retry if the division is not successful within max iterations.

    // initializations
    int NoTiles = rows*cols;
    double fairDivision = 1.0 / nr;
    int effectiveSize = NoTiles - nr - ob;
    int termThr;
    if (effectiveSize % nr !=0) {
        termThr=1;
    }
    else {
        termThr=0;
    }
    ROS_DEBUG("%d free cells.", effectiveSize);
    // initialize distances of cells to cps
    vector<valarray<double>> AllDistances(nr, valarray<double>(rows*cols));
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols;j++) {
            for (int r=0; r<nr; r++) {
                AllDistances[r][i*cols+j] = hypot(cps[r][1]-i, cps[r][0]-j);
            }
        }
    }
    ROS_DEBUG("Computed distances from CPSs to all cells.");

    vector<valarray<double>> MetricMatrix = AllDistances;

    // perform area division
    success = false;
    while (termThr<=discr && !success) {
        ROS_DEBUG("Try division with discrepancy %d<=%d.", termThr, discr);
        // initializations
        double downThres = ((double)NoTiles-(double)termThr*(nr-1)) / (double)(NoTiles*nr);
        double upperThres = ((double)NoTiles+termThr) / (double)(NoTiles*nr);
        success = true;
        // main optimization loop
        int iter = 0;
        while (iter <= max_iter) {
            assign(MetricMatrix);
            // find connected areas
            vector<valarray<float>> ConnectedMultiplierList(nr);
            double plainErrors[nr];
            double divFairError[nr];
            for (int r=0; r<nr; r++) {
                valarray<float> ConnectedMultiplier(1, rows*cols);
                regions[r] = true;
                connected_components cc(BWlist[r], rows, cols, true);
                valarray<int> Ilabel = cc.compactLabeling();
                // at least one unconnected regions among r-robot's regions is found
                if (cc.getMaxLabel() > 1) {
                    regions[r] = false;

                    // find robot's sub-region and construct robot and non-robot binary regions
                    cc.constructBinaryImages(Ilabel[cps[r][1] * cols + cps[r][0]]);

                    // construct the final connected component multiplier
                    ConnectedMultiplier = CalcConnectedMultiplier(cc.NormalizedEuclideanDistanceBinary(true), cc.NormalizedEuclideanDistanceBinary(false));
                }
                ConnectedMultiplierList[r] = ConnectedMultiplier;

                // calculate the deviation from the the optimal assignment
                plainErrors[r] = ArrayOfElements[r] / (double) effectiveSize;
                if (plainErrors[r] < downThres) {
                    divFairError[r] = downThres - plainErrors[r];
                }
                else if (plainErrors[r] > upperThres) {
                    divFairError[r] = upperThres - plainErrors[r];
                }
            }

            // exit conditions
            if (isThisAGoalState(termThr)) {
                break;
            }

            // check fairness among different partitions
            double TotalNegPerc = 0.0, totalNegPlainErrors = 0.0;
            double correctionMult[nr];
            for (int r=0; r<nr; r++) {
                if (divFairError[r] < 0) {
                    TotalNegPerc += abs(divFairError[r]);
                    totalNegPlainErrors += plainErrors[r];
                }
                correctionMult[r] = 1.0;
            }

            // restore fairness
            for (int r=0; r<nr; r++) {
                if (totalNegPlainErrors != 0.0) {
                    if (divFairError[r]<0.0) {
                        correctionMult[r] = 1.0 + (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                    }
                    else {
                        correctionMult[r] = 1.0 - (plainErrors[r] / totalNegPlainErrors) * (TotalNegPerc / 2.0);
                    }
                }
                MetricMatrix[r] = FinalUpdateOnMetricMatrix(correctionMult[r], MetricMatrix[r], ConnectedMultiplierList[r]);
            }

            iter++;
        }

        // could not find area division
        if (iter >= max_iter) {
            max_iter = max_iter/2;
            success = false;

            ROS_DEBUG("Failed to divide in %d iterations.", iter);

            // increase allowed area discrepancy
            termThr++;
        }

        else
            ROS_DEBUG("Succeeded division with discrepancy %d after %d < %d iterations", termThr, iter, max_iter);
    }

    if (success == false)
        ROS_ERROR("Area division failed!");
}

nav_msgs::OccupancyGrid area_division::get_grid (nav_msgs::OccupancyGrid map, string cps)
{
    // Create a new grid map with assigned cells:
    // - Copy the input map to the assigned map.
    // - Iterate through each cell, marking it as free if assigned to the specified cps,
    //   and as occupied if assigned to other cpss.
    // - Update the header timestamps for the new grid map.

    // create new grid map
    nav_msgs::OccupancyGrid assigned;
    assigned = map;

    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols;j++) {
            // mark assigned cells as free
            if (A[i*cols+j] == uuid_map[cps]) {
                assigned.data[i*cols+j] = CELL_FREE;
            }

            // mark cells assigned to other cpss as obstacles
            else {
                assigned.data[i*cols+j] = CELL_OCCUPIED;
            }
        }
    }

    assigned.header.stamp = Time::now();
    assigned.info.map_load_time == Time::now();

    return assigned;
}

void area_division::initialize_cps (std::map<std::string, std::vector<int>> cpss)
{
    // Initialize cps's from the given map:
    // - Reset relevant variables and data structures.
    // - Iterate through the provided cps map, placing each cps into the grid and internal data structures.
    // - Count the total number of cps.

    // initialize
    nr = 0;
    cps.clear();
    uuid_map.clear();
    A.resize(rows*cols);
    regions.clear();
    regions.resize(cpss.size());

    // place cpss in the map
    for (auto c : cpss) {
        // divison algorithm assumes origin at top left
        int x = c.second[0];
        int y = c.second[1];

        // index of position in grid map
        int idx = y * cols + x;

        ROS_DEBUG("CPS %d (%s) at (%d,%d) index %d", nr, c.first.c_str(), x, y, idx);

        // if (gridmap[idx] == 127) {
        //     ROS_WARN("Skipping CPS %s at (%d, %d) due to unexplored cell", c.first.c_str(), x, y);
        //     continue;
        // }

        // place cps in data structures
        //gridmap[idx] = numeric_limits<signed char>::max();
        A[idx] = nr;

        // store cps position and mapping of uuid
        cps.push_back(initializer_list<int>{x,y});
        uuid_map[c.first] = nr;

        // count number of cpss
        nr++;
    }
}

void area_division::initialize_map (int r, int c, vector<signed char> src)
{
    // Initialize the map with given rows, columns, and source grid data:
    // - Set the number of rows and columns.
    // - Copy the source grid data to the internal gridmap.
    // - Count the number of occupied cells (grid cells with a value >= 40).

    // initialization
    rows = r;
    cols = c;
    gridmap = src;
    ob = 0;

    // count number of occupied cells
    for (int i=0; i<gridmap.size(); ++i)
        if (gridmap[i] >= 40)                  //TUNABLE
            ++ob;
        // else if (gridmap[i]==-1)
        // {
        //     gridmap[i]=127;
        //     ++ob;}
        // else
        //     gridmap[i]=0;

    ROS_DEBUG("There are %d occupied cells.", ob);
}

void area_division::assign (vector<valarray<double>> matrix)
{
    // Assign free grid cells to robots based on a metric matrix:
    // - Initialize the BWlist for each robot and set the initial positions.
    // - Iterate over each grid cell, assigning it to the robot with the lowest metric value.
    // - Mark obstacle cells appropriately
    BWlist.resize(nr);
    for (int r=0; r<nr; r++) {
        BWlist[r].resize(rows*cols);
        BWlist[r][cps[r][1] * cols + cps[r][0]] = 1;
    }
    ArrayOfElements.clear();
    ArrayOfElements.resize(nr);
    for (int i=0; i<rows; i++) {
        for (int j=0; j<cols; j++) {
            int idx=i*cols+j;
            // free grid cell, assign to a robot
            if (gridmap[idx] < 40) {   //TUNABLE
                // find index of robot that has lowest metric value
                double minV = matrix[0][idx];
                int indMin = 0;
                for (int r=1; r<nr; r++) {
                    if (matrix[r][idx] <= minV) {
                        minV = matrix[r][idx];
                        indMin = r;
                    }
                }
                // store assignment
                A[idx] = indMin;
                BWlist[indMin][idx] = 1;
                ArrayOfElements[indMin]++;
            }
            

            // obstacle
            else {
                A[idx] = nr;
            }
        }
    }
}

valarray<float> area_division::CalcConnectedMultiplier(valarray<float> dist1, valarray<float> dist2)
{
    // Calculate the connected multiplier matrix based on the difference between two distance matrices:
    // - dist1 and dist2: Input distance matrices.
    // - returnM: The resulting multiplier matrix.
    // - MaxV and MinV: Variables to track the maximum and minimum values in the difference matrix.
    // Normalize the difference matrix to a specified range using variate_weight.
    valarray<float> returnM(rows*cols);
    float MaxV = 0;
    float MinV = numeric_limits<float>::max();
    for (int i=0;i<rows;i++){
        for (int j=0;j<cols;j++){
            returnM[i*cols+j] = dist1[i*cols+j] - dist2[i*cols+j];
            if (MaxV < returnM[i*cols+j]) {MaxV = returnM[i*cols+j];}
            if (MinV > returnM[i*cols+j]) {MinV = returnM[i*cols+j];}
        }
    }

    for (int i=0;i<rows;i++){
        for (int j=0;j<cols;j++){
            returnM[i*cols+j] =(returnM[i*cols+j] - MinV)*((2*(float)variate_weight)/(MaxV-MinV))+(1-(float)variate_weight);
        }
    }

    return  returnM;
}

valarray<double> area_division::FinalUpdateOnMetricMatrix(double CM, valarray<double> curentONe, valarray<float> CC)
{
    // Update the metric matrix using the provided parameters
    // The updated metric matrix (MMnew) is calculated by element-wise multiplying 
    // curentONe with CM and CC.
    valarray<double> MMnew(rows*cols);

    for (int i=0; i<MMnew.size(); ++i) {
        MMnew[i] = curentONe[i] * CM * CC[i];
    }

    return MMnew;
}


bool area_division::isThisAGoalState(int thres)
{
    // Check if the current state meets the goal criteria:
    // - Determine the maximum and minimum number of cells assigned to any region.
    // - Verify that all regions are assigned.
    // - Return true if the difference between max and min assigned cells is within the given threshold.

    int maxCellsAss = 0;
    int minCellsAss = numeric_limits<int>::max();


    for (int r=0; r<nr; r++) {
        if (maxCellsAss < ArrayOfElements[r]) {
            maxCellsAss = ArrayOfElements[r];
        }
        if (minCellsAss > ArrayOfElements[r]) {
            minCellsAss = ArrayOfElements[r];
        }

        if (!regions[r]) {
            return false;
        }
    }

    return (maxCellsAss - minCellsAss) <= thres;
}