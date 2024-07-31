#include <vector>


class PathGenerator {
public:
    //calculates paths based on some physical road parameters (overtaken object size, for instance)
    //unclear at this point
    void CalculatePaths();

    
    std::vector<CandidatePath> GetPaths(){ return paths};
private:
    std::vector<CandidatePath> paths;
};

class CandidatePath{
public:
    //sample path at points given by samples
    std::vector<Point2f> operator()(std::vector<float> samples) const;
private:
    //some path variables that define it, like controls for B spline
};