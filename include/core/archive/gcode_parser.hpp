#ifndef GCODE_PARSER_HPP  
#define GCODE_PARSER_HPP  

#include <Eigen/Dense>
#include <string>    
#include <optional> 

enum class MotionMode {  
    NONE, G0, G1, G2, G3  
};  

enum class UnitMode {  
    MM, INCH  
};  

enum class CoordMode {  
    ABSOLUTE, RELATIVE  
};  

enum class PlaneMode {  
    XY, ZX, YZ  
};  

struct GCodeCommand {
	UnitMode unit = UnitMode::MM;
	CoordMode coord = CoordMode::ABSOLUTE;
	PlaneMode plane = PlaneMode::ZX;
    MotionMode motion = MotionMode::NONE;
    Eigen::Vector3d startPoint = Eigen::Vector3d::Zero();
    std::optional<double> x;  
    std::optional<double> y;  
    std::optional<double> z;  
    std::optional<double> i, j, k;  
    std::optional<double> f, p;  
    int mcode = -1;  
    std::string originalLine;  
    int lineNumber = -1;  
};  

class GCodeParser {  
public:  
    GCodeParser();  

    std::vector<GCodeCommand> parseFile(const std::string& filepath);  

private:  
    UnitMode unitMode;  
    CoordMode coordMode;  
    PlaneMode planeMode;  
    MotionMode motionMode;  

    double lastX, lastY, lastZ;  

    GCodeCommand parseLine(const std::string& line, int lineNumber);  
    double convertToMM(double value) const;  
};  
void printGCodeCommand(const GCodeCommand& cmd);
#endif // GCODE_PARSER_HPP
