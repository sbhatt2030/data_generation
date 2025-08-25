#include "core/gcode_parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cctype>


GCodeParser::GCodeParser(): 
    motionMode(MotionMode::NONE),
    unitMode(UnitMode::MM),
    coordMode(CoordMode::ABSOLUTE),
    planeMode(PlaneMode::XY),
    lastX(0.0), lastY(0.0), lastZ(0.0) {
}

std::vector<GCodeCommand> GCodeParser::parseFile(const std::string& filepath) {
    std::vector<GCodeCommand> commands;
    std::ifstream file(filepath);
    std::string line;
    int lineNumber = 0;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filepath << std::endl;
        return commands;
    }

    while (std::getline(file, line)) {
        if (!line.empty()) {
            GCodeCommand cmd = parseLine(line, lineNumber);
            if (cmd.motion != MotionMode::NONE || cmd.mcode != -1) {
                commands.push_back(cmd);
            }
        }
        ++lineNumber;
    }
    return commands;
}

GCodeCommand GCodeParser::parseLine(const std::string& line, int lineNumber) {
    GCodeCommand cmd;
    cmd.originalLine = line;
    cmd.lineNumber = lineNumber;

    std::istringstream ss(line);
    std::string word;

    while (ss >> word) {
        char prefix = std::toupper(word[0]);
        double value = std::stod(word.substr(1));

        switch (prefix) {
        case 'G': {
            int gcode = static_cast<int>(value);
            switch (gcode) {
            case 0: cmd.motion = MotionMode::G0; break;
            case 1: cmd.motion = MotionMode::G1; break;
            case 2: cmd.motion = MotionMode::G2; break;
            case 3: cmd.motion = MotionMode::G3; break;
            case 4: cmd.p = value; break; // dwell
            case 17: planeMode = PlaneMode::XY; break;
            case 18: planeMode = PlaneMode::ZX; break;
            case 19: planeMode = PlaneMode::YZ; break;
            case 20: unitMode = UnitMode::INCH; break;
            case 21: unitMode = UnitMode::MM; break;
            case 90: coordMode = CoordMode::ABSOLUTE; break;
            case 91: coordMode = CoordMode::RELATIVE; break;
            default: break;
            }
            break;
        }
        case 'M': cmd.mcode = static_cast<int>(value); break;
        case 'X':
            if (coordMode == CoordMode::ABSOLUTE && cmd.motion != MotionMode::G0 ) {
                cmd.x = convertToMM(value);
            }
            else if(coordMode == CoordMode::RELATIVE && cmd.motion != MotionMode::G0) {
                
                cmd.x = lastX + convertToMM(value);
            }
            else if (coordMode == CoordMode::ABSOLUTE && cmd.motion == MotionMode::G0){
				lastX = convertToMM(value);

            }
            else {
                lastX += convertToMM(value);
            }
                break;
        case 'Y': 
            if (coordMode == CoordMode::ABSOLUTE && cmd.motion != MotionMode::G0) {
                cmd.y = convertToMM(value);
            }
            else if(coordMode == CoordMode::RELATIVE && cmd.motion != MotionMode::G0) {
                cmd.y = lastY + convertToMM(value);
            }
			else if (coordMode == CoordMode::ABSOLUTE && cmd.motion == MotionMode::G0) {
				lastY = convertToMM(value);
                }
            else {
                lastY += convertToMM(value);
			} break;
        case 'Z': 
            if (coordMode == CoordMode::ABSOLUTE && cmd.motion != MotionMode::G0) {
                cmd.z = convertToMM(value);
            }
            else if(coordMode == CoordMode::RELATIVE && cmd.motion != MotionMode::G0) {
                cmd.z = lastZ + convertToMM(value);
			}
			else if (coordMode == CoordMode::ABSOLUTE && cmd.motion == MotionMode::G0) {
				lastZ = convertToMM(value);
                }
            else {
				lastZ += convertToMM(value);
			} break;
        case 'I': cmd.i = convertToMM(value); break;
        case 'J': cmd.j = convertToMM(value); break;
        case 'K': cmd.k = convertToMM(value); break;
        case 'F': cmd.f = convertToMM(value); break;
        case 'P': cmd.p = value; break;
        default: break;
        }
        cmd.unit = unitMode;
        cmd.coord = coordMode;
        cmd.plane = planeMode;
        cmd.startPoint(0) = lastX;
		cmd.startPoint(1) = lastY;
		cmd.startPoint(2) = lastZ;
    }
    return cmd;
}

double GCodeParser::convertToMM(double value) const {
    return unitMode == UnitMode::MM ? value : value * 25.4;
}


void printGCodeCommand(const GCodeCommand& cmd) {
    std::cout << "MotionMode: " << static_cast<int>(cmd.motion) << "\n";

    if (cmd.x) std::cout << "x: " << *cmd.x << " ";
    if (cmd.y) std::cout << "y: " << *cmd.y << " ";
    if (cmd.z) std::cout << "z: " << *cmd.z << " ";
    std::cout << "\n";
	if (cmd.i) std::cout << "i: " << *cmd.i << " ";
	if (cmd.j) std::cout << "j: " << *cmd.j << " ";
	if (cmd.k) std::cout << "k: " << *cmd.k << " ";
	std::cout << "\n";
	if (cmd.f) std::cout << "f: " << *cmd.f << " ";
	if (cmd.p) std::cout << "p: " << *cmd.p << " ";
	std::cout << "\n";
	std::cout << "Unit: " << static_cast<int>(cmd.unit) << "\n";
	std::cout << "Coord: " << static_cast<int>(cmd.coord) << "\n";
	std::cout << "Plane: " << static_cast<int>(cmd.plane) << "\n";
	std::cout << "Starting X: " << cmd.startPoint(0) << " ";
	std::cout << "Starting Y: " << cmd.startPoint(1) << " ";
	std::cout << "Starting Z: " << cmd.startPoint(2) << " ";
    std::cout << "\n";
    std::cout << "MCode: " << cmd.mcode << "\n";
    std::cout << "Original line: " << cmd.originalLine << "\n";
    std::cout << "Line number: " << cmd.lineNumber << "\n";
    std::cout << "--------------------\n";
}

