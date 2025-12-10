#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <algorithm>

using namespace std;

// Data Structure for GPS Coordinates
struct Point {
    double lat;
    double lon;
};

// --- CONFIGURATION ---
const double ALTITUDE = 30.0; // Target altitude for the mission (meters)

// --- HELPER FUNCTIONS ---

// Convert Meters to Latitude Degrees (Approximation)
// 1 Degree Latitude ~= 111,132 meters
double meters_to_lat_deg(double meters) {
    return meters / 111132.0;
}

// Convert Meters to Longitude Degrees based on current Latitude
// Longitude distance changes as we move away from the equator
double meters_to_lon_deg(double meters, double current_lat) {
    double lat_rad = current_lat * M_PI / 180.0;
    return meters / (111132.0 * cos(lat_rad));
}

// Ray-Casting Algorithm: Checks if a point lies inside a polygon
// This is a standard geometric test to determine inclusion
bool is_point_in_polygon(Point p, const vector<Point>& polygon) {
    bool inside = false;
    size_t j = polygon.size() - 1;
    for (size_t i = 0; i < polygon.size(); i++) {
        if ((polygon[i].lon > p.lon) != (polygon[j].lon > p.lon) &&
            (p.lat < (polygon[j].lat - polygon[i].lat) * (p.lon - polygon[i].lon) / (polygon[j].lon - polygon[i].lon) + polygon[i].lat)) {
            inside = !inside;
        }
        j = i;
    }
    return inside;
}

// --- MAIN GENERATOR FUNCTION ---
void generate_lawnmower(const vector<Point>& area_polygon, double spacing_meters) {

    // 1. Calculate Bounding Box (Find Min/Max limits)
    double min_lat = 90.0, max_lat = -90.0;
    double min_lon = 180.0, max_lon = -180.0;

    for (const auto& p : area_polygon) {
        if (p.lat < min_lat) min_lat = p.lat;
        if (p.lat > max_lat) max_lat = p.lat;
        if (p.lon < min_lon) min_lon = p.lon;
        if (p.lon > max_lon) max_lon = p.lon;
    }

    // 2. Calculate Step Sizes (in Degrees)
    double lat_step = meters_to_lat_deg(spacing_meters);

    // 3. Generate Waypoints
    vector<Point> waypoints;
    bool go_right = true; // Direction flag for Zig-Zag pattern

    // Scan row by row (Latitude iteration)
    for (double curr_lat = min_lat; curr_lat <= max_lat; curr_lat += lat_step) {

        // Resolution for checking points along the longitude line
        // Testing every 5 meters ensures we capture the polygon edges accurately
        double scan_resolution = meters_to_lon_deg(5.0, curr_lat);

        vector<Point> line_points;

        for (double curr_lon = min_lon; curr_lon <= max_lon; curr_lon += scan_resolution) {
            Point p = {curr_lat, curr_lon};

            // If point is inside the polygon, add to current line candidates
            if (is_point_in_polygon(p, area_polygon)) {
                line_points.push_back(p);
            }
        }

        // If no points fall inside the polygon for this row, skip it
        if (line_points.empty()) continue;

        // The START and END points of the intersection are our waypoints
        // We fly straight between start and end, so intermediate points are not needed
        Point start = line_points.front();
        Point end = line_points.back();

        if (go_right) {
            waypoints.push_back(start);
            waypoints.push_back(end);
        } else {
            waypoints.push_back(end);   // Reverse order for Zig-Zag (efficient turning)
            waypoints.push_back(start);
        }

        go_right = !go_right; // Flip direction for the next row
    }

    // 4. Write to File (QGroundControl .waypoints Format)
    string filename = "search_mission.waypoints";
    ofstream outfile(filename);

    if (outfile.is_open()) {
        // Write File Header (Standard QGC Header)
        outfile << "QGC WPL 110" << endl;

        // Write Home Point (Dummy - Index 0)
        // Column Format: Index, Curr, CoordFrame, Command, P1, P2, P3, P4, Lat, Lon, Alt, Autocontinue
        outfile << "0\t1\t0\t16\t0\t0\t0\t0\t"
                << fixed << setprecision(8) << area_polygon[0].lat << "\t"
                << area_polygon[0].lon << "\t"
                << ALTITUDE << "\t1" << endl;

        // Write Mission Waypoints
        for (size_t i = 0; i < waypoints.size(); ++i) {
            outfile << (i + 1) << "\t0\t3\t16\t0\t0\t0\t0\t" // 16 = MAV_CMD_NAV_WAYPOINT
                    << fixed << setprecision(8) << waypoints[i].lat << "\t"
                    << waypoints[i].lon << "\t"
                    << ALTITUDE << "\t1" << endl;
        }

        outfile.close();
        cout << "Success! File generated: " << filename << endl;
        cout << "Total Waypoints: " << waypoints.size() << endl;
    } else {
        cerr << "Error: Could not create output file!" << endl;
    }
}

int main() {
    // Example Scenario: SUAS Search Area Coordinates
    // Defining a simple rectangular area
    vector<Point> area = {
        {-35.362, 149.162}, // Bottom Left
        {-35.362, 149.168}, // Bottom Right
        {-35.366, 149.168}, // Top Right
        {-35.366, 149.162}  // Top Left
    };

    double spacing = 30.0; // Scan spacing in meters

    cout << "Generating Lawnmower mission..." << endl;
    generate_lawnmower(area, spacing);

    return 0;
}