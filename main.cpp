#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <mavlink/common/mavlink.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;
using namespace std;

// --- CONFIGURATION ---
constexpr double MAX_ALTITUDE_LIMIT = 120.0;
constexpr double NFZ_LAT = -35.362000;
constexpr double NFZ_LON = 149.164000;
constexpr double NFZ_RADIUS = 50.0;     // Meter
constexpr double HOME_RADIUS_TH = 15.0; // Distance threshold to Home for the end/start of the run

// Data Structure
struct DataPoint {
    uint32_t time_boot_ms = 0;
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
};

// Structure to store Lap Results
struct LapInfo {
    int lap_id = 0;
    bool is_valid = true;
    double max_alt = 0.0;
    string fail_reason = "None";
};

// --- HELPER FUNCTIONS ---
// Haversine Formula: Distance in meters between two GPS coordinates
double get_distance_metres(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0; // Earth Radius
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double delta_phi = (lat2 - lat1) * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}

int main() {

    string filename = "mission.tlog";
    ifstream logfile(filename, ios::binary);

    if (!logfile.is_open()) {
        cerr << "ERROR: " << filename << " not found!" << endl;
        cerr << "Hint: Did you run 'generate_mission_log.py'?" << endl;
        return 1;
    }

    mavlink_message_t msg;
    mavlink_status_t status;

    // Using vector to keep data on the heap (Prevents Memory Leaks)
    vector<DataPoint> flight_data;
    vector<LapInfo> laps;

    // Lap Logic Variables
    bool is_in_lap = false;
    bool current_lap_valid = true;
    double current_lap_max_alt = 0;
    string current_fail_reason = "None";
    int lap_counter = 0;

    // Home Point (First takeoff location)
    DataPoint home_pos{};
    bool home_set = false;

    cout << "Analyzing log file..." << endl;

    char byte;
    while (logfile.get(byte)) {
        if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)byte, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t pos;
                mavlink_msg_global_position_int_decode(&msg, &pos);

                DataPoint dp{};
                dp.time_boot_ms = pos.time_boot_ms;
                dp.lat = pos.lat / 1E7;
                dp.lon = pos.lon / 1E7;
                dp.alt = pos.relative_alt / 1000.0; // mm -> m

                // Filter ground data (Keep only in-air data)
                if (dp.alt > 1.0) {
                    flight_data.push_back(dp);

                    // Determine Home Point (When altitude exceeds 1m for the first time)
                    if (!home_set) {
                        home_pos = dp;
                        home_set = true;
                        cout << "Home point locked: " << home_pos.lat << ", " << home_pos.lon << endl;
                    }

                    // --- LAP ALGORITHM ---
                    double dist_to_home = get_distance_metres(dp.lat, dp.lon, home_pos.lat, home_pos.lon);

                    // Case 1: Lap Start (Moved away from Home)
                    if (!is_in_lap && dist_to_home > HOME_RADIUS_TH + 5.0) { // +5m histerezis
                        is_in_lap = true;
                        lap_counter++;
                        // Reset variables for the new lap
                        current_lap_valid = true;
                        current_fail_reason = "Clean";
                        current_lap_max_alt = 0;
                    }

                    // Case 2: In-Lap Checks
                    if (is_in_lap) {
                        // Max altitude tracking
                        if (dp.alt > current_lap_max_alt) current_lap_max_alt = dp.alt;

                        // REQUIREMENT 1: Altitude Check
                        if (dp.alt > MAX_ALTITUDE_LIMIT) {
                            if (current_lap_valid) { // Record only the first failure
                                current_lap_valid = false;
                                current_fail_reason = "ALTITUDE VIOLATION (" + to_string((int)dp.alt) + "m)";
                            }
                        }

                        // REQUIREMENT 2: NFZ Check
                        double dist_to_nfz = get_distance_metres(dp.lat, dp.lon, NFZ_LAT, NFZ_LON);
                        if (dist_to_nfz < NFZ_RADIUS) {
                            if (current_lap_valid) {
                                current_lap_valid = false;
                                current_fail_reason = "NFZ VIOLATION (Inside Zone)";
                            }
                        }

                        // Case 3: Lap Finish (Returned to Home)
                        if (dist_to_home < HOME_RADIUS_TH) {
                            is_in_lap = false;

                            // Save lap result
                            LapInfo result{};
                            result.lap_id = lap_counter;
                            result.is_valid = current_lap_valid;
                            result.max_alt = current_lap_max_alt;
                            result.fail_reason = current_fail_reason;
                            laps.push_back(result);
                        }
                    }
                }
            }
        }
    }

    logfile.close();

    // --- REPORTING (REQ: Flight Summary) ---
    cout << "\n========================================" << endl;
    cout << "           FLIGHT ANALYSIS REPORT           " << endl;
    cout << "========================================" << endl;

    if (flight_data.empty()) {
        cout << "[!] No flight data found." << endl;
        return 0;
    }

    int valid_laps = 0;
    for (const auto& lap : laps) {
        cout << "LAP #" << lap.lap_id << " -> ";
        if (lap.is_valid) {
            cout << "[ SUCCESS ] ";
            valid_laps++;
        } else {
            cout << "[ FAILED   ] ";
        }
        cout << "| Max Alt: " << fixed << setprecision(1) << lap.max_alt << "m ";
        cout << "| Note: " << lap.fail_reason << endl;
    }

    cout << "----------------------------------------" << endl;
    cout << "Total laps: " << laps.size() << endl;
    cout << "Valid laps: " << valid_laps << endl;
    cout << "Success rate: %" << (laps.empty() ? 0 : (valid_laps * 100 / laps.size())) << endl;

    // --- PLOTTING (REQ: 2D Graph) ---
    vector<double> time_axis, alt_axis, lat_axis, lon_axis;
    vector<double> nfz_circle_lat, nfz_circle_lon;

    double start_time = flight_data[0].time_boot_ms;

    for (const auto& d : flight_data) {
        time_axis.push_back((d.time_boot_ms - start_time) / 1000.0);
        alt_axis.push_back(d.alt);
        lat_axis.push_back(d.lat);
        lon_axis.push_back(d.lon);
    }

    // Draw NFZ Circle (For visual reference)
    for (int i = 0; i <= 360; i += 5) {
        double theta = i * M_PI / 180.0;
        // Simple conversion from meters to degrees (Valid for small areas)
        double dlat = (NFZ_RADIUS / 6371000.0) * (180.0 / M_PI);
        double dlon = dlat / cos(NFZ_LAT * M_PI / 180.0);

        nfz_circle_lat.push_back(NFZ_LAT + dlat * sin(theta));
        nfz_circle_lon.push_back(NFZ_LON + dlon * cos(theta));
    }

    try {
        plt::backend("TkAgg"); // Stable backend

        // Window 1: Altitude Analysis
        plt::figure(1);
        plt::plot(time_axis, alt_axis);
        // Limit Line (120m)
        vector<double> limit_line(time_axis.size(), MAX_ALTITUDE_LIMIT);
        plt::plot(time_axis, limit_line, {{"color", "red"}, {"linestyle", "--"}, {"label", "Limit (120m)"}});

        plt::title("Time vs Altitude");
        plt::xlabel("Time (s)");
        plt::ylabel("Altitude (m)");
        plt::legend();
        plt::grid(true);

        // Window 2: Trajectory and NFZ
        plt::figure(2);
        plt::plot(lon_axis, lat_axis, {{"label", "Ucus Rotasi"}, {"color", "blue"}});
        plt::plot(nfz_circle_lon, nfz_circle_lat, {{"label", "NO-FLY ZONE"}, {"color", "red"}, {"linewidth", "2"}});

        // Mark Home Point
        vector<double> hx = {home_pos.lon}, hy = {home_pos.lat};
        plt::plot(hx, hy, {{"marker", "s"}, {"color", "green"}, {"label", "Home"}});

        plt::title("2D Mission Path and NFZ Analysis");
        plt::xlabel("Longitude");
        plt::ylabel("Latitude");
        plt::axis("equal"); // Maintain map aspect ratio
        plt::legend();
        plt::grid(true);

        cout << "Plots drawn. Opening windows..." << endl;
        plt::show();
    }
    catch(const std::exception& e) {
        cerr << "Plotting error: " << e.what() << endl;
    }

    return 0;
}