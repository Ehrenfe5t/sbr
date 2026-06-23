// sbr_main.cpp — SBR 独立可执行入口 (JSON 配置驱动)
// 用法: sbr_app <config.json>
#include "sbr/sbr_engine.h"
#include "sbr/sbr_bvh_accelerator.h"
#include "sbr/sbr_scene_loader.h"
#include "sbr/sbr_math.h"
#include <cstdio>
#include <cstring>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>

using namespace sbr;

// ═══════════════════════════════════════════════════════════
// 简易 JSON 解析 (零外部依赖)
// ═══════════════════════════════════════════════════════════

static std::string readFile(const std::string& path) {
    std::ifstream f(path); if (!f) return "";
    std::stringstream ss; ss << f.rdbuf(); return ss.str();
}
static std::string jsonStr(const std::string& json, const std::string& key) {
    std::string s = "\"" + key + "\"";
    size_t p = json.find(s); if (p == std::string::npos) return "";
    p = json.find(':', p + s.size()); if (p == std::string::npos) return "";
    p = json.find('"', p + 1); if (p == std::string::npos) return "";
    size_t e = json.find('"', p + 1); if (e == std::string::npos) return "";
    return json.substr(p + 1, e - p - 1);
}
static double jsonNum(const std::string& json, const std::string& key, double def = 0.0) {
    std::string s = "\"" + key + "\"";
    size_t p = json.find(s); if (p == std::string::npos) return def;
    p = json.find(':', p + s.size()); if (p == std::string::npos) return def;
    p++; while (p < json.size() && (json[p]==' '||json[p]=='\t'||json[p]=='\n')) p++;
    char* end; double v = std::strtod(json.c_str() + p, &end);
    return (end > json.c_str() + p) ? v : def;
}
static int jsonInt(const std::string& json, const std::string& key, int def = 0) {
    return static_cast<int>(jsonNum(json, key, static_cast<double>(def)));
}
static bool jsonBool(const std::string& json, const std::string& key, bool def = false) {
    // JSON boolean: "key": true / false (unquoted)
    std::string s = "\"" + key + "\"";
    size_t p = json.find(s); if (p == std::string::npos) return def;
    p = json.find(':', p + s.size()); if (p == std::string::npos) return def;
    p++; while (p < json.size() && (json[p]==' '||json[p]=='\t'||json[p]=='\n'||json[p]=='\r')) p++;
    if (p + 4 <= json.size() && json.compare(p, 4, "true") == 0) return true;
    if (p + 5 <= json.size() && json.compare(p, 5, "false") == 0) return false;
    return def;
}

// ═══════════════════════════════════════════════════════════
// JSON 结果输出
// ═══════════════════════════════════════════════════════════

static std::string escapeJson(const std::string& s) {
    std::string r;
    for (char c : s) {
        if (c == '"')  r += "\\\"";
        else if (c == '\\') r += "/";   // Windows path: \ → /
        else r += c;
    }
    return r;
}

static void saveResultJson(const std::string& path, const SbrCoverageResult& result,
                            const SbrConfig& config, const Point3& tx,
                            double bvhMs, double sbrMs, int totalFaces, int totalWedges,
                            const std::string& objPath, const std::string& configPath) {
    std::ofstream f(path); if (!f) { std::fprintf(stderr,"Cannot write %s\n",path.c_str()); return; }
    f << "{\n";
    f << "  \"scene_file\": \"" << escapeJson(objPath) << "\",\n";
    f << "  \"config_file\": \"" << escapeJson(configPath) << "\",\n";
    f << "  \"status\": \"" << (result.succeeded ? "OK" : "FAILED") << "\",\n";
    f << "  \"stats\": {\n";
    f << "    \"total_faces\": " << totalFaces << ",\n";
    f << "    \"total_wedges\": " << totalWedges << ",\n";
    f << "    \"total_rays\": " << result.total_rays << ",\n";
    f << "    \"active_rx_count\": " << result.active_rx_count << ",\n";
    f << "    \"bvh_build_ms\": " << bvhMs << ",\n";
    f << "    \"sbr_trace_ms\": " << sbrMs << "\n";
    f << "  },\n";
    f << "  \"tx\": { \"x\": " << tx.x << ", \"y\": " << tx.y << ", \"z\": " << tx.z << " },\n";
    f << "  \"rx_records\": [\n";
    for (size_t i = 0; i < result.rx_records.size(); ++i) {
        auto& rec = result.rx_records[i];
        f << "    {\n";
        f << "      \"rx_index\": " << rec.rx_index << ",\n";
        f << "      \"position\": [" << rec.rx_position.x << "," << rec.rx_position.y << "," << rec.rx_position.z << "],\n";
        f << "      \"total_power_dBm\": " << rec.total_power_dBm << ",\n";
        f << "      \"hit_count\": " << rec.ray_hit_count << ",\n";
        f << "      \"paths\": [\n";
        for (size_t pi = 0; pi < rec.paths.size(); ++pi) {
            auto& p = rec.paths[pi];
            f << "        {";
            f << "\"len\": " << p.total_length << ", ";
            f << "\"nodes\": " << p.nodes.size() << ", ";
            f << "\"los\": " << (p.is_los ? "true" : "false") << ", ";
            f << "\"has_tx\": " << (p.contains_transmission ? "true" : "false") << ", ";
            f << "\"sequence\": \"";
            for (auto& n : p.nodes) {
                switch (n.interaction_type) {
                case InteractionType::Tx: f << 'T'; break;
                case InteractionType::Rx: f << 'R'; break;
                case InteractionType::Reflection: f << 'r'; break;
                case InteractionType::Transmission: f << 't'; break;
                case InteractionType::Diffraction: f << 'd'; break;
                default: f << '?';
                }
            }
            f << "\", ";
            f << "\"points\": [";
            for (size_t ni = 0; ni < p.nodes.size(); ++ni) {
                if (ni > 0) f << ", ";
                f << "[" << p.nodes[ni].point.x << "," << p.nodes[ni].point.y << "," << p.nodes[ni].point.z << "]";
            }
            f << "]";
            f << "}";
            if (pi + 1 < rec.paths.size()) f << ",";
            f << "\n";
        }
        f << "      ]\n";
        f << "    }";
        if (i + 1 < result.rx_records.size()) f << ",";
        f << "\n";
    }
    f << "  ]\n}\n";
    std::printf("[Output] Result JSON saved: %s\n", path.c_str());
}

static void saveResultCsv(const std::string& path, const SbrCoverageResult& result) {
    std::ofstream f(path); if (!f) return;
    f << "rx_id,rx_x,rx_y,rx_z,path_idx,length_m,nodes,is_los,has_transmission,sequence\n";
    for (auto& rec : result.rx_records) {
        for (size_t pi = 0; pi < rec.paths.size(); ++pi) {
            auto& p = rec.paths[pi];
            f << rec.rx_index << "," << rec.rx_position.x << "," << rec.rx_position.y << "," << rec.rx_position.z << ",";
            f << pi << "," << p.total_length << "," << p.nodes.size() << "," << (p.is_los?1:0) << "," << (p.contains_transmission?1:0) << ",";
            for (auto& n : p.nodes) {
                switch (n.interaction_type) {
                case InteractionType::Tx: f << 'T'; break;
                case InteractionType::Rx: f << 'R'; break;
                case InteractionType::Reflection: f << 'r'; break;
                case InteractionType::Transmission: f << 't'; break;
                case InteractionType::Diffraction: f << 'd'; break;
                default: f << '?';
                }
            }
            f << "\n";
        }
    }
    std::printf("[Output] Result CSV saved: %s\n", path.c_str());
}

// ═══════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::printf("SBR Geometric Pathfinding Module\n");
        std::printf("Usage: sbr_app <config.json>\n\n");
        std::printf("Config JSON keys:\n");
        std::printf("  scene.obj_file            Path to Wavefront .obj\n");
        std::printf("  scene.material_map_file   Path to material binding JSON\n");
        std::printf("  scene.material_csv_file   Path to ITU material CSV (optional)\n");
        std::printf("  sbr.center_frequency_hz   Center frequency (Hz)\n");
        std::printf("  sbr.ray_count             Number of rays\n");
        std::printf("  sbr.max_ray_depth         Max total bounces\n");
        std::printf("  sbr.max_reflection_count  Max reflections\n");
        std::printf("  sbr.max_transmission_count Max transmissions\n");
        std::printf("  sbr.max_diffraction_count Max diffractions\n");
        std::printf("  receiver.rx_sphere_radius_m  Receiver sphere radius\n");
        std::printf("  tx.x/y/z                  Transmitter position\n");
        std::printf("  rx_mode                   \"list\" or \"grid\"\n");
        std::printf("  rx_list[]                 Rx positions (list mode)\n");
        std::printf("  rx_grid.x_min/max/step... Rx grid (grid mode)\n");
        std::printf("  output.result_json        Output JSON path\n");
        std::printf("  output.result_csv         Output CSV path\n");
        return 1;
    }

    std::string json = readFile(argv[1]);
    if (json.empty()) { std::fprintf(stderr, "FATAL: Cannot read config\n"); return 1; }

    // ── 解析配置 ──
    std::string objPath  = jsonStr(json, "obj_file");
    std::string mapPath  = jsonStr(json, "material_map_file");
    std::string csvPath  = jsonStr(json, "material_csv_file");

    SbrConfig config;
    config.center_frequency_hz    = jsonNum(json, "center_frequency_hz", 2.4e9);
    config.ray_count              = jsonInt(json, "ray_count", 50000);
    config.max_ray_depth          = jsonInt(json, "max_ray_depth", 4);
    config.max_reflection_count   = jsonInt(json, "max_reflection_count", 3);
    config.max_transmission_count = jsonInt(json, "max_transmission_count", 0);
    config.max_diffraction_count  = jsonInt(json, "max_diffraction_count", 0);
    config.diffraction_rays_per_event = jsonInt(json, "diffraction_rays_per_event", 4);
    config.wedge_max_distance_m   = jsonNum(json, "wedge_max_distance_m", 5.0);
    config.wedge_max_candidates   = jsonInt(json, "wedge_max_candidates", 8);
    config.ray_power_threshold_dB = jsonNum(json, "ray_power_threshold_dB", -80.0);
    config.tx_power_dBm           = jsonNum(json, "tx_power_dBm", 0.0);

    config.rx_sphere_radius_m     = jsonNum(json, "rx_sphere_radius_m", 0.3);
    config.enable_dynamic_rx_radius = jsonBool(json, "enable_dynamic_rx_radius", true);
    config.ray_tube_radius_scale  = jsonNum(json, "ray_tube_radius_scale", 0.5);
    config.ray_tube_min_radius_m  = jsonNum(json, "ray_tube_min_radius_m", 0.1);
    config.ray_tube_max_radius_m  = jsonNum(json, "ray_tube_max_radius_m", 2.0);

    config.enable_path_dedup             = jsonBool(json, "enable_path_dedup", true);
    config.enable_path_similarity_pruning = jsonBool(json, "enable_path_similarity_pruning", true);
    config.path_similarity_length_tol_m  = jsonNum(json, "path_similarity_length_tol_m", 0.05);
    config.path_top_n_per_rx             = jsonInt(json, "path_top_n_per_rx", 20);
    config.store_paths                   = true;  // always store for visualization
    std::printf("[Config] store_paths=%d dedup=%d sim=%d topN=%d\n",
                (int)config.store_paths, (int)config.enable_path_dedup,
                (int)config.enable_path_similarity_pruning, config.path_top_n_per_rx);

    Point3 txPoint = MakeVec3(jsonNum(json, "x", 10.0), jsonNum(json, "y", 2.0), jsonNum(json, "z", -2.0));

    // Tx 在 "tx" 子对象中的位置需要特殊处理
    {
        size_t txp = json.find("\"tx\"");
        if (txp != std::string::npos) {
            std::string txBlock = json.substr(txp);
            double txx = jsonNum(txBlock, "x", 10.0);
            double txy = jsonNum(txBlock, "y", 2.0);
            double txz = jsonNum(txBlock, "z", -2.0);
            txPoint = MakeVec3(txx, txy, txz);
        }
    }

    std::string rxMode = jsonStr(json, "rx_mode");
    std::vector<Point3> rxPoints;

    if (rxMode == "grid") {
        double xmin = jsonNum(json, "x_min", 3.0), xmax = jsonNum(json, "x_max", 17.0);
        double xs   = jsonNum(json, "x_step", 3.0);
        double ymin = jsonNum(json, "y_min", 0.5), ymax = jsonNum(json, "y_max", 2.5);
        double ys   = jsonNum(json, "y_step", 2.0);
        double zmin = jsonNum(json, "z_min", -18.0), zmax = jsonNum(json, "z_max", -2.0);
        double zs   = jsonNum(json, "z_step", 4.0);
        for (double x = xmin; x <= xmax + xs*0.1; x += xs)
        for (double y = ymin; y <= ymax + ys*0.1; y += ys)
        for (double z = zmin; z <= zmax + zs*0.1; z += zs)
            rxPoints.push_back(MakeVec3(x, y, z));
        if (rxPoints.size() > 100) rxPoints.resize(100);
    } else {
        // list mode: scan for rx_list array entries
        size_t rxp = json.find("\"rx_list\"");
        if (rxp != std::string::npos) {
            size_t p = json.find('[', rxp);
            if (p != std::string::npos) {
                // parse each { "id":..., "x":..., "y":..., "z":... } block
                size_t end = json.find(']', p);
                std::string listBlock = json.substr(p, end - p + 1);
                size_t pos = 0;
                while ((pos = listBlock.find("\"x\"", pos)) != std::string::npos) {
                    double rx = jsonNum(listBlock.substr(pos), "x", txPoint.x);
                    double ry = jsonNum(listBlock.substr(pos), "y", txPoint.y);
                    double rz = jsonNum(listBlock.substr(pos), "z", txPoint.z);
                    rxPoints.push_back(MakeVec3(rx, ry, rz));
                    pos++;
                }
            }
        }
    }
    if (rxPoints.empty()) {
        rxPoints.push_back(MakeVec3(txPoint.x + 2, txPoint.y, txPoint.z));
    }

    std::string outJson = jsonStr(json, "result_json");
    std::string outCsv  = jsonStr(json, "result_csv");
    if (outJson.empty()) outJson = "output/sbr_result.json";
    if (outCsv.empty())  outCsv  = "output/sbr_paths.csv";

    // ── 1-4. 加载 ──
    MaterialDatabase matDb;
    if (!csvPath.empty()) matDb.LoadFromCsv(csvPath);

    Scene scene;
    if (!LoadOBJ(objPath, scene)) return 1;
    LoadMaterialMap(mapPath, scene, matDb, config.center_frequency_hz);
    BuildEdgeWedgeTopology(scene, 30.0);

    // ── 5. BVH ──
    auto t0 = std::chrono::steady_clock::now();
    auto bvh = std::make_unique<BvhAccelerator>(scene);
    auto t1 = std::chrono::steady_clock::now();
    double bvhMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("[SBR] Scene: %zu faces, %zu wedges, BVH: %.0f ms\n",
                scene.faces.size(), scene.wedges.size(), bvhMs);

    // ── 6. Run ──
    SbrEngine engine(std::move(bvh));
    NumericToleranceConfig tol;
    auto t2 = std::chrono::steady_clock::now();
    SbrCoverageResult result = engine.RunCoverage(scene, matDb, config, txPoint, rxPoints, tol);
    auto t3 = std::chrono::steady_clock::now();
    double sbrMs = std::chrono::duration<double, std::milli>(t3 - t2).count();

    long long totalPaths = 0; int rxHit = 0;
    for (auto& rec : result.rx_records) { totalPaths += rec.ray_hit_count; if (rec.ray_hit_count > 0) rxHit++; }

    long long dR=0,dT=0,dD=0;
    for (auto& rec : result.rx_records) for (auto& p : rec.paths) for (auto& n : p.nodes) {
        if (n.interaction_type == InteractionType::Reflection) dR++;
        else if (n.interaction_type == InteractionType::Transmission) dT++;
        else if (n.interaction_type == InteractionType::Diffraction) dD++;
    }
    std::printf("[SBR] Trace: %.0f ms | %d rays | %lld paths | %d/%zu Rx hit | R=%lld T=%lld D=%lld\n",
                sbrMs, result.total_rays, totalPaths, rxHit, rxPoints.size(), dR, dT, dD);

    // ── 7. Output ──
    saveResultJson(outJson, result, config, txPoint, bvhMs, sbrMs,
                   static_cast<int>(scene.faces.size()),
                   static_cast<int>(scene.wedges.size()),
                   objPath, argv[1]);
    saveResultCsv(outCsv, result);

    return result.succeeded ? 0 : 1;
}
