// sbr_scene_loader.cpp — OBJ + 材质JSON + 拓扑构建
#include "sbr/sbr_scene_loader.h"
#include "sbr/sbr_math.h"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#include <cstdio>

namespace sbr {

// ═══════════════════════════════════════════════════════════
// OBJ 加载 (Wavefront .obj ASCII, 格式: o / v / vn / f v//vn)
// ═══════════════════════════════════════════════════════════

bool LoadOBJ(const std::string& path, Scene& scene) {
    std::ifstream f(path);
    if (!f.is_open()) { std::fprintf(stderr, "[OBJ] Cannot open: %s\n", path.c_str()); return false; }

    scene.vertices.clear(); scene.normals.clear();
    scene.faces.clear(); scene.objects.clear();

    std::string line, currentObject;
    int objectId = 0;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string token; ss >> token;

        if (token == "o" || token == "g") {
            ss >> currentObject;
            if (!currentObject.empty()) {
                SceneObjectRecord obj;
                obj.object_id = objectId++;
                obj.object_name = currentObject;
                scene.objects.push_back(obj);
            }
        }
        else if (token == "v") {
            double x, y, z; ss >> x >> y >> z;
            scene.vertices.push_back(MakeVec3(x, y, z));
        }
        else if (token == "vn") {
            double nx, ny, nz; ss >> nx >> ny >> nz;
            scene.normals.push_back(Normalize(MakeVec3(nx, ny, nz)));
        }
        else if (token == "f") {
            // 格式: f v1//vn1 v2//vn2 v3//vn3 (只支持三角面)
            std::string v1, v2, v3, v4;
            ss >> v1 >> v2 >> v3;
            if (v1.empty()) continue;

            auto parseVertex = [](const std::string& s, int& vi, int& ni) {
                vi = ni = -1;
                size_t d1 = s.find('/');
                if (d1 == std::string::npos) { vi = std::stoi(s) - 1; return; }
                vi = std::stoi(s.substr(0, d1)) - 1;
                size_t d2 = s.find('/', d1 + 1);
                if (d2 != std::string::npos) ni = std::stoi(s.substr(d2 + 1)) - 1;
            };

            int vi0, ni0, vi1, ni1, vi2, ni2;
            parseVertex(v1, vi0, ni0); parseVertex(v2, vi1, ni1); parseVertex(v3, vi2, ni2);

            Face face;
            face.face_id = static_cast<int>(scene.faces.size());
            face.object_id = objectId - 1;
            if (!currentObject.empty() && !scene.objects.empty())
                scene.objects.back().face_ids.push_back(face.face_id);
            face.object_name = currentObject;
            face.vertex_index0 = vi0; face.vertex_index1 = vi1; face.vertex_index2 = vi2;
            face.normal_index = ni0;

            // 计算面法向
            if (vi0 >= 0 && vi1 >= 0 && vi2 >= 0 &&
                vi0 < (int)scene.vertices.size() &&
                vi1 < (int)scene.vertices.size() &&
                vi2 < (int)scene.vertices.size()) {
                Vec3 e1 = SubtractVec(scene.vertices[vi1], scene.vertices[vi0]);
                Vec3 e2 = SubtractVec(scene.vertices[vi2], scene.vertices[vi0]);
                face.normal = Normalize(Cross(e1, e2));
                face.centroid = Scale(AddVec(AddVec(scene.vertices[vi0],
                    scene.vertices[vi1]), scene.vertices[vi2]), 1.0/3.0);
                face.area = 0.5 * Length(Cross(e1, e2));
                face.bounds.min = MakeVec3(
                    std::min({scene.vertices[vi0].x, scene.vertices[vi1].x, scene.vertices[vi2].x}),
                    std::min({scene.vertices[vi0].y, scene.vertices[vi1].y, scene.vertices[vi2].y}),
                    std::min({scene.vertices[vi0].z, scene.vertices[vi1].z, scene.vertices[vi2].z}));
                face.bounds.max = MakeVec3(
                    std::max({scene.vertices[vi0].x, scene.vertices[vi1].x, scene.vertices[vi2].x}),
                    std::max({scene.vertices[vi0].y, scene.vertices[vi1].y, scene.vertices[vi2].y}),
                    std::max({scene.vertices[vi0].z, scene.vertices[vi1].z, scene.vertices[vi2].z}));
                face.bounds.valid = true;
            }

            // 默认: 仅反射
            face.reflection_enabled = true;
            face.transmission_enabled = false;
            face.diffraction_candidate_enabled = false;
            face.front_material_name = "Air";
            face.back_material_name = "Concrete";
            face.surface_eps_r = 7.0;

            scene.faces.push_back(face);
        }
    }

    std::printf("[OBJ] Loaded: %zu vertices, %zu faces, %zu objects from %s\n",
                scene.vertices.size(), scene.faces.size(), scene.objects.size(), path.c_str());
    return !scene.faces.empty();
}

// ═══════════════════════════════════════════════════════════
// 材质绑定 JSON 加载 (简化解析, 无外部依赖)
// ═══════════════════════════════════════════════════════════

// 简易JSON值提取 (在整个字符串中搜索 key)
static std::string JsonStrValGlobal(const std::string& json, const std::string& key) {
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return "";
    pos = json.find(':', pos + search.size());
    if (pos == std::string::npos) return "";
    pos = json.find('"', pos + 1);
    if (pos == std::string::npos) return "";
    size_t end = json.find('"', pos + 1);
    if (end == std::string::npos) return "";
    return json.substr(pos + 1, end - pos - 1);
}
static bool JsonBoolValGlobal(const std::string& json, const std::string& key, bool def = false) {
    // JSON boolean is unquoted: "key": true / "key": false
    // Don't use JsonStrValGlobal (which looks for quoted strings)
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return def;
    pos = json.find(':', pos);
    if (pos == std::string::npos) return def;
    pos++; while (pos < json.size() && (json[pos]==' ' || json[pos]=='\t' || json[pos]=='\n' || json[pos]=='\r')) pos++;
    if (pos + 4 <= json.size() && json.compare(pos, 4, "true") == 0) return true;
    if (pos + 5 <= json.size() && json.compare(pos, 5, "false") == 0) return false;
    return def;
}

// 从 JSON 文本中提取每个对象块 {...}
// 跳过外层, 提取嵌套在数组内的每个 {...} 对象
static std::vector<std::string> ExtractObjectBlocks(const std::string& json) {
    std::vector<std::string> blocks;
    // 找到 "objects" 数组的起始 [
    size_t arrStart = json.find("\"objects\"");
    if (arrStart == std::string::npos) return blocks;
    arrStart = json.find('[', arrStart);
    if (arrStart == std::string::npos) return blocks;

    // 在数组内提取每个 {...} 对象
    size_t pos = arrStart + 1;
    while (pos < json.size()) {
        // 跳过空白找到 {
        while (pos < json.size() && json[pos] != '{' && json[pos] != ']') pos++;
        if (pos >= json.size() || json[pos] == ']') break;

        // 找到匹配的 }
        size_t start = pos;
        int depth = 1;
        pos++;
        while (pos < json.size() && depth > 0) {
            if (json[pos] == '{') depth++;
            else if (json[pos] == '}') depth--;
            pos++;
        }
        if (depth == 0)
            blocks.push_back(json.substr(start, pos - start));
        // pos now points past the closing }
    }
    return blocks;
}

bool LoadMaterialMap(const std::string& path, Scene& scene,
                     MaterialDatabase& matDb, double freqHz) {
    // 读取整个文件
    std::ifstream f(path);
    if (!f.is_open()) { std::fprintf(stderr, "[MatMap] Cannot open: %s\n", path.c_str()); return false; }
    std::stringstream ss; ss << f.rdbuf();
    std::string json = ss.str();

    // 构建 object_name → object_id 映射
    std::unordered_map<std::string, int> nameToObj;
    for (const auto& obj : scene.objects)
        nameToObj[obj.object_name] = obj.object_id;

    // 提取每个对象块并解析属性
    auto blocks = ExtractObjectBlocks(json);
    int matchedFaces = 0;

    for (const auto& block : blocks) {
        std::string objName = JsonStrValGlobal(block, "object_name");
        if (objName.empty()) continue;

        std::string frontMat = JsonStrValGlobal(block, "front_material_name");
        std::string backMat  = JsonStrValGlobal(block, "back_material_name");
        bool reflect  = JsonBoolValGlobal(block, "reflection_enabled", true);
        bool transmit = JsonBoolValGlobal(block, "transmission_enabled", false);
        bool diffract = JsonBoolValGlobal(block, "diffraction_candidate_enabled", false);

        auto it = nameToObj.find(objName);
        if (it == nameToObj.end()) continue;

        // 查询材质 ε_r
        double epsR = 7.0, sigma = 0.015;
        if (!matDb.empty()) {
            auto p = matDb.QueryByName(backMat, freqHz);
            if (p.epsilon_r > 1.0) { epsR = p.epsilon_r; sigma = p.sigma; }
        }

        auto& obj = scene.objects[it->second];
        for (int fi : obj.face_ids) {
            if (fi >= 0 && fi < (int)scene.faces.size()) {
                Face& face = scene.faces[fi];
                face.front_material_name = frontMat;
                face.back_material_name  = backMat;
                face.surface_material_name = backMat;
                face.reflection_enabled = reflect;
                face.transmission_enabled = transmit;
                face.diffraction_candidate_enabled = diffract;
                face.surface_eps_r = epsR;
                face.surface_sigma = sigma;
                face.object_type = JsonStrValGlobal(block, "object_type");
                matchedFaces++;
            }
        }
    }

    std::printf("[MatMap] Applied to %d faces from %s\n", matchedFaces, path.c_str());
    return matchedFaces > 0;
}

// ═══════════════════════════════════════════════════════════
// 边/楔边拓扑构建
// ═══════════════════════════════════════════════════════════

// 无向边键
struct EdgeKey {
    int v0, v1;
    EdgeKey(int a, int b) : v0(std::min(a,b)), v1(std::max(a,b)) {}
    bool operator==(const EdgeKey& o) const { return v0==o.v0 && v1==o.v1; }
};
struct EdgeKeyHash { size_t operator()(const EdgeKey& k) const { return (size_t)k.v0*104729 + k.v1; } };

void BuildEdgeWedgeTopology(Scene& scene, double wedgeAngleMinDeg) {
    // UTD 阶段1: 基本楔边条件
    // 1. 两相邻面元共享边  2. 二面角(法线夹角) ∈ [3°, 177°]
    // 文献: Keller 1962; Kouyoumjian & Pathak 1974
    const double kDihedralMin = 3.0, kDihedralMax = 177.0;
    scene.edges.clear(); scene.wedges.clear();

    // 1. 收集共享边: 无向边 → 相邻面列表
    std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHash> edgeToFaces;
    for (size_t fi = 0; fi < scene.faces.size(); ++fi) {
        const Face& f = scene.faces[fi];
        int vs[3] = {f.vertex_index0, f.vertex_index1, f.vertex_index2};
        for (int ei = 0; ei < 3; ++ei) {
            EdgeKey key(vs[ei], vs[(ei+1)%3]);
            edgeToFaces[key].push_back(static_cast<int>(fi));
        }
    }

    // 2. 构建 Edge 对象
    std::unordered_map<EdgeKey, int, EdgeKeyHash> keyToEdgeId;
    for (const auto& [key, adjFaces] : edgeToFaces) {
        Edge e;
        e.edge_id = static_cast<int>(scene.edges.size());
        e.vertex_index0 = key.v0; e.vertex_index1 = key.v1;
        e.start = scene.vertices[key.v0]; e.end = scene.vertices[key.v1];
        e.direction = Normalize(SubtractVec(e.end, e.start));
        e.midpoint = Scale(AddVec(e.start, e.end), 0.5);
        e.length = Length(SubtractVec(e.end, e.start));
        e.face_id0 = adjFaces.size() > 0 ? adjFaces[0] : -1;
        e.face_id1 = adjFaces.size() > 1 ? adjFaces[1] : -1;
        e.is_boundary = (adjFaces.size() < 2);
        e.is_non_manifold = (adjFaces.size() > 2);

        // 计算二面角 (如果两个相邻面)
        if (adjFaces.size() >= 2) {
            const Vec3& n0 = scene.faces[adjFaces[0]].normal;
            const Vec3& n1 = scene.faces[adjFaces[1]].normal;
            double dotN = Clamp(Dot(n0, n1), -1.0, 1.0);
            e.dihedral_angle_deg = std::acos(dotN) * 180.0 / kPi;
            // supports_wedge: 排除共面边 (接近0°或180°) 和过于尖锐的边
            e.supports_wedge = (e.dihedral_angle_deg >= kDihedralMin &&
                                e.dihedral_angle_deg <= kDihedralMax);
        }

        keyToEdgeId[key] = e.edge_id;
        scene.edges.push_back(e);
    }

    // 3. 关联面到边 (设置 adjacent_edge_id)
    for (size_t fi = 0; fi < scene.faces.size(); ++fi) {
        Face& f = scene.faces[fi];
        int vs[3] = {f.vertex_index0, f.vertex_index1, f.vertex_index2};
        for (int ei = 0; ei < 3; ++ei) {
            EdgeKey key(vs[ei], vs[(ei+1)%3]);
            auto it = keyToEdgeId.find(key);
            int eid = (it != keyToEdgeId.end()) ? it->second : -1;
            if (ei == 0) f.adjacent_edge_id0 = eid;
            else if (ei == 1) f.adjacent_edge_id1 = eid;
            else f.adjacent_edge_id2 = eid;
        }
    }

    // 4. 为支持楔边的边创建 Wedge (UTD 外角 = 360° - 二面角)
    for (const auto& e : scene.edges) {
        if (!e.supports_wedge) continue;

        // UTD楔边候选: 两面共享边 + 法线夹角∈[3°,177°]
        // 大侧 = 360°-dihedral, 小侧 = dihedral
        // 阶段1不判断Tx/Rx位置, 全部标记为候选
        double exteriorAngle = 360.0 - e.dihedral_angle_deg;  // 大侧
        bool isCandidate = (e.dihedral_angle_deg >= kDihedralMin && e.dihedral_angle_deg <= kDihedralMax);

        Wedge w;
        w.wedge_id = static_cast<int>(scene.wedges.size());
        w.source_edge_id = e.edge_id;
        w.positive_face_id = e.face_id0;
        w.negative_face_id = e.face_id1;
        w.segment_start = e.start; w.segment_end = e.end;
        w.center_point = e.midpoint; w.direction = e.direction;
        w.length = e.length;
        w.wedge_angle_deg = exteriorAngle;           // 大侧(自由空间侧)
        w.dihedral_angle_deg = e.dihedral_angle_deg; // 小侧(法线指向侧)
        w.diffractable = isCandidate;
        w.valid_for_utd = isCandidate;
        w.convexity = (exteriorAngle > 180.0) ? WedgeConvexity::Convex : WedgeConvexity::Concave;
        w.bounds.min = MakeVec3(std::min(e.start.x, e.end.x),
                                 std::min(e.start.y, e.end.y),
                                 std::min(e.start.z, e.end.z));
        w.bounds.max = MakeVec3(std::max(e.start.x, e.end.x),
                                 std::max(e.start.y, e.end.y),
                                 std::max(e.start.z, e.end.z));
        w.bounds.valid = true;
        scene.wedges.push_back(w);
    }

    // 统计
    int diffCount = 0;
    for (auto& w : scene.wedges) if (w.diffractable) diffCount++;
    std::printf("[Topo] Built: %zu edges, %zu wedges (%d diffractable) from %zu faces\n",
                scene.edges.size(), scene.wedges.size(), diffCount,
                scene.faces.size());
}

} // namespace sbr
