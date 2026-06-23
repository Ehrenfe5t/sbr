// sbr_material.h — 材质数据库 (与 H2hRT rt::MaterialDatabase 兼容)
#pragma once
#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <unordered_set>

namespace sbr {

// ── 材质属性 ──
struct MaterialProps {
    double epsilon_r = 1.0;
    double sigma     = 0.0;
    double mu_r      = 1.0;
    std::string name;
};

// ── 材质数据库 ──
class MaterialDatabase {
public:
    // 从 CSV 加载: id,name,category,frequency_Hz,epsilon_r,sigma,mu_r
    bool LoadFromCsv(const std::string& filePath);

    // 按名称查询 (频率插值)
    MaterialProps QueryByName(const std::string& name, double freqHz) const;

    // 按 ID 查询 (频率插值)
    MaterialProps QueryById(int id, double freqHz) const;

    bool empty() const { return byName_.empty(); }

    bool HasMaterial(const std::string& name) const {
        return !name.empty() && byName_.find(name) != byName_.end();
    }

private:
    std::unordered_map<std::string, std::map<double, MaterialProps>> byName_;
    std::unordered_map<int, std::map<double, MaterialProps>> byId_;

    static MaterialProps Interpolate(const std::map<double, MaterialProps>& data, double freq);
};

} // namespace sbr
