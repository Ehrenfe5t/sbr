// sbr_material.cpp — MaterialDatabase CSV 加载 + 查询实现
#include "sbr/sbr_material.h"
#include <cstdio>
#include <cstdlib>

namespace sbr {

bool MaterialDatabase::LoadFromCsv(const std::string& filePath) {
    std::ifstream f(filePath);
    if (!f.is_open()) return false;

    std::string line;
    std::getline(f, line); // skip header

    while (std::getline(f, line)) {
        if (line.empty()) continue;

        std::vector<std::string> parts;
        std::string token;
        std::istringstream ss(line);
        while (std::getline(ss, token, ',')) {
            parts.push_back(token);
        }
        if (parts.size() < 7) continue;

        int id; double freq, epsR, sigma, muR; std::string name;
        try {
            id    = std::stoi(parts[0]);
            name  = parts[1];
            freq  = std::stod(parts[3]);
            epsR  = std::stod(parts[4]);
            sigma = std::stod(parts[5]);
            muR   = std::stod(parts[6]);
        } catch (...) { continue; }

        // 清理中文注解: "Concrete[水泥]" → "Concrete"
        {
            auto bp = name.find('[');
            if (bp != std::string::npos) name = name.substr(0, bp);
        }
        {
            auto bp = name.find('(');
            if (bp != std::string::npos) name = name.substr(0, bp);
        }
        while (!name.empty() && name.back() == ' ') name.pop_back();

        MaterialProps p;
        p.epsilon_r = epsR;
        p.sigma     = sigma;
        p.mu_r      = muR;
        p.name      = name;

        byName_[name][freq] = p;
        byId_[id][freq]     = p;
    }

    return !byName_.empty();
}

MaterialProps MaterialDatabase::QueryByName(const std::string& name, double freqHz) const {
    auto itN = byName_.find(name);
    if (itN == byName_.end()) {
        // 每材质名仅告警一次
        static std::unordered_set<std::string> warned;
        static std::mutex warnMutex;
        {
            std::lock_guard<std::mutex> lock(warnMutex);
            if (warned.insert(name).second) {
                std::fprintf(stderr,
                    "[MaterialDB] WARNING: material '%s' not found, using vacuum (eps_r=1.0)\n",
                    name.c_str());
            }
        }
        return MaterialProps{};
    }
    return Interpolate(itN->second, freqHz);
}

MaterialProps MaterialDatabase::QueryById(int id, double freqHz) const {
    auto it = byId_.find(id);
    if (it == byId_.end()) return MaterialProps{};
    return Interpolate(it->second, freqHz);
}

MaterialProps MaterialDatabase::Interpolate(
    const std::map<double, MaterialProps>& data, double freq) {
    if (data.empty()) return MaterialProps{};

    auto it = data.lower_bound(freq);
    if (it == data.begin()) return it->second;
    if (it == data.end())   return data.rbegin()->second;

    auto prev = std::prev(it);
    double f0 = prev->first, f1 = it->first;

    // ITU-R P.2040 幂律模型: 对数-对数插值
    double logF0 = std::log10(f0);
    double logF1 = std::log10(f1);
    double logF  = std::log10(freq);
    double frac  = (logF - logF0) / (logF1 - logF0);

    const auto& p0 = prev->second;
    const auto& p1 = it->second;

    MaterialProps r;
    r.name = p0.name;
    r.epsilon_r = std::pow(10.0,
        std::log10(std::max(1.0, p0.epsilon_r)) +
        frac * (std::log10(std::max(1.0, p1.epsilon_r)) - std::log10(std::max(1.0, p0.epsilon_r))));
    r.sigma = std::pow(10.0,
        std::log10(std::max(1e-15, p0.sigma)) +
        frac * (std::log10(std::max(1e-15, p1.sigma)) - std::log10(std::max(1e-15, p0.sigma))));
    r.mu_r = p0.mu_r + frac * (p1.mu_r - p0.mu_r);

    return r;
}

} // namespace sbr
