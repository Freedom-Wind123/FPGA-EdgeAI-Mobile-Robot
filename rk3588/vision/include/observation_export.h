#pragma once
// Local observations only. FPGA remains the only autonomous motion controller.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include "types.h"

inline bool exportObservations(const std::string& path,
                               const std::vector<Detection>& dets,
                               long long capture_ms, int width, float hfov_deg) {
    if(path.empty() || width<=0 || !std::isfinite(hfov_deg) || hfov_deg<=0 || hfov_deg>=180) return false;
    const double pi=3.14159265358979323846;
    const double focal=width/(2.0*std::tan(hfov_deg*pi/360.0));
    const auto temp=path+".tmp";
    std::ofstream out(temp, std::ios::trunc);
    if(!out) return false;
    out<<"{\"timestamp_ms\":"<<capture_ms<<",\"objects\":[";
    bool first=true;
    for(const auto& d:dets) {
        if(!std::isfinite(d.conf)||!std::isfinite(d.x1)||!std::isfinite(d.x2)||!std::isfinite(d.dist_m))continue;
        if(d.dist_m<0.1f||d.dist_m>12.0f||d.conf<0.6f||d.track_id<0)continue;
        const auto bearing=std::atan((width/2.0-(d.x1+d.x2)/2.0)/focal);
        if(!first)out<<",";
        first=false;
        out<<"{\"id\":"<<d.track_id<<",\"cls\":"<<d.cls
           <<",\"color\":"<<d.color<<",\"confidence\":"<<d.conf
           <<",\"bearing_rad\":"<<bearing<<",\"range_mm\":"<<std::lround(d.dist_m*1000.0f)<<"}";
    }
    out<<"]}";
    out.close();
    if(!out)return false;
    return std::rename(temp.c_str(),path.c_str())==0;
}
