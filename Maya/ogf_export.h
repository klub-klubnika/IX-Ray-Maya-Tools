#pragma once

#include <string>
namespace xray_re { class xr_object; }

enum class ogf_smoothing { normals, soc, cscop };

// Compiles an editable skeleton into the SDK's non-progressive OGF v4 format.
// The object must contain triangulated meshes in the skeleton's bind space.
bool save_skeletal_ogf(xray_re::xr_object& object, const char* path,
    ogf_smoothing smoothing, unsigned influence_limit, const std::string& motion_refs,
    std::string& error);
