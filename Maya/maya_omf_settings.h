#pragma once

#include <maya/MString.h>

namespace xray_re {
class xr_skl_motion;
}

// Scene-persistent OMF metadata shared by the Motion Browser and export tools.
// Speed is intentionally not restored from a source motion: its default follows
// the current Maya frame rate, while the export dialog remains editable.
void remember_omf_scene_settings(const xray_re::xr_skl_motion& motion);
void append_omf_scene_settings(MString& options);
