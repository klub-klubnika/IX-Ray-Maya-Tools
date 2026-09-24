#include <maya/MGlobal.h>
#include <maya/MStringArray.h>
#include "maya_omf_settings.h"
#include "xr_skl_motion.h"

namespace {

struct omf_setting
{
	const char* scene_key;
	const char* option_key;
};

const omf_setting export_settings[] = {
	{ "ixrayOmfSpeed", "omf_speed" },
	{ "ixrayOmfAccrue", "omf_accrue" },
	{ "ixrayOmfFalloff", "omf_falloff" },
	{ "ixrayOmfFlags", "omf_flags" },
	{ "ixrayOmfStopAtEnd", "omf_stop_at_end" },
};

void store_scene_value(const char* key, const MString& value)
{
	MGlobal::executeCommand(MString("fileInfo \"") + key + "\" \"" + value + "\"", false, false);
}

}

void remember_omf_scene_settings(const xray_re::xr_skl_motion& motion)
{
	store_scene_value("ixrayOmfAccrue", MString() + motion.accrue());
	store_scene_value("ixrayOmfFalloff", MString() + motion.falloff());
	store_scene_value("ixrayOmfFlags", MString() + int(motion.flags()));
	store_scene_value("ixrayOmfStopAtEnd",
		motion.flags() & xray_re::xr_skl_motion::SMF_STOP_AT_END ? "true" : "false");
}

void append_omf_scene_settings(MString& options)
{
	MStringArray motion_name;
	if (MGlobal::executeCommand("fileInfo -q \"ixrayOmfMotionName\"", motion_name, false) == MS::kSuccess
		&& motion_name.length() > 0 && motion_name[0].length() > 0)
		options += MString("omf_motion_name=") + motion_name[0] + ";";

	for (const omf_setting& setting : export_settings) {
		MStringArray value;
		if (MGlobal::executeCommand(MString("fileInfo -q \"") + setting.scene_key + "\"", value, false) == MS::kSuccess
			&& value.length() > 0)
			options += MString(setting.option_key) + "=" + value[0] + ";";
	}
}
