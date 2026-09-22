#pragma once
#ifndef __MAYA_MOTIONS_BROWSER_H__
#define __MAYA_MOTIONS_BROWSER_H__

#include <maya/MSyntax.h>
#include <maya/MString.h>

MStatus record_imported_motion_file(const MString& path);

void*	motion_list_command_creator();
MSyntax	motion_list_syntax_creator();
void*	motion_load_command_creator();
MSyntax	motion_load_syntax_creator();
void*	motion_export_command_creator();
MSyntax	motion_export_syntax_creator();
void*	motion_info_command_creator();
MSyntax	motion_info_syntax_creator();

#endif
