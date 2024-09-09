#!/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

# search all directories for .project files and process them as xml files, extracting the project name and the source files
# The project name is in projectDescription/name and the source files are in projectDescription/linkedResources/link/name
# write out a CMakeLists.txt file for each project

def process_project_file(project_file):
    project_name = ''
    source_files = []
    tree = ET.parse(project_file)
    root = tree.getroot()
    project_name = root.find('name').text
    if project_name == '':
        print('No project name found in %s' % project_file)
        return
    for element in root.iter('linkedResources'):
        for link in element.iter('link'):
            for name in link.iter('locationURI'):
                # append the name to the source_files list after removing the leading 'PARENT-1-PROJECT_LOC/' from path keeping the rest
                source_files.append(re.sub(r'PARENT-1-PROJECT_LOC/', '', name.text))

    return project_name, source_files

def process_cproject_file(project_file):
    adefines = []
    cdefines = []
    incdirs = []
    sourcedirs = []
    linker_script = ''
    tree = ET.parse(project_file)
    root = tree.getroot()
    for element in root.iter('cconfiguration'):
        # check if element.attrib['id'] has release
        if 'debug' in element.attrib['id']:
            print ("Skipping debug configuration")
            continue
        # find the cdtBuildSystem element
        for storagemodule in element.iter('storageModule'):
            if storagemodule.attrib['moduleId'] == 'cdtBuildSystem':
                for build in storagemodule.iter('configuration'):
                    print("Processing configuration %s" % build.attrib['name'])
                    for folder in build.iter('folderInfo'):
                        for toolchain in folder.iter('toolChain'):
                            for tool in toolchain.iter('tool'):
                                if tool.attrib['superClass'] == "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler":
                                    for option in tool.iter('option'):
                                        if option.attrib['valueType'] == 'definedSymbols':
                                            for symbol in option.iter('listOptionValue'):
                                                cdefines.append(symbol.attrib['value'])
                                        if option.attrib['valueType'] == 'includePath':
                                            for path in option.iter('listOptionValue'):
                                                incdirs.append(path.attrib['value'])
                                        if option.attrib['valueType'] == 'includeFiles':
                                            for file in option.iter('listOptionValue'):
                                                incdirs.append(file.attrib['value'])
                                if tool.attrib['superClass'] == "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.assembler":
                                    for option in tool.iter('option'):
                                        if option.attrib['valueType'] == 'definedSymbols':
                                            for symbol in option.iter('listOptionValue'):
                                                adefines.append(symbol.attrib['value'])
                                if tool.attrib['superClass'] == "com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker":
                                    for option in tool.iter('option'):
                                        if option.attrib['superClass'] == 'com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.linker.option.script':
                                            # example value is ${workspace_loc:/${ProjName}/STM32H7S3L8HX_ROMxspi2_app.ld} get the filename from this and append to ldflags
                                            linker_script = option.attrib['value'].split('/')[-1].split('}')[0]
                    for folder in build.iter('sourceEntries'):
                        for entry in folder.iter('entry'):
                            # if flags string has VALUE_WORKSPACE_PATH then add it to source search path
                            if 'VALUE_WORKSPACE_PATH' in entry.attrib['flags']:
                                sourcedirs.append(entry.attrib['name'])
    return adefines,cdefines,incdirs,sourcedirs,linker_script

def write_cmake_file(project_directory, project_name, source_files, assembly_source_files, adefines, cdefines, incdirs, linker_script):
    # replace ../../ with '' in incdirs
    incdirs = [re.sub(r'\.\./\.\./', '', x) for x in incdirs]
    # replace ../ with project_directory/ in incdirs
    for i in range(len(incdirs)):
        if incdirs[i].startswith('../'):
            incdirs[i] = os.path.join(project_directory, re.sub(r'\.\./', '', incdirs[i]))
    # if there are no source files, return
    print('project_name: %s in %s directory' % (project_name, project_directory))
    print('  Number of source files: %d' % (len(source_files)))
    print('  ADEFINES: %s' % (adefines))
    print('  CDEFINES: %s' % (cdefines))
    print('  INCDIRS: %s' % (incdirs))
    print('  Linker script: %s' % (linker_script))
    print()

    if len(source_files) == 0:
        return
    cmake_extra = ''
    if (len(assembly_source_files) > 0):
        source_files.extend(assembly_source_files)
        cmake_extra = 'set_source_files_properties(%s PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")\n\n' % ' '.join(assembly_source_files)

    with open('generated_%s.cmake' % project_name, 'w') as f:
        f.write('cmake_minimum_required(VERSION 3.27)\n')
        f.write('add_executable(%s)\n' % project_name)
        f.write('target_sources(%s PRIVATE \n    %s\n)\n\n' % (project_name, '\n    '.join(source_files)))
        if (len(adefines) > 0):
            f.write('target_compile_definitions(%s PRIVATE \n    %s\n)\n\n' % (project_name, '\n    '.join(adefines)))
        if (len(incdirs) > 0):
            f.write('target_include_directories(%s PRIVATE \n    %s\n)\n\n' % (project_name, '\n    '.join(incdirs)))
        if (linker_script != ''): 
            f.write('set_target_properties(%s PROPERTIES LINK_FLAGS "-T${CMAKE_CURRENT_SOURCE_DIR}/%s/%s")\n\n' % (project_name, project_directory, linker_script))
        if (len(cdefines) > 0):
            f.write('target_compile_definitions(%s PRIVATE \n    %s\n)\n\n' % (project_name, '\n    '.join(cdefines)))
        f.write(cmake_extra)

def process_project_directory(project_directory):
    project_name = ''
    source_files = []
    assembly_source_files = []
    adefines = []
    cdefines = []
    incdirs = []
    source_dirs = []
    linker_script = ''

    for file in os.listdir(project_directory):
        if file.endswith('.project'):
            project_name, source_files = process_project_file(os.path.join(project_directory, file))
    for file in os.listdir(project_directory):
        if file.endswith('.cproject'):
            adefines,cdefines,incdirs,source_dirs,linker_script = process_cproject_file(os.path.join(project_directory, file))
    
    # iterate all the source_dirs and check if they exist, if yes, then add all the .c files recursively to the source_files list
    for source_dir in source_dirs:
        if os.path.isdir(os.path.join(project_directory, source_dir)):
            for root, dirs, files in os.walk(os.path.join(project_directory, source_dir)):
                for file in files:
                    if file.endswith('.c'):
                        source_files.append(os.path.join(root, file))

    # check if there is a startup_*.s file  recursively in the project_directory somewhere
    # if there is, add it to the source_files list
    for root, dirs, files in os.walk(project_directory):
        for file in files:
            if file.startswith('startup_') and file.endswith('.s'):
                assembly_source_files.append(os.path.join(root, file))

    if project_name != '':
        write_cmake_file(project_directory, project_name, source_files, assembly_source_files, adefines, cdefines, incdirs,linker_script)
        return project_name

    return False


# find all subdirectories and process them            
if __name__ == '__main__':
    all_projects = []
    # iterate the directories under the current directory
    for project_directory in os.listdir('.'):
        if os.path.isdir(project_directory):
            tmp = process_project_directory(project_directory)
            if tmp:
                all_projects.append(tmp)
    
    # write out the list of all projects into generated_all_projects.cmake
    with open('generated_all_projects.cmake', 'w') as f:
        f.write('set(ALL_PROJECTS\n')
        for project in all_projects:
            f.write('    %s\n' % project)
        f.write(')\n')
