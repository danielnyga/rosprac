##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(owl_memory_converter)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS rosjava_build_tools message_generation)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Conversion.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})
