# SMEKF Module
add_library(SMEKF MODULE src/SMEKF.c)
target_include_directories(SMEKF PRIVATE include ${Basilisk_INCLUDE_DIRS})
target_link_libraries(SMEKF PRIVATE ${Basilisk_LIBRARIES})
set_target_properties(SMEKF PROPERTIES PREFIX "")

# QUEST Module
add_library(questAttDet MODULE src/questAttDet.c)
target_include_directories(questAttDet PRIVATE include ${Basilisk_INCLUDE_DIRS})
target_link_libraries(questAttDet PRIVATE ${Basilisk_LIBRARIES})
set_target_properties(questAttDet PROPERTIES PREFIX "")

# Install targets
install(TARGETS SMEKF questAttDet LIBRARY DESTINATION ${Python_SITEARCH}/excite)
