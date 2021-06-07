#
# kilobot headers
#
# argos3/plugins/robots/kilobot/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR
    simulator/dynamics2d_kilobot_model.h
    simulator/insectbot_entity.h
    simulator/insectbot_measures.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)



if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}
    simulator/dynamics2d_kilobot_model.cpp
    simulator/insectbot_entity.cpp)

  # Compile the graphical visualization only if the necessary libraries have been found
  include(ARGoSCheckQTOpenGL)
  if(ARGOS_COMPILE_QTOPENGL)
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}
      simulator/qtopengl_kilobot.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT}
      simulator/qtopengl_kilobot.h
      simulator/qtopengl_kilobot.cpp)
  endif(ARGOS_COMPILE_QTOPENGL)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create kilobot plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_insectbot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_insectbot
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d)
if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_insectbot argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)


#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/kilobot/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}      DESTINATION include/argos3/plugins/robots/kilobot/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_insectbot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

