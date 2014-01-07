
function(pr2_enable_rpath target)
   # Set ${target} with RPATH built in so that we can install it suid
   set_target_properties(${target} PROPERTIES SKIP_BUILD_RPATH FALSE)
   
   # Set the install RPATH to the install path
   set(RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")

   # If LD_LIBRARY_PATH is set, add it to the install RPATH
   #  this works in a normal catkin environment, but fails if the user unsets
   #  their LD_LIBRARY_PATH manually for some reason
   if(DEFINED ENV{LD_LIBRARY_PATH})
      set(RPATH "${RPATH}:$ENV{LD_LIBRARY_PATH}")
   endif()

   message("Install RPATH for ${target} is ${RPATH}")

   # Apply our computed RPATH to the target
   set_target_properties(${target} PROPERTIES INSTALL_RPATH ${RPATH})

   # Don't use the final RPATH in devel space
   set_target_properties(${target} PROPERTIES BUILD_WITH_INSTALL_RPATH FALSE)

endfunction()
