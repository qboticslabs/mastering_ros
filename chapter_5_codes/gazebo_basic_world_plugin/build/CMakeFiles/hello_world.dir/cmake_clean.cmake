FILE(REMOVE_RECURSE
  "CMakeFiles/hello_world.dir/hello_world.cc.o"
  "libhello_world.pdb"
  "libhello_world.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/hello_world.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
