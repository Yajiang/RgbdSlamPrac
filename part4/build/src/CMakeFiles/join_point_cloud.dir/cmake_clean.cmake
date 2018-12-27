file(REMOVE_RECURSE
  "../../bin/join_point_cloud.pdb"
  "../../bin/join_point_cloud"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/join_point_cloud.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
