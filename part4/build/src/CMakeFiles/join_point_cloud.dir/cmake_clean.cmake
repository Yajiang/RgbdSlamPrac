file(REMOVE_RECURSE
  "../../lib/libjoin_point_cloud.pdb"
  "../../lib/libjoin_point_cloud.a"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/join_point_cloud.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
