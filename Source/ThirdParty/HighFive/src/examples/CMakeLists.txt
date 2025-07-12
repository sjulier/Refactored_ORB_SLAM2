set(core_examples
        compound_types.cpp
        broadcasting_arrays.cpp
        create_attribute_string_integer.cpp
        create_dataset_double.cpp
        create_datatype.cpp
        create_extensible_dataset.cpp
        create_large_attribute.cpp
        create_page_allocated_files.cpp
        readme_snippet.cpp
        read_write_dataset_string.cpp
        read_write_raw_ptr.cpp
        read_write_single_scalar.cpp
        read_write_std_strings.cpp
        read_write_vector_dataset.cpp
        read_write_vector_dataset_references.cpp
        renaming_objects.cpp
        select_by_id_dataset_cpp11.cpp
        select_partial_dataset_cpp11.cpp
)

set(span_examples
        read_write_std_span.cpp
)

set(easy_examples
        easy_attribute.cpp
        easy_dumpoptions.cpp
        easy_load_dump.cpp
)

set(boost_examples
        boost_multi_array_2D.cpp
        boost_multiarray_complex.cpp
        boost_ublas_double.cpp
)

set(eigen_examples
        eigen_matrix.cpp
        eigen_vector.cpp
        eigen_map.cpp
)

set(hl_hdf5_examples
        hl_hdf5_inmemory_files.cpp
)

set(parallel_hdf5_examples
        parallel_hdf5_collective_io.cpp
        parallel_hdf5_independent_io.cpp
)

set(half_float_examples
        create_dataset_half_float.cpp
)

function(compile_example example_source)
  get_filename_component(example_filename ${example_source} NAME)
  string(REPLACE ".cpp" "_bin" example_name ${example_filename})

  add_executable(${example_name} ${example_source})
  target_link_libraries(${example_name} PUBLIC HighFive HighFiveWarnings)
  if(${ARGC} GREATER_EQUAL 2)
    target_link_libraries(${example_name} PUBLIC ${ARGV1} ${ARGV2} ${ARGV3} ${ARGV4})
  endif()
endfunction()


foreach(example_source ${core_examples})
  compile_example(${example_source} HighFiveFlags)
endforeach()

foreach(example_source ${easy_examples})
  compile_example(${example_source} HighFiveFlags HighFiveOptionalDependencies)
endforeach()

if(HIGHFIVE_TEST_SPAN)
  foreach(example_source ${span_examples})
    compile_example(${example_source} HighFiveFlags)
  endforeach()
endif()

if(HIGHFIVE_TEST_BOOST)
  foreach(example_source ${boost_examples})
    compile_example(${example_source} HighFiveFlags HighFiveBoostDependency)
  endforeach()
endif()

if(HIGHFIVE_TEST_EIGEN)
  foreach(example_source ${eigen_examples})
    compile_example(${example_source} HighFiveFlags HighFiveEigenDependency)
  endforeach()
endif()

if(HDF5_IS_PARALLEL)
  foreach(example_source ${parallel_hdf5_examples})
    compile_example(${example_source})
  endforeach()
endif()

add_library(HighFiveHlHdf5Dependency INTERFACE)
find_package(HDF5 QUIET COMPONENTS HL NAMES HDF5_HL)
if(${HDF5_HL_FOUND})
    message("HDF5 HL: ${HDF5_HL_LIBRARIES}")
    target_link_libraries(HighFiveHlHdf5Dependency ${HDF5_HL_LIBRARIES})

    foreach(example_source ${hl_hdf5_examples})
      compile_examples(${example_source} HighFiveFlags HighFiveHlHdf5Dependency)
    endforeach()
endif()

# TODO Half-float examples
