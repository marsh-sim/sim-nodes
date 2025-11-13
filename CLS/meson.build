project('CLS', 'cpp',
  version: '0.1.0',
  default_options: [
    'cpp_std=c++17',
    'warning_level=2',
    'buildtype=debug'
  ]
)

# Add DEBUG define for debug builds
add_project_arguments(
  get_option('buildtype') == 'debug' ? '-DDEBUG' : [],
  language: 'cpp'
)

# Include directories
inc_dirs = include_directories(
  'include',
  'include/Wittenstein',
  'include/mavlink/c_library_v2',
  'src'
)

# Wittenstein library sources
wittenstein_sources = files(
  'include/Wittenstein/CLSUtils.cc',
  'include/Wittenstein/WAS_Comms_Api.cc'
)

# MARSH connection sources
marsh_sources = files(
  'src/marshconnection.cc',
  'src/timer.cc'
)

# CLS data sources
cls_data_sources = files(
  'src/controlloadingdata.cc'
)

# Create Wittenstein static library
wittenstein_lib = static_library('wittenstein',
  wittenstein_sources,
  include_directories: inc_dirs
)

# Create MARSH static library
marsh_lib = static_library('marsh',
  marsh_sources,
  include_directories: inc_dirs
)

# Create CLS data static library
cls_data_lib = static_library('cls',
  cls_data_sources,
  include_directories: inc_dirs
)

# Main executable
clsstream_exe = executable('clsstream',
  'CLSstream.cc',
  include_directories: inc_dirs,
  link_with: [wittenstein_lib, marsh_lib, cls_data_lib],
  install: true
)
